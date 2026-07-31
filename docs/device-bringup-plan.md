# Plan: guided device bring-up (skill + MCP)

> **Status (2026-07-31):** the measurement half shipped on branch `device-bringup` — see the
> **`device-bringup`** skill. No MCP: every read-only tool in §3 turned out to be a shell command
> (`curl localhost:1250/?action=getdevices`, read the vendordep JSON), and the actuating half in §3
> duplicates Tuner X, so it was cut. Steps 6–8 of §4 — the part Tuner X structurally cannot do,
> because it needs the ratio the *code* declares — are what got built, as an always-on logger plus
> a report script. §7 phases 0, 1 and 2 are dropped or moot as a result.

Goal: a student plugs in a Kraken, a CANcoder, a CANrange — and gets walked through the whole
bring-up by something with **live access to the hardware**. Check firmware, assign an ID, confirm
direction, fuse the encoder, zero it, verify the gear ratio, set soft limits, then generate the
subsystem code. Later: characterise and tune it.

This is what frc5712.com teaches, except it can see the robot.

---

## 1. What is already verified (do not re-derive)

Tested on 2026-07-31 against this template in simulation.

**Phoenix Diagnostic Server** — an HTTP JSON API on `localhost:1250`, started automatically by the
robot program. It prints `[phoenix-diagnostics] Server 2026.50.0 running on port: 1250` on every
run, sim included.

- `GET /?action=getdevices` **works in simulation**. Returned all 16 simulated devices with
  `ID`, `Model`, `CurrentVers` (firmware), `HardwareRev`, `BootloaderRev`, `SoftStatus`
  (`"Simulated Device."`), `IsPROLicensed`, `SupportsConfigs`, plus `BusUtilPerc`.
- Per-device actions `selftest`, `getconfigs`, `config`, `blink`, `setid` return
  `Error: -120, "Specified device was not found"` against simulated devices. Adding `&canbus=sim`
  or a device name does not help.
- **Correction (2026-07-31):** this note used to read that those actions were "recognised, since
  the server echoes the action rather than rejecting it." That inference was wrong — `action=
  bogusaction` gets the same echo and the same -120. The device lookup fails before the action is
  ever dispatched, so the response says nothing about which actions exist. Whether `blink` and
  `setid` work over HTTP is still **unknown** and still needs a real device.

> **The constraint that shapes everything below: inventory works without hardware; device
> operations need a real bus.** You cannot develop the interesting half of an MCP against sim if
> you go through the diagnostic server.

**CTRE CLI tools** (from docs.ctr-electronics.com/cli-tools.html):

| Tool | What it is |
| --- | --- |
| **Corvus** | *Generates mechanisms* — a code generator, **not** a device config tool. Common misconception; check whether its output is useful before building on it. |
| **Caniv** | Enumerate/configure **CANivore** devices only. No macOS. |
| **Owlet** | Converts `.hoot` logs to other formats. Useful for the tuning phase. |
| **Phoenix Diagnostic Server** | Backs Tuner X. No macOS. |
| **Passerine** | MIDI → CHRP. Irrelevant. |

---

## 2. Architecture

Three possible paths to the hardware. Pick per capability, don't force one.

**Path A — Diagnostic server HTTP.** Inventory, firmware versions, licensing, bus utilisation.
Read-only in practice. Sim-testable for inventory. On a real robot the server runs *on the
SystemCore*, so the MCP talks to `<robot-address>:1250`, not localhost.

**Path B — CTRE CLIs.** `Caniv` for CANivore-level setup, `Owlet` for log conversion. Niche.

**Path C — Robot-side bring-up OpMode + NetworkTables.** ← **the primary actuator.**

Path C wins for everything that touches a device:

- The robot program **already owns** the devices through `LoggedTalonFX` / `LoggedCANcoder`.
  Zeroing an encoder or applying a config is a one-line call in robot code, with no risk of
  fighting the running program for the bus.
- **It works identically in sim and on hardware**, so the whole MCP and skill can be built and
  tested with no robot on the bench — exactly what Path A cannot do.
- **Everything is already logged.** A bring-up session becomes a `.wpilog`, so it is reviewable
  and replayable with the machinery on the `advanced-replay` branch. A tuning run you can replay
  is worth far more than one you cannot.

So: **MCP = capabilities. Skill = curriculum.** The MCP exposes primitives; the skill holds the
teaching script, the order of steps, and what "good" looks like. That split is why the answer to
"skill or MCP?" is both.

---

## 3. Proposed MCP surface

Read-only (safe, no confirmation):

- `list_devices()` → id, model, firmware, licensed, bus, healthy
- `device_health(id)` → self-test output, faults, sticky faults
- `mechanism_state(name)` → live position/velocity/voltage/current from the robot's logged inputs
- `expected_firmware()` → what the pinned vendordep wants, to diff against actual

Actuating (**every one confirmation-gated**):

- `set_device_id(from, to)`
- `apply_config(name, configJson)`
- `zero_encoder(name)` → read absolute position, write `MagnetOffset`
- `nudge(name, volts, seconds)` → tiny bounded motion for direction/ratio checks
- `set_soft_limits(name, min, max)`

Orchestration:

- `start_session(mechanism)` / `record_step(...)` — so the transcript of a bring-up is saved
  alongside the log.

Transport to the robot: an NT4 client (robotpy `ntcore`) is the obvious choice. Alternative is a
small HTTP endpoint in robot code. **Decide this early — it is the main unknown.**

---

## 4. The bring-up script (arm as the worked example)

This is the skill's content. Each step states what the student does physically, what the tool
checks, and what failure looks like.

1. **Inventory.** Enumerate the bus. A brand-new device sits at **ID 0** — that is almost always
   the thing just plugged in. Flag duplicates.
2. **Firmware.** Diff `CurrentVers` against what the pinned Phoenix vendordep expects. Refuse to
   continue on a mismatch; wrong firmware wastes hours downstream.
3. **Identify + name.** Blink the device so the student can *see* which one it is, then assign the
   ID and the log name (`"Arm"`) that becomes the `LoggedTalonFX` field.
4. **Bus placement.** A CANcoder fused with `withRemoteCANcoder` **must be on the same CAN bus as
   its TalonFX.** Nothing in code stops you splitting them; it fails on hardware. Check it here.
5. **Direction.** Apply a tiny bounded voltage. Ask: did it move the way you expect? If not, flip
   `MotorOutput.Inverted` rather than negating things downstream.
6. **Sensor agreement.** Confirm motor position and CANcoder position move the *same* direction by
   the *expected ratio*. Disagreement means inversion or fusing is wrong — catch it now.
7. **Zero.** "Put the arm horizontal and confirm." Read absolute position, write `MagnetOffset`.
   Remember `refresh()` before `apply()` or you wipe the offset you just set.
8. **Gear ratio.** "Move the arm to 90° and confirm." Compare the encoder delta to the expected
   90°. If it is off by a clean factor, report the *measured* ratio — this is the single most
   common setup bug and the one students cannot diagnose alone.
9. **Soft limits.** Set them before anything moves under closed loop.
10. **Gravity sign.** For an arm, confirm `kG` pushes the right way at a known angle.
11. **Generate code.** Emit the `LoggedTalonFX` / `LoggedCANcoder` fields and the config block in
    this template's shape (see the `add-a-mechanism` skill). Evaluate whether **Corvus** output can
    seed this or whether generating directly is simpler.

Steps 5–8 are the heart of it. They are exactly what a mentor does by eye and what a student
cannot do alone.

---

## 5. Tuning (later phase)

Only after bring-up is solid. The template already logs applied volts, velocity, position,
closed-loop error and reference for every `LoggedTalonFX`, which is most of what characterisation
needs.

- **kS / kV / kA** — voltage ramp, then linear regression over the logged data.
- **kG** — hold at a known angle, read the holding voltage.
- **kP / kD** — iterate against a step response; score from the log, not by eye.
- Because runs are logged, a tuning session is **reproducible and replayable**. That is the piece
  hand-tuning never has.

---

## 6. Safety rails (non-negotiable)

- **Nothing moves without explicit human confirmation.** Every motion command is confirm-gated.
- **Bounded motion only**: low voltage, low current limit, short duration, soft limits set first.
- **The tool cannot see the robot.** Any step depending on physical state ("arm is horizontal")
  must be confirmed by the student, never assumed.
- **Writes to device IDs and configs are destructive.** Read current config, show the diff, then
  ask.
- **Disable must always work.** Never leave a mechanism commanded after a step ends.
- Log every action taken, so a bad session can be reconstructed.

---

## 7. Phases

| Phase | Deliverable | Needs hardware? |
| --- | --- | --- |
| 0 | **Spike:** confirm per-device diagnostic actions (`selftest`, `getconfigs`, `setid`) work against a real device from a laptop. This is the one thing sim could not answer. | **Yes** |
| 1 | Read-only MCP (`list_devices`, `expected_firmware`) + a `device-inventory` skill | No |
| 2 | Robot-side bring-up OpMode + NT command surface; decide NT4 vs HTTP transport | No |
| 3 | The walkthrough skill, arm first | Bench device |
| 4 | Code generation into this template's shape; evaluate Corvus | No |
| 5 | Characterisation and tuning off the logs | Bench device |

Phases 1, 2 and 4 are fully buildable with no robot. Start there.

---

## 8. Open questions

1. **Where does the diagnostic server live on a real robot**, and can a laptop MCP reach it while
   the robot program is running? (In sim it was local. On a SystemCore it will not be.)
2. **Does the diagnostic server conflict** with the running robot program over device access, or do
   they coexist? This decides how much of Path A is usable at all.
3. **NT4 from Python** — is robotpy `ntcore` the right client for the MCP process?
4. **Does Corvus generate anything usable** for the `LoggedTalonFX` wrapper shape, or is it swerve/
   Tuner-flavoured only?
5. Should the bring-up OpMode ship in the template (a `@Utility` OpMode) or live in a separate
   dev-only source set?

---

## 9. Why this matters

Every FRC team re-does this by hand, badly, every year — and the failures (wrong gear ratio, a
flipped inversion, an encoder zeroed at the wrong pose) all look identical from the driver station:
"the arm goes to the wrong place." A tool that walks a student through it, with live sensor access
and a logged transcript, is genuinely new. Ship the read-only half first and it is useful on day
one.
