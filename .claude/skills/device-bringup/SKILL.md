---
name: device-bringup
description: Bring up a new mechanism on real hardware — measure its true gear ratio, sensor direction, CANcoder zero, gravity feedforward (kG) and velocity feedforward (kV) from a log, then write the measured numbers into the subsystem. Use when a Kraken/CANcoder is newly wired, when an arm drives to the wrong angle, when a flywheel misses its commanded speed, when asked to zero an encoder or verify a gear ratio, or after replacing a gearbox.
---

# Device bring-up

> File links below are relative to the **repo root**, not to this skill's directory.

The failures this catches — a wrong gear ratio, a flipped inversion, an encoder zeroed at the wrong
pose — all look identical from the driver station: *"the mechanism goes to the wrong place."* This
measures what the mechanism actually does so you can compare it to what the code claims.

## What this covers, and what Tuner X covers

Use **Phoenix Tuner X** for anything about a *device*: firmware, blinking an LED to identify it,
assigning a CAN ID, self-test, licensing. It ships with Phoenix and it is the right tool.

Use **this** for anything that needs to know what the *code* expects: the rotor-to-mechanism ratio
declared in the subsystem, the sign the closed loop assumes, the feedforward gains. Tuner X cannot
know those.

## 1. See what is on the bus

The running robot program hosts a Phoenix diagnostic server on port **1250** — the same one Tuner X
drives. [tools/devices.py](tools/devices.py) talks to it. **The robot or sim must be running.**

```powershell
python tools/devices.py list                   # local sim
python tools/devices.py --host 10.TE.AM.2 list # a real robot
```

```
16 devices
vendordep 26.50.0-alpha-1 expects firmware 26.50.0.x

  ID   0  Talon FX vers. C       26.50.0.0 (Phoenix 6)   <- ID 0 - factory default, probably the one you just plugged in
  ID  31  Talon FX vers. C       26.50.0.0 (Phoenix 6)
  ID  32  CANCoder vers. H       26.50.0.0 (Phoenix 6)
```

It flags **ID 0** (factory default — almost always the thing you just plugged in), duplicate IDs,
and any firmware whose **major** version does not match the pinned vendordep.

Only the major is compared, on purpose. An alpha vendordep's minor version runs ahead of every
released firmware: `26.50.0-alpha-1` has no matching `26.50.x` CRF and never will, because the 2027
alpha library runs on 2026-season (`26.x`) device firmware. Comparing more than the major flags
every device on the bus forever. Before hunting for a version, check what actually exists in
[CTRE's firmware-index.json](https://github.com/CrossTheRoadElec/Phoenix-Releases/blob/master/firmware-index.json)
— it also declares the newest season, e.g. `"Latest": {"v6": "2026"}`.

> In **simulation** this lists only devices the code constructed, so it cannot discover something
> new. On hardware it enumerates the real bus.

## 2. Identify it, then give it an ID

```powershell
python tools/devices.py blink 0      # which physical device is flashing?
python tools/devices.py setid 0 31   # that one becomes ID 31
```

| Action | Where it stands |
| --- | --- |
| `blink` | **Works**, verified on real hardware and against a simulated device (`Error=0`) |
| `setid` | **Works on real hardware** — verified on a real Talon FX (`41 -> 21`, `Error=0`, re-enumerated at the new ID, and a running robot program started seeing the device mid-run). The `-109` refusal is simulation-only: a simulated device has no non-volatile storage to write an ID into |
| `selftest` | Not available over HTTP: `-144 "This feature requires Tuner X."` |
| `getconfigs` | Not available over HTTP: `-116` |
| firmware update | Tuner X |

A device is addressed by **`model` + `canbus` + `id`, all three** — and `canbus` is the *empty
string* in simulation. Get any of them wrong and the server answers `-120 "Specified device was
not found"`, which reads like "unsupported action" but is not; `action=bogusaction` returns the
same thing. `devices.py` looks all three up for you.

`caniv.exe` (ships with Tuner X) does **not** help here — `--list`, `--blink` and `--flash` all act
on the CANivore adapter itself, not on devices attached to its bus.

Last thing before you write code: **a CANcoder used as a TalonFX's feedback source must be on the
same CAN bus as that TalonFX.** Nothing in code stops you splitting them; it just fails on
hardware.

## 3. Write a BARE subsystem — devices only, no config

Here is the trap: you cannot write the real subsystem yet. The gear ratio, the sensor direction and
the zero are exactly the numbers you do not know, and the powered sweep in step 6 drives the
mechanism *under closed loop using those numbers*. Guess them and the first powered move is the one
that breaks something.

So split it. `BringUp` only needs the devices to exist — not gains, not feedback config, not
commands:

```java
public class Wrist extends Mechanism {
  private final LoggedTalonFX motor = new LoggedTalonFX(41, TunerConstants.kCANBus, "Wrist");
  private final LoggedCANcoder encoder = new LoggedCANcoder(42, TunerConstants.kCANBus, "Wrist");
}
```

plus one line in [Robot.java](src/main/java/frc/robot/Robot.java): `public final Wrist wrist = new
Wrist();`. That is the whole thing. Nothing to get wrong, because there are no numbers in it yet.

Name the wrappers carefully — the names are what get matched:

| Wrapper name | Meaning |
| --- | --- |
| `LoggedTalonFX(31, bus, "Arm")` + `LoggedCANcoder(32, bus, "Arm")` | motor and its sensor — measured against each other, giving the gear ratio |
| `LoggedTalonFX(33, bus, "Arm/2")` | a **follower** of `"Arm"` — measured against the leader's rotor, so it should read ±1 |
| `LoggedTalonFX(21, bus, "Flywheel")` | motor with no sensor and no leader — checked as a velocity loop |

## 4. Measure it DISABLED, by hand

In sim, record a fixed-length disabled log with:

```powershell
./gradlew simulateJavaAgent -Pmode=disabled -PstopAfter=20
```

On the robot: deploy, and **leave it disabled**. Motors cannot actuate, sensors still read, and the log is
still written — so move the mechanism through as much of its range as you can *by hand* and you get
the gear ratio, the sensor direction and the zero with nothing powered and nothing to guess.

This is the step that makes the real subsystem writable. Do it before you set a single gain.

[BringUp.java](src/main/java/frc/robot/hardware/BringUp.java) runs every loop, in every mode, in
sim and on hardware, and records to the log:

| Key (under `/RealOutputs/BringUp/<name>/`) | Meaning |
| --- | --- |
| `SensorTravelRot` | Mechanism turns since the run started — how much travel backs the ratio |
| `MeasuredRatio` | `RotorTravel / SensorTravel`, **signed**, sampled at the furthest travel seen so far. `NaN` until 0.01 rot of travel |

A motor with neither a CANcoder nor a leader gets no `BringUp` keys — there is nothing to compare
its rotor against. `tools/bringup_report.py` checks those as velocity loops instead.

The report **discovers mechanisms from the log**; nothing is hardcoded. It also works out what kind
of mechanism each one is from what it did: holding voltage that follows `cos(angle)` is an arm,
the same holding voltage at every pose is an elevator, and it names the `GravityType` that matches.
That catches `Arm_Cosine` set on an elevator.

Everything is computed from logged inputs, so a bring-up run **replays** (see the `run-replay`
skill).

## Running it

**In sim** — the `Bring-Up` `@Utility` OpMode sweeps the arm through three poses while the flywheel
holds speed, so one run produces every number. Nothing physical can move, so this one auto-enables
and ends by itself:

```powershell
./gradlew simulateJavaAgent '-Pmode=utility:Bring-Up' -PstopAfter=14
python tools/bringup_report.py
```

**On real devices, add `-PhwSim` — and then you drive it, not the script.** `-PhwSim` talks to the
devices over CAN, so real motors turn. The run will **not** enable itself: it starts the Driver
Station, prints which OpMode to pick, and waits for you.

```powershell
./gradlew simulateJavaAgent -PhwSim '-Pmode=utility:Bring-Up'
```

Then, in the Driver Station: pick the OpMode, **clear the mechanism's path**, hit Enable — and keep
a hand on Disable, which is the stop button. Closing the Driver Station or Ctrl-C in the terminal
also stops the robot. Leave `-PstopAfter` off for hardware runs: it counts from program start, not
from when you enable, so it will usually fire in the middle of nothing.

> Every number in this skill is from one bench run on one motor. They show what good output looks
> like and what the failure modes are — they are **never** values to reuse. Your mechanism's ratio,
> kG, kS and kV are yours to measure. Reporting a number from this file as if you measured it is the
> one unforgivable bring-up error.

### Short version

1. Close Tuner X. Only one program can own the CANivore.
2. `python tools/devices.py list` — find it. `blink` — confirm which one.
3. `setid <old> <new>` — give it the ID the subsystem expects.
4. Strip the subsystem to devices + an explicit all-zero config. It cannot move, on purpose.
5. Disabled, hands on: move it, read the ratio and the sign.
6. Open loop volts: measure kV (spinner) or the lift voltage (arm).
7. Write those in. Add kP last, and only then let it move under closed loop.

Never skip 4. A motor keeps its **last** config, so "no config" means "whatever the previous
mechanism left behind" — a leftover flywheel kV once meant a 100 rot/s cruise on an arm.

### On hardware, in this order

Each step depends on less than the one after it, so a failure shows up before it can corrupt a
number you trust.

**1. Disabled, moved by hand.** Leave the robot **disabled** and move the mechanism through as much
of its range as you can. `BringUp` still logs. Nothing is powered, and it gets you the ratio, the
sign and the zero.

> **Do not touch the mechanism until you hear the device chirp on deploy.** Devices answer a second
> or two after the program starts, and any position seed lands at that moment — not when the program
> was launched. Move before the chirp and the pose you think you seeded is not the pose recorded.
> Measured: a device retained `422.75` rot across a restart and read it back 39 ms into the next run,
> well before the seed at 1.5 s.

Hold the *final* pose until the run ends, rather than trying to hit a moment. The last plateau in the
log is then the reading, so exact timing stops mattering — useful because whoever is running Gradle
cannot cue you mid-run.

**2. Open loop, held voltages.** The **Flywheel Volts** `@Utility` OpMode holds a few voltages and
the report fits kV as the *slope* of volts against rps:

```powershell
./gradlew simulateJavaAgent -PhwSim '-Pmode=utility:Flywheel Volts'
# pick the OpMode in the Driver Station, clear the path, enable; disable when it has settled
python tools/bringup_report.py
```

Do this **before** any closed-loop run. `VoltageOut` uses no gains, so this is the one measurement
that cannot be poisoned by a bad gain *or by a config that silently failed to apply* — the exact
failure the Tuner X warning above describes. It is also the low-voltage look at which way the shaft
actually turns. The closed loop, by contrast, needs a kV in order to measure a kV.

A single settled point folds kS into kV and reads high. Measured on a bare Kraken X60, per-point
volts/rps ran 0.1117 / 0.1105 / 0.1102 at 1 / 2 / 3 V while the slope gave **0.1094**.

**3. Closed loop, to confirm — not to measure.** Write the measured kV in, then run the **Bring-Up**
OpMode, clear the mechanism's path, and enable. Selecting the OpMode and enabling *is* the
confirmation gate; disable always stops it. Read the log from `/U/logs` (see the `log-reading`
skill). It should now settle on target: with kV 0.1094 it settled at 25.14 rps on 2.766 V against
2.756 V predicted, a 10 mV agreement between two independent methods.

### Which gains a bare bench can actually give you

| Gain | Set by | Measure on a bare shaft? |
| --- | --- | --- |
| `kV` | the motor itself | **Yes — transfers unchanged** |
| `kS` | friction of the whole assembly | Floor only; re-measure with the load on |
| `kA` | moment of inertia | **No** — a bare rotor's inertia is not the machine's |
| `kP` | inertia, plus how hard it should fight | **No** — tune last, with the load on |

kV is a property of the motor. Everything below it is a property of the *machine*, so measuring kA or
tuning kP against a bare shaft produces gains for a robot that does not exist. Do kV here; do the
rest after the wheel, arm or gearbox is bolted on, kP last.

kS deserves one caveat: the fit's intercept is an **extrapolation** below the slowest speed sampled,
and a 3-point fit cannot separate Coulomb friction (true kS) from viscous drag (which hides in kV).
For a kS you can lean on, ramp voltage slowly until the mechanism just breaks loose.

## Reading the report

```
ARM
  rotor:mechanism ratio = 49.99   (measured over 0.50 rot)
  to zero the CANcoder where it stopped, ADD -0.0825 to MagnetOffset
  holding voltage by pose:
       90.0 deg   +0.000 V
      180.0 deg   -0.332 V
       29.7 deg   +0.289 V
  kG (fit of V = kG*cos(angle) over those poses) = 0.332

FLYWHEEL
  commanded 25.00 rps, settled at 25.15 rps on +3.003 V
  direction: ok
  measured volts per rps = 0.1194   <- start kV here, then re-run
```

| Reading | What it means | What to do |
| --- | --- | --- |
| Ratio matches the subsystem's `GEAR_RATIO` | Gearing and fusing are right | Nothing |
| Ratio is a clean multiple off (25 vs 50, 3:1) | Wrong gearbox stage in the code, or the wrong stage measured | Set the code to the **measured** number |
| Ratio is **negative** | Motor and sensor disagree on direction | Flip `MotorOutput.Inverted`, or the CANcoder's `SensorDirection` — not both |
| Ratio is `NaN` | Under 0.01 rot of travel | Move it further |
| Holding voltage at 90° is not ~0 | The arm's zero is not where `Arm_Cosine` thinks horizontal is | Re-zero (below) |
| kG differs from the config | Gravity feedforward is wrong; the arm sags or climbs | Set `Slot0.kG` to the measured value, re-run, confirm it converges |
| Flywheel settles above the commanded speed | `kV` too high (feedforward overdriving) | Set `Slot0.kV` to `volts per rps`, re-run |
| Flywheel settles below | `kV` too low, or the wheel is loaded | Same, then check for rub |
| `direction: BACKWARDS` | Motor spins the wrong way | Flip `MotorOutput.Inverted` — do **not** negate the setpoint at the call site |
| Follower reads `-1` when it should be `+1` (or the reverse) | Follower wired or configured the wrong way | Flip the `Follower` request's `MotorAlignmentValue` (`Aligned` / `Opposed`) |
| Follower is not near ±1 at all | It is not actually following — wrong leader ID, or it is being commanded separately | Check the `Follower` control request |
| kG line says ELEVATOR on an arm (or the reverse) | `Slot0.GravityType` is set to the wrong kind | Match `GravityType` to what the poses measured |

kG and kV are measured against whatever gains are currently loaded, so they move a little as you
correct them. **Re-run until the number stops changing** — two runs agreeing within a few percent
is done.

## Zeroing a CANcoder

1. Put the mechanism in the pose that should read zero (for the arm: **horizontal**, because
   `Arm_Cosine` measures from there) and confirm it by eye.
2. Negate `/Hardware/CANcoder/<name>/AbsolutePositionRot` at the end of the log — that is the
   delta, because absolute position already has the current `MagnetOffset` applied.
   `tools/bringup_report.py` prints it for you.
3. **Add** it to the existing `MagnetOffset` in Tuner X, or in code:

```java
CANcoderConfiguration config = new CANcoderConfiguration();
encoder.device().getConfigurator().refresh(config);   // refresh FIRST
config.MagnetSensor.MagnetOffset += delta;
encoder.device().getConfigurator().apply(config);
```

`refresh()` before `apply()` is not optional — `apply()` writes **every** field, so building a
fresh config object silently wipes the offset you are trying to set. Same trap as the
`AbsoluteSensorDiscontinuityPoint` block in [Arm.java](src/main/java/frc/robot/subsystems/arm/Arm.java).

## Writing the results into the code

Put the measured numbers where the mechanism is configured, not in a spreadsheet:

- gear ratio → the subsystem's `GEAR_RATIO`, used by both `Feedback.RotorToSensorRatio` and the
  simulation model, so one wrong number can't hide in only one of them
- `kG`, `kV`, `kS` → `config.Slot0.*`
- soft limits → `config.SoftwareLimitSwitch.*`, **before** anything runs closed-loop

Then deploy and re-run the sweep to confirm.

## Safety

- Nothing here moves on its own. The **Bring-Up** OpMode does — selecting it and enabling is the
  confirmation, and disable always works.
- Do the disabled, hand-moved pass first. It gets the ratio, the sign and the zero with no risk.
- Set soft limits before the first closed-loop move, not after.
- The tool cannot see the robot. Any claim about physical state ("the arm is horizontal") is yours
  to confirm, never the tool's to assume.
- **Check the sign before naming a cause.** A log showing accelerate-then-stop looks like a mechanism
  falling, but gravity can only move it the direction gravity pulls. Establish which sign is "down" —
  cut the power and see which way it drifts — before blaming gravity. Got this wrong once on a real
  arm: the travel was positive, positive turned out to be *up*, so it was never a fall.

## Is this hardware, or just sim?

Phoenix loads either its hardware or its simulation natives at startup, and `Utils.isSimulation()`
reports which — so despite the name, it is **false** in hardware-attached simulation:

| | pure sim | hardware-attached sim | SystemCore |
| --- | --- | --- | --- |
| `RobotBase.isReal()` | false | false | true |
| `Utils.isSimulation()` (Phoenix) | **true** | **false** | false |
| `device.isConnected()` | true always | true iff the device answers | true iff the device answers |

`isConnected()` is the per-device check, and it means nothing in pure sim — a simulated device
answers even at a CAN ID nothing is configured for. Get into hardware-attached sim with
`./gradlew simulateJava -PhwSim` (verified: `Utils.isSimulation()` really does report `false`, and
the console names the CANivore by USB serial). It composes with `-Pheadless` and `-Pmode=...`.

**Close Phoenix Tuner X first, and make sure no earlier robot JVM is still alive.** Only one process
can own the CANivore. While something else holds it the robot program still *receives* status
signals — devices look connected, positions update — but every `apply()` comes back `TxFailed`, so
configs never land and gains stay at whatever the device already had. Measured, not guessed. The
tell is in the console:

```
[phoenix-diagnostics] Server ... first attempt to open server failed at port 1250
TalonFX 21 failed to configure after 5 attempts (TxFailed)
```

Port 1250 already taken means another owner. On Windows: `Get-NetTCPConnection -LocalPort 1250`, and
check for a stray `java.exe` — stopping a Gradle sim can leave the robot JVM running.

Do **not** use `CANBus.getStatus()` or `isNetworkFD()`. With nothing plugged in — and even with a
nonsense bus name — both report a healthy FD bus (`Status=OK`, `isNetworkFD=true`). The failure
only ever surfaces as a `[phoenix] CANbus Failed to Connect` console line.

**The sim plants cannot fight a real device.** With the hardware natives loaded every
`TalonFXSimState` setter returns `NotSupported` and does nothing. They do waste a little CPU, since
`getMotorVoltage()` reads 0.0, so gate `updateSimulation` on the table above if you care.

## Limits

- **Sim cannot validate feedback configuration.** The arm's sim plant writes the rotor and the
  CANcoder independently, so `RotorToSensorRatio`, fused-vs-remote, magnet offset and sensor
  inversion never enter the control path a sim run exercises. A green sim proves the control logic;
  it does not prove the device config. Verify that class of change on hardware.
- **Sim cannot test follower direction.** Phoenix slaves a simulated follower's rotor to its
  leader and ignores `MotorAlignmentValue`, so a reversed follower still reads `+1` in sim.
  Measured, not assumed: an `Opposed` follower logged a rotor position byte-identical to its
  leader's. Verify follower direction on hardware.
- kA needs a voltage ramp, not held steps — this measures kG, kV and (from the open-loop fit's
  intercept) kS only. Treat a bare-shaft kS as a floor.
- Fusing a CANcoder (`withFusedCANcoder`) needs a Phoenix Pro license. Unlicensed, it falls back to
  remote and `RotorToSensorRatio` goes unused — declare it anyway so the number is recorded.
