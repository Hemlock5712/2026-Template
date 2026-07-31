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

The robot program runs a Phoenix diagnostic server on port **1250**. It answers plain HTTP while
the program is running — in sim and on the robot — so inventory needs no tool at all:

```bash
curl -s "http://localhost:1250/?action=getdevices"        # sim
curl -s "http://10.TE.AM.2:1250/?action=getdevices"       # on the robot
```

```
ID= 31  Talon FX vers. C   fw=26.50.0.0 (Phoenix 6)  pro=True
ID= 32  CANCoder vers. H   fw=26.50.0.0 (Phoenix 6)  pro=True
```

Each entry has `ID`, `Model`, `CurrentVers` (firmware), `IsPROLicensed` and `SupportsConfigs`, plus
a bus-wide `BusUtilPerc`. **A brand-new device sits at ID 0** — that is almost always the thing you
just plugged in. Duplicate IDs show up here too.

**Firmware check:** diff `CurrentVers` against the pinned vendordep in
[vendordeps/](vendordeps/) — `26.50.0-alpha-1` there means `26.50.0.0` on the device. Fix a
mismatch before anything else; wrong firmware wastes hours further down.

> In **simulation** this lists only devices the code constructed, so it cannot discover something
> new. On hardware it enumerates the real bus.

Per-device actions (`selftest`, `getconfigs`, `setid`, `blink`) return `Error: -120` in sim. Do not
read that as "supported but no device": `action=bogusaction` returns exactly the same thing, so the
response says nothing about which actions exist. **Whether they work over HTTP against real
hardware is untested.** Until someone checks, blink and set IDs in Tuner X.

## 2. The one-time device setup (Tuner X)

1. Update firmware to match the vendordep, if step 1 found a mismatch.
2. Blink it so you can see which physical device you're about to configure, then assign the CAN ID.
3. **A CANcoder used as a TalonFX's feedback source must be on the same CAN bus as that TalonFX.**
   Nothing in code stops you splitting them; it just fails on hardware.

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
holds speed, so one run produces every number:

```powershell
./gradlew simulateJavaAgent '-Pmode=utility:Bring-Up' -PstopAfter=14
python tools/bringup_report.py
```

**On hardware, without moving anything** — leave the robot **disabled** and move the mechanism by
hand through as much of its range as you can. `BringUp` still logs. This is the safe first pass and
it gets you the ratio, the sign and the zero.

**On hardware, under power** — deploy, select the **Bring-Up** utility OpMode, clear the mechanism's
path, and enable. Selecting the OpMode and enabling *is* the confirmation gate; disable always
stops it. Read the log from `/U/logs` (see the `log-reading` skill) and run the same report.

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
`./gradlew simulateJava -PhwSim`.

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
- kS and kA need a voltage ramp at more than one speed — this measures kG and kV only.
- Fusing a CANcoder (`withFusedCANcoder`) needs a Phoenix Pro license. Unlicensed, it falls back to
  remote and `RotorToSensorRatio` goes unused — declare it anyway so the number is recorded.
