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

## The one-time device setup (Tuner X)

Do this before anything below. A new device ships at **CAN ID 0** — that's almost always the one
you just plugged in.

1. Firmware — match every device to the pinned Phoenix vendordep version.
2. Blink it so you can see which physical device you're about to configure, then assign the CAN ID.
3. **A CANcoder used as a TalonFX's feedback source must be on the same CAN bus as that TalonFX.**
   Nothing in code stops you splitting them; it just fails on hardware.

## Measuring: `BringUp`

[BringUp.java](src/main/java/frc/robot/hardware/BringUp.java) runs every loop, in every mode, in
sim and on hardware. It pairs a `LoggedTalonFX` with a `LoggedCANcoder` **by log name** — both
called `"Arm"` — and records to the log:

| Key (under `/RealOutputs/BringUp/<name>/`) | Meaning |
| --- | --- |
| `SensorTravelRot` | Mechanism turns since the run started — how much travel backs the ratio |
| `MeasuredRatio` | `RotorTravel / SensorTravel`, **signed**, sampled at the furthest travel seen so far. `NaN` until 0.01 rot of travel |

A motor with no matching CANcoder (the flywheel) gets no `BringUp` keys — there is nothing to
compare its rotor against. Check those from `/Hardware/TalonFX/<name>/` directly.

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

AdvantageKit already logs it — check `/SystemStats/Network/CAN*/Available` and `RX/Packets`. In
pure simulation every interface reads `Available: false` with zero packets.

Do **not** use `CANBus.getStatus()` or `isNetworkFD()` for this. Phoenix fakes a healthy bus in
simulation: with nothing plugged in they return `OK` and `true`.

## Limits

- **Sim cannot validate feedback configuration.** The arm's sim plant writes the rotor and the
  CANcoder independently, so `RotorToSensorRatio`, fused-vs-remote, magnet offset and sensor
  inversion never enter the control path a sim run exercises. A green sim proves the control logic;
  it does not prove the device config. Verify that class of change on hardware.
- kS and kA need a voltage ramp at more than one speed — this measures kG and kV only.
- Fusing a CANcoder (`withFusedCANcoder`) needs a Phoenix Pro license. Unlicensed, it falls back to
  remote and `RotorToSensorRatio` goes unused — declare it anyway so the number is recorded.
