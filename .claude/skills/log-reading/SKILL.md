---
name: log-reading
description: How to find and analyze this robot's logs — WPILib .wpilog files written by DataLogManager (drive telemetry + DS + NetworkTables) and Phoenix .hoot files written by the CANivore. Lists the real keys this code publishes, where logs live, and how to read them with AdvantageScope or a scripted DataLogReader. Use after a sim/match run to inspect what the robot did.
---

# Log Reading

This template is **logging-only** — there is no AdvantageKit and no log-replay. Two kinds of logs
get written:

| Format | Written by | What's in it |
| --- | --- | --- |
| **`.wpilog`** | WPILib `DataLogManager` + `DriverStation.startDataLog`, started in [Robot.java](src/main/java/frc/robot/Robot.java) | Every NetworkTables value change (incl. all `Drivetrain/*` telemetry), console output, DS state, joysticks |
| **`.hoot`** | Phoenix 6 / the CANivore, configured in [TunerConstants.java](src/main/java/frc/robot/generated/TunerConstants.java) (`new CANBus("canivore", "./logs/example.hoot")`) | Raw CAN traffic for every Phoenix device (drive/steer TalonFX, CANcoders, Pigeon2, arm, flywheel) at high rate |

Start with the `.wpilog` for "what did the robot think/do"; drop to the `.hoot` for low-level
device signals (applied volts, currents, closed-loop error, CAN health).

## Where logs live

| Source | Path |
| --- | --- |
| Sim `.wpilog` | [logs/](logs/) in the project dir — `WPILIB_<UTC-yyyyMMdd_HHmmss>.wpilog`. Newest is most recent. (Named `WPILIB_TBD_*` until the DS connects, then renamed.) |
| Sim `.hoot` | [logs/example.hoot](logs/) (path from `TunerConstants`). |
| Real robot `.wpilog` | A USB drive's `logs/` folder if attached, else `/home/systemcore/logs/` (SystemCore, **not** a roboRIO). |
| Real robot `.hoot` | Wherever the `CANBus` log path points; pull via Tuner X. |

## What this code actually publishes (the `.wpilog` keys)

[Telemetry.java](src/main/java/frc/robot/utils/Telemetry.java) publishes the swerve state to the
NetworkTables table `Drivetrain` every odometry update. `DataLogManager` logs NT entries with an
**`NT:` prefix**, so in the log file the keys are `NT:/Drivetrain/...`:

| Log key | Type | Meaning |
| --- | --- | --- |
| `NT:/Drivetrain/Pose` | `struct:Pose2d` | Odometry pose (blue-alliance origin) |
| `NT:/Drivetrain/Velocity` | `struct:ChassisVelocities` | Measured robot-relative chassis velocity |
| `NT:/Drivetrain/RawHeading` | `struct:Rotation2d` | Raw gyro yaw |
| `NT:/Drivetrain/ModuleStates` | `struct:SwerveModuleVelocity[]` | Per-module measured velocity + angle |
| `NT:/Drivetrain/ModuleTargets` | `struct:SwerveModuleVelocity[]` | Per-module commanded targets |
| `NT:/Drivetrain/ModulePositions` | `struct:SwerveModulePosition[]` | Per-module distance + angle (estimator inputs) |
| `NT:/Drivetrain/TranslationSpeedMps` | `double` | `hypot(vx, vy)` |
| `NT:/Drivetrain/RotationSpeedRadPerSec` | `double` | Yaw rate magnitude |
| `NT:/Drivetrain/OdometryPeriodSeconds` | `double` | Time between odometry samples |
| `NT:/Drivetrain/OdometryFrequencyHz` | `double` | `1 / OdometryPeriod` (≈250 Hz on CAN FD) |

Also present, from `DriverStation.startDataLog` and WPILib itself:

| Log key | Meaning |
| --- | --- |
| `DS:controlWord` | DS enabled/mode bits — use to find **enabled** transitions |
| `DS:opMode` | The selected OpMode id — which mode/routine was running |
| `DS:joystick` | Joystick/controller data |
| `NT:/FMSInfo/*` | `ControlWord`, `IsRedAlliance`, `MatchType`, `MatchNumber`, `EventName`, `GameData` |
| `messages` | Console / `DataLogManager.log(...)` output |

> **Want a new key in the log?** Publish it to NetworkTables — either add it to
> [Telemetry.java](src/main/java/frc/robot/utils/Telemetry.java), or `NetworkTableInstance`-publish
> it anywhere. `DataLogManager` captures every NT change automatically. There is no
> `Logger.recordOutput` / `@AutoLog` here.

**Finding auto/teleop start:** look at `DS:controlWord` (enabled bit + mode bits) and `DS:opMode`.
The first sample where the control word flips to enabled is the start of the active mode; `DS:opMode`
tells you which OpMode it was.

**Vision keys:** the robot does **not** republish Limelight data, but `DataLogManager` logs all NT,
so if the real Limelights (`limelight-br` / `limelight-bl`) are connected their
`/limelight-br/*` and `/limelight-bl/*` entries are captured. In **sim there is no Limelight**,
so expect none.

## Reading `.wpilog` — AdvantageScope (interactive)

Open the `.wpilog` in **AdvantageScope** ("Open Log"). It decodes the struct schemas embedded in the
log, so `Drivetrain/Pose` drops onto the 2D/3D field view and `ModuleStates`/`ModulePositions`
render on the swerve widget. Best when you don't yet know which keys matter. Tabular CSV export is
available for any selection.

## Reading `.wpilog` — scripted (`wpiutil` DataLogReader)

For agent-driven analysis, parse the WPILOG directly. `pip install robotpy-wpiutil` if outside the
robot JVM. Two-pass pattern (collect entry names, then values):

```python
from wpiutil.log import DataLogReader
import struct

path = "logs/WPILIB_20260626_233929.wpilog"
entries = {}
for r in DataLogReader(path):
    if r.isStart():
        d = r.getStartData()
        entries[d.entry] = (d.name, d.type)  # d.type e.g. "struct:Pose2d" or "double"

for r in DataLogReader(path):
    if r.isStart() or r.isFinish() or r.isControl() or r.isSetMetadata():
        continue
    name, typ = entries.get(r.getEntry(), ("", ""))
    if name == "NT:/Drivetrain/Pose":
        ts = r.getTimestamp() / 1e6
        x, y, theta = struct.unpack("<ddd", bytes(r.getRaw()))
        print(ts, x, y, theta)
```

Struct decoding cheat sheet (little-endian; **confirm against the `type` schema string** — these are
the 2027 `org.wpilib` types):

| Type | Bytes | `struct.unpack` |
| --- | --- | --- |
| `double` | 8 | `<d` |
| `int64` | 8 | `<q` |
| `boolean` | 1 | `<?` |
| `string` | var | `payload.decode("utf-8")` |
| `struct:Pose2d` | 24 | `<ddd` → (x, y, theta_rad) |
| `struct:Rotation2d` | 8 | `<d` → theta_rad |
| `struct:ChassisVelocities` | 24 | `<ddd` → (vx, vy, omega) |
| `struct:SwerveModuleVelocity` | 16 | `<dd` → (speed_mps, angle_rad) |
| `struct:SwerveModulePosition` | 16 | `<dd` → (distance_m, angle_rad) |

Array types (`struct:Foo[]`) are N back-to-back records of the element layout — divide the payload
length by the element size. AdvantageScope is easier for arrays; use Python for scalar time-series.

## Reading `.hoot` — Phoenix tooling

`.hoot` is CTRE's binary CAN log. Read it with:

- **Tuner X → Log Extractor** (GUI): open the `.hoot`, browse/plot signals, export CSV, or **convert
  to `.wpilog`** so you can open it in AdvantageScope alongside the DataLogManager log.
- **`owlet`** (CTRE's CLI converter, ships with Phoenix Tuner): `owlet <in>.hoot <out>.wpilog`.

Use `.hoot` when you need per-device truth the `Drivetrain/*` summary doesn't show: applied output
voltage, supply/stator current, closed-loop error/reference, device temperature, CAN bus utilization.

## Common analyses

- **"Did `DriveToPose` reach the goal?"** Plot `Drivetrain/Pose` (x, y, theta) over time; compare the
  end pose to the routine's goal in [AutonomousOpMode.java](src/main/java/frc/robot/opmodes/AutonomousOpMode.java).
- **"Did we stall / saturate?"** `Drivetrain/TranslationSpeedMps` near 0 while a command is active →
  cross-check applied volts / stator current in the `.hoot`.
- **"Which OpMode ran, and when did it enable?"** `DS:opMode` + `DS:controlWord`.
- **"Is odometry healthy?"** `Drivetrain/OdometryFrequencyHz` should sit near 250 (CAN FD) and be
  steady.
- **"Wheels fighting the target?"** Overlay `Drivetrain/ModuleStates` vs `ModuleTargets` per module.

## Don'ts

- Don't look for `/RealOutputs/*`, `/ReplayOutputs/*`, or `/AdvantageKit/*` keys — there's no
  AdvantageKit in this template.
- Don't expect a `_replay.wpilog` — there is no replay path. Re-run the sim instead (`run-sim` skill).
- Don't expect vision keys from a sim log — there's no vision sim.
