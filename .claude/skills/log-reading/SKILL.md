---
name: log-reading
description: How to find and analyze this robot's logs — AdvantageKit .wpilog files (drive telemetry + DS + system stats) and Phoenix .hoot files written by the CANivore. Lists the real keys this code logs, where logs live, and how to read them with AdvantageScope or a scripted DataLogReader. Use after a sim/match run to inspect what the robot did.
---

# Log Reading

> File links below are relative to the **repo root**, not to this skill's directory.

This template logs with **AdvantageKit, logging-only** — no IO layers and no log-replay. Two kinds
of logs get written:

| Format | Written by | What's in it |
| --- | --- | --- |
| **`.wpilog`** | AdvantageKit (`WPILOGWriter`, wired in [Robot.java](src/main/java/frc/robot/Robot.java)) | Everything passed to `Logger.recordOutput` (all `Drivetrain/*` telemetry), DS state + joysticks, Systemcore system stats, console output |
| **`.hoot`** | Phoenix 6 / the CANivore, configured in [TunerConstants.java](src/main/java/frc/robot/generated/TunerConstants.java) (`new CANBus("canivore", "./logs/example.hoot")`) | Raw CAN traffic for every Phoenix device (drive/steer TalonFX, CANcoders, Pigeon2, arm, flywheel) at high rate |

Start with the `.wpilog` for "what did the robot think/do"; drop to the `.hoot` for low-level
device signals (applied volts, currents, closed-loop error, CAN health). Note the `.wpilog` records
at the **50 Hz loop rate** — AdvantageKit keeps one value per key per loop — so high-rate analysis
(e.g. 250 Hz odometry detail) belongs in the `.hoot`.

## Where logs live

| Source | Path |
| --- | --- |
| Sim `.wpilog` | [logs/](logs/) in the project dir — `akit_<random>.wpilog`, renamed to `akit_<yy-MM-dd_HH-mm-ss>.wpilog` once the time is known. Newest is most recent. |
| Sim `.hoot` | **None is produced.** The path in `TunerConstants` only takes effect with real Phoenix hardware on the bus — a sim run leaves `logs/` with `.wpilog` files only. Don't go looking for applied volts / stator current in sim; they aren't recorded anywhere. |
| Real robot `.wpilog` | USB drive `/U/logs` (the `WPILOGWriter` default; pass a path to change it). |
| Real robot `.hoot` | Wherever the `CANBus` log path points; pull via Tuner X. |

Everything logged is also mirrored **live** to NetworkTables under `/AdvantageKit/...`
(`NT4Publisher`), so AdvantageScope can watch the same keys in real time.

## What this code actually logs (the `.wpilog` keys)

[Telemetry.java](src/main/java/frc/robot/utils/Telemetry.java) logs the swerve state once per loop
(sampled from the drivetrain by [DriveMechanism](src/main/java/frc/robot/subsystems/DriveMechanism.java)).
Outputs land under a **`/RealOutputs/` prefix**:

| Log key | Type | Meaning |
| --- | --- | --- |
| `/RealOutputs/Drivetrain/Pose` | `struct:Pose2d` | Odometry pose (blue-alliance origin) |
| `/RealOutputs/Drivetrain/Velocity` | `struct:ChassisVelocities` | Measured robot-relative chassis velocity |
| `/RealOutputs/Drivetrain/RawHeading` | `struct:Rotation2d` | Raw gyro yaw |
| `/RealOutputs/Drivetrain/ModuleStates` | `struct:SwerveModuleVelocity[]` | Per-module measured velocity + angle |
| `/RealOutputs/Drivetrain/ModuleTargets` | `struct:SwerveModuleVelocity[]` | Per-module commanded targets |
| `/RealOutputs/Drivetrain/ModulePositions` | `struct:SwerveModulePosition[]` | Per-module distance + angle (estimator inputs) |
| `/RealOutputs/Drivetrain/TranslationSpeedMps` | `double` | `hypot(vx, vy)` |
| `/RealOutputs/Drivetrain/RotationSpeedRadPerSec` | `double` | Yaw rate magnitude |
| `/RealOutputs/Drivetrain/OdometryPeriodSeconds` | `double` | Time between odometry samples |
| `/RealOutputs/Drivetrain/OdometryFrequencyHz` | `double` | `1 / OdometryPeriod` (≈250 Hz on CAN FD) |
| `/RealOutputs/Arm/AngleDegrees` | `double` | Measured arm angle. **0° = straight out horizontally** (the `Arm_Cosine` frame), so the presets read: scoring ≈ 30°, **stow = 90°**, intake = 180°. Stow is not 0. |
| `/RealOutputs/Arm/TargetDegrees` | `double` | Angle the arm is driving toward — graph against `AngleDegrees` |
| `/RealOutputs/Arm/AtTarget` | `boolean` | `Arm.isAtTarget()`; what the stow autos wait on |
| `/RealOutputs/Flywheel/SpeedRps` | `double` | Measured wheel speed, rotations/sec (target is 25) |
| `/RealOutputs/Flywheel/AtTarget` | `boolean` | `Flywheel.isAtTarget()` |
| `/RealOutputs/Superstructure/Scoring` | `boolean` | True while the StateMachine demo is in its Scoring state. **Only written by the "State Machine (no driving)" teleop** — it never appears in an autonomous log. |

Also present, logged by AdvantageKit itself (all verified in a real sim log):

| Log key | Meaning |
| --- | --- |
| `/DriverStation/Enabled`, `/DriverStation/RobotMode` | Enabled flag + mode — use to find **enabled** transitions |
| `/DriverStation/OpMode`, `/DriverStation/OpModeId` | The selected OpMode (name + id) — which mode/routine was running |
| `/DriverStation/Joystick0..5/*` | Joystick/controller data (axes, buttons, POVs) |
| `/DriverStation/MatchType`, `MatchNumber`, `EventName`, `GameData`, `AllianceStation` | Match info |
| `/SystemStats/*` | Systemcore health: `BatteryVoltage`, `CPU/*`, `Memory/*`, `IMU/*`, `Network/CAN0..4/*`, `Faults/*` |
| `/RealOutputs/Console` | Captured console output (`System.out` + errors) |
| `/RealOutputs/Logger/*`, `/RealOutputs/LoggedRobot/*` | AdvantageKit's own timing diagnostics |
| `/RealMetadata/ProjectName` | Metadata recorded at startup |

> **Want a new key in the log?** Call `Logger.recordOutput("MySubsystem/MyKey", value)` from the
> main loop — anywhere in a subsystem, command, or OpMode. Or annotate a getter/field with
> `@AutoLogOutput` on any object reachable from `Robot`'s fields (`AutoLogOutputManager.addObject`
> is wired in `Robot`). **NetworkTables topics are NOT auto-recorded** — publishing to NT alone no
> longer puts a value in the log.

**Finding auto/teleop start:** `/DriverStation/Enabled` flips true at the start of the active mode;
`/DriverStation/OpMode` tells you by name which OpMode was selected.

**Vision keys:** LimelightLib's NT tables (`/limelight-br/*` raw results, `/limelight_telemetry/*`
accepted/rejected estimates, `/limelightshared/robot_orientation_set`) are **live-only now** — view
them in AdvantageScope while connected, but they are not recorded to the `.wpilog` (nothing routes
them through `Logger`). In **sim there is no Limelight**, so they're empty anyway.

## Reading `.wpilog` — AdvantageScope (interactive)

Open the `.wpilog` in **AdvantageScope** ("Open Log"). It decodes the struct schemas embedded in the
log, so `Drivetrain/Pose` drops onto the 2D/3D field view and `ModuleStates`/`ModulePositions`
render on the swerve widget; the Console tab shows `/RealOutputs/Console`. Best when you don't yet
know which keys matter. Tabular CSV export is available for any selection.

## Reading `.wpilog` — scripted (`wpiutil` DataLogReader)

For agent-driven analysis, parse the WPILOG directly. `pip install robotpy-wpiutil` if outside the
robot JVM. Two-pass pattern (collect entry names, then values):

```python
from wpiutil.log import DataLogReader
import struct

path = "logs/akit_26-07-25_01-34-10.wpilog"
entries = {}
for r in DataLogReader(path):
    if r.isStart():
        d = r.getStartData()
        entries[d.entry] = (d.name, d.type)  # d.type e.g. "struct:Pose2d" or "double"

for r in DataLogReader(path):
    if r.isStart() or r.isFinish() or r.isControl() or r.isSetMetadata():
        continue
    name, typ = entries.get(r.getEntry(), ("", ""))
    if name == "/RealOutputs/Drivetrain/Pose":
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
  to `.wpilog`** so you can open it in AdvantageScope alongside the AdvantageKit log.
- **`owlet`** (CTRE's CLI converter, ships with Phoenix Tuner): `owlet <in>.hoot <out>.wpilog`.

Use `.hoot` when you need per-device truth the `Drivetrain/*` summary doesn't show: applied output
voltage, supply/stator current, closed-loop error/reference, device temperature, CAN bus utilization.

## Common analyses

- **"Did `DriveToPose` reach the goal?"** Plot `Drivetrain/Pose` (x, y, theta) over time; compare the
  end pose to the routine's goal in [AutonomousOpMode.java](src/main/java/frc/robot/opmodes/AutonomousOpMode.java).
- **"Did we stall / saturate?"** `Drivetrain/TranslationSpeedMps` near 0 while a command is active →
  cross-check applied volts / stator current in the `.hoot`.
- **"Which OpMode ran, and when did it enable?"** `/DriverStation/OpMode` + `/DriverStation/Enabled`.
- **"Is odometry healthy?"** `Drivetrain/OdometryFrequencyHz` should sit near 250 (CAN FD) and be
  steady.
- **"Wheels fighting the target?"** Overlay `Drivetrain/ModuleStates` vs `ModuleTargets` per module.
- **"Brownout / CAN trouble?"** `/SystemStats/BatteryVoltage`, `/SystemStats/Faults/*`,
  `/SystemStats/Network/CAN0..4/*`.

## Don'ts

- Don't look for `NT:`-prefixed or `DS:`-prefixed keys — those were the old DataLogManager format.
  This template's keys live under `/RealOutputs/`, `/DriverStation/`, `/SystemStats/`.
- Don't look for `/ReplayOutputs/*` or expect a `_replay.wpilog` — logging-only, no replay path.
  Re-run the sim instead (`run-sim` skill).
- Don't expect NT topics in the log — only `Logger.recordOutput` / `@AutoLogOutput` values are
  recorded (that includes the Limelight NT tables: live-only).
- Don't expect vision keys from a sim log — there's no vision sim.
