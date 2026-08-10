---
name: log-reading
description: How to find and analyze this robot's logs — AdvantageKit .wpilog files (drive telemetry + DS + system stats) and Phoenix .hoot files written by the CANivore. Lists the real keys this code logs, where logs live, and how to read them with AdvantageScope or a scripted DataLogReader. Use after a sim/match run to inspect what the robot did.
---

# Log Reading

> File links below are relative to the **repo root**, not to this skill's directory.

This template logs with **AdvantageKit, logging-only** — no IO layers, but logs *can* be replayed
through changed code (`run-replay` skill). Two kinds of logs get written:

| Format | Written by | What's in it |
| --- | --- | --- |
| **`.wpilog`** | AdvantageKit (`WPILOGWriter`, wired in [Robot.java](src/main/java/frc/robot/Robot.java)) | Everything passed to `Logger.recordOutput` (all `Drivetrain/*` telemetry), DS state + joysticks, Systemcore system stats, console output |
| **`.hoot`** | Phoenix 6 / the CANivore, on by default on real hardware. Path and on/off are `SignalLogger.setPath` / `.start()` / `.enableAutoLogging()` — **not** a `CANBus` argument | Raw CAN traffic for every Phoenix device (drive/steer TalonFX, CANcoders, Pigeon2, arm, flywheel) at high rate |

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
| Real robot `.hoot` | Wherever `SignalLogger.setPath` points (default is a USB drive, else `/home/systemcore/logs`); pull via Tuner X. |

> **Trap:** `new CANBus(name, hootPath)` does *not* configure hoot logging — it calls
> `HootReplay.loadFile(hootPath)` and **replays** that file instead of reading the real bus.
> `TunerConstants` used to do this; it only ever worked because the file was missing and the
> failing load was ignored.

Everything logged is also mirrored **live** to NetworkTables under `/AdvantageKit/...`
(`NT4Publisher`), so AdvantageScope can watch the same keys in real time.

## What this code actually logs (the `.wpilog` keys)

Two kinds of key, and the difference matters:

- **Inputs** — everything read from hardware, at the top level (`/Drivetrain/Pose`,
  `/Hardware/TalonFX/Arm/VelocityRps`). Written by `LoggedHardware.refreshAll()`. These are what
  replay feeds back in.
- **Outputs** — anything the code computed, under **`/RealOutputs/`**. In a replay log you also get
  **`/ReplayOutputs/`**: the same keys recomputed by the current code. Graph the two against each
  other to see what a change would have done. See the **`run-replay`** skill.

[LoggedSwerveDrivetrain](src/main/java/frc/robot/hardware/LoggedSwerveDrivetrain.java) logs the
swerve state once per loop. Note `Pose`, `Velocity` and the module arrays are **inputs** now, so
they have no `/RealOutputs/` prefix; the derived speeds below still do:

| Log key | Type | Meaning |
| --- | --- | --- |
| `/Drivetrain/Pose` | `struct:Pose2d` | Odometry pose (blue-alliance origin) |
| `/Drivetrain/Velocity` | `struct:ChassisVelocities` | Measured robot-relative chassis velocity |
| `/Drivetrain/RawHeading` | `struct:Rotation2d` | Raw gyro yaw |
| `/Drivetrain/ModuleVelocities` | `struct:SwerveModuleVelocity[]` | Per-module measured velocity + angle |
| `/Drivetrain/ModuleTargets` | `struct:SwerveModuleVelocity[]` | Per-module commanded targets |
| `/Drivetrain/ModulePositions` | `struct:SwerveModulePosition[]` | Per-module distance + angle (estimator inputs) |
| `/RealOutputs/Drivetrain/TranslationSpeedMps` | `double` | `hypot(vx, vy)` |
| `/RealOutputs/Drivetrain/RotationSpeedRadPerSec` | `double` | Yaw rate magnitude |
| `/RealOutputs/Drivetrain/OdometryPeriodSeconds` | `double` | Time between odometry samples |
| `/RealOutputs/Drivetrain/OdometryFrequencyHz` | `double` | `1 / OdometryPeriod` (≈250 Hz on CAN FD) |
| `/RealOutputs/Arm/AngleDegrees` | `double` | Measured arm angle. **0° = straight out horizontally** (the `Arm_Cosine` frame), so the presets read: scoring ≈ 30°, **stow = 90°**, intake = 180°. Stow is not 0. |
| `/RealOutputs/Arm/TargetDegrees` | `double` | Angle the arm is driving toward — graph against `AngleDegrees` |
| `/RealOutputs/BringUp/Arm/MeasuredRatio` | `double` | Measured rotor:mechanism ratio, signed, sampled at the furthest travel so far. `NaN` until the arm has moved. See the `device-bringup` skill |
| `/RealOutputs/Flywheel/SpeedRps` | `double` | Measured wheel speed, rotations/sec (target is 25) |
| `/RealOutputs/Flywheel/AtTarget` | `boolean` | `Flywheel.isAtTarget()` — measured speed within tolerance of 25 rps. The arm has no equivalent key: it has three poses, so it has `atVertical()` / `atHorizontal()` / `atScoring()` instead. Graph `Arm/AngleDegrees` against `Arm/TargetDegrees` |
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

**Vision keys:** `Hardware/Limelight/<name>/*` holds the whole frame, ~55 keys per camera, in three
groups of parallel arrays tied together by frame index:

| Group | Entries per frame | Examples |
| --- | --- | --- |
| Frame | 1 | `TxDegrees`, `TargetDistanceMeters`, `CaptureLatencyMs`, `Imu*` (yaw/pitch/roll, gyro Z, accel XYZ) |
| Estimate | 2 (MegaTag1, MegaTag2) | `Poses`, `TagCounts`, `TagSpanMeters`, `StdDevX/Y/Theta`, `RejectionFlags` |
| Tag | one per tag seen | `TagIds`, `TagAmbiguity`, `RobotPoseTargetSpace`, `TargetPoseRobotSpace`, `RobotPoseFieldSpaceMegaTag2` |

All of it is an **input**, so a formula you write next season can use a value you never consumed this
season. `Vision/<name>/*` alongside it is our *decisions* (`Accepted`, `AcceptedStdDevXY`) — outputs,
which recompute on replay.

LimelightLib's own NT tables (`/limelight-br/*`, `/limelight_telemetry/*`) are still live-only —
nothing routes NT topics to the log — but you no longer need them: everything the library parses out
of a frame is in the keys above. In **sim** a fake tag is synthesised (ID 1 at x=3 m), so the tag
keys are populated but invented.

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
- Don't expect `/ReplayOutputs/*` in an ordinary run — those keys only exist in a `_replay.wpilog`
  produced by replaying a recording (`run-replay` skill).
- Don't expect NT topics in the log — only `Logger.recordOutput` / `@AutoLogOutput` values are
  recorded (that includes the Limelight NT tables: live-only).
- Don't expect vision keys from a sim log — there's no vision sim.
