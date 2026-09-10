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
device signals (closed-loop error, device temperature, CAN utilization).

Two traps about *when* values land in the `.wpilog`:

- The loop is **200 Hz** (`Robot.PERIOD_SECONDS = 0.005`), not WPILib's usual 50. Measured 199.5 Hz
  from `/Timestamp` in a real log. Anything that integrates must read timestamps, not assume 20 ms.
- **AdvantageKit only writes a record when a value changes.** Nothing is evenly sampled:
  `/Hardware/TalonFX/Arm/MotorVoltage` has 145 records against 2883 loops in the same log. Carry
  values forward (zero-order hold) before integrating, or an energy figure comes out ~20x wrong.

The 250 Hz odometry detail is *not* only in the `.hoot` — `Drivetrain/Sample*` below carries every
odometry sample, about 1.25 per loop (250 Hz into a 200 Hz loop).

## Where logs live

| Source | Path |
| --- | --- |
| Sim `.wpilog` | [logs/](logs/) in the project dir — `akit_<random>.wpilog`, renamed to `akit_<yy-MM-dd_HH-mm-ss>.wpilog` once the time is known. Newest is most recent. |
| Sim `.hoot` | **None is produced.** The path in `TunerConstants` only takes effect with real Phoenix hardware on the bus — a sim run leaves `logs/` with `.wpilog` files only. Applied volts and current are still there for the *wrapped* motors (`/Hardware/TalonFX/Arm/*`, `/Hardware/TalonFX/Flywheel/*`) — `LoggedTalonFX` logs them as inputs, sim included. What has none is the **swerve drive**: its twelve devices live inside CTRE's `SwerveDrivetrain` and are not wrapped, so 8 of the robot's 10 motors report no current anywhere. |
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
| `/Drivetrain/Pose` | `struct:Pose2d` | CTRE's odometry pose (blue-alliance origin). An **input** — replay feeds it back, it does not recompute. **The code does not drive on this**; see `EstimatedPose` |
| `/Drivetrain/Velocity` | `struct:ChassisVelocities` | Measured robot-relative chassis velocity |
| `/Drivetrain/RawHeading` | `struct:Rotation2d` | Raw gyro yaw |
| `/Drivetrain/ModuleVelocities` | `struct:SwerveModuleVelocity[]` | Per-module measured velocity + angle |
| `/Drivetrain/ModuleTargets` | `struct:SwerveModuleVelocity[]` | Per-module commanded targets |
| `/Drivetrain/ModulePositions` | `struct:SwerveModulePosition[]` | Per-module distance + angle (estimator inputs) |
| `/Drivetrain/OdometryPeriodSeconds` | `double` | Time between odometry samples. **Input, no `/RealOutputs/` prefix** |
| `/Drivetrain/TimestampSeconds` | `double` | When the module data above was sampled |
| `/Drivetrain/OdometryValid` | `boolean` | False when CTRE's odometry thread stopped keeping up — the pose is stale. Check this first when a pose looks frozen |
| `/Drivetrain/OnCanFd` | `boolean` | True on a CANivore. False means odometry fell back to ~100 Hz |
| `/Drivetrain/Rotation3d` | `struct:Rotation3d` | Full gyro orientation — pitch and roll as well as yaw. Nothing consumes it yet; it's there so a tip detector can be written against old logs |
| `/Drivetrain/OperatorForwardDirection` | `struct:Rotation2d` | The driver's "forward": 0 on blue, π on red. Changes ~once a match |
| `/Drivetrain/SampleTimestamps` | `double[]` | Every odometry sample since last loop (~1-2 at 250 Hz into a 200 Hz loop) |
| `/Drivetrain/SampleHeadings` | `struct:Rotation2d[]` | Gyro yaw at each of those samples |
| `/Drivetrain/SamplePositions` | `struct:SwerveModulePosition[]` | Wheel positions at each sample, **flattened four per timestamp** |
| `/RealOutputs/Drivetrain/TranslationSpeedMps` | `double` | `hypot(vx, vy)` |
| `/RealOutputs/Drivetrain/OdometryFrequencyHz` | `double` | `1 / OdometryPeriod` (≈250 Hz on CAN FD) |
| `/RealOutputs/Drivetrain/OdometrySamplesPerLoop` | `int64` | How many samples the loop drained |
| `/RealOutputs/Drivetrain/EstimatedPose` | `struct:Pose2d` | **Our** re-integrated pose — what `DriveMechanism.getPose()` returns, so this is the pose every command drives on. Unlike `Drivetrain/Pose` it recomputes in replay |
| `/RealOutputs/Drivetrain/EstimatedPoseErrorMeters` | `double` | How far our estimate sits from CTRE's. Runs ≤ 9 mm in practice; a jump means the re-integration stopped matching |
| `/RealOutputs/Drivetrain/Request` | `string` | The `SwerveRequest` subclass in force |
| `/RealOutputs/Drivetrain/CommandedVelocity` | `struct:ChassisVelocities` | The velocity the request asked for. Graph against `ModuleVelocities` to see what the drive actually did |
| `/RealOutputs/Drivetrain/SkidRatio` | `double` | Instrumentation only — nothing acts on it. Read `SkidDetector`'s blind spots first |
| `/RealOutputs/Drivetrain/WheelForceDemandNewtons` | `double[]` | Force this cycle's velocity change asked of each module, from CTRE's `WheelForceCalculator`. Demand, not grip — no load transfer in it, and it scales with the mass/MOI still hard-coded in `LoggedSwerveDrivetrain` |
| `/RealOutputs/Arm/AngleDegrees` | `double` | Measured arm angle. **0° = straight out horizontally** (the `Arm_Cosine` frame), so the presets read: scoring ≈ 30°, **stow = 90°**, intake = 180°. Stow is not 0. |
| `/RealOutputs/Arm/TargetDegrees` | `double` | Angle the arm is driving toward — graph against `AngleDegrees` |
| `/RealOutputs/BringUp/Arm/MeasuredRatio` | `double` | Measured rotor:mechanism ratio, signed, sampled at the furthest travel so far. `NaN` until the arm has moved. See the `device-bringup` skill |
| `/RealOutputs/BringUp/Arm/SensorTravelRot` | `double` | Rotor travel the ratio above was measured over |
| `/RealOutputs/Hardware/TalonFX/<name>/Request` | `string` | The control request last sent to that motor |
| `/RealOutputs/Flywheel/SpeedRps` | `double` | Measured wheel speed, rotations/sec (target is 25) |
| `/RealOutputs/Flywheel/AtTarget` | `boolean` | `Flywheel.isAtTarget()` — measured speed within tolerance of 25 rps. The arm has no equivalent key: it has three poses, so it has `atVertical()` / `atHorizontal()` / `atScoring()` instead. Graph `Arm/AngleDegrees` against `Arm/TargetDegrees` |
| `/RealOutputs/Superstructure/Scoring` | `boolean` | True while the StateMachine demo is in its Scoring state. **Only written by the "State Machine (no driving)" teleop** — absent from every log in `logs/` today, because nothing has run that OpMode. |

Also present, logged by AdvantageKit itself (all verified in a real sim log):

| Log key | Meaning |
| --- | --- |
| `/DriverStation/Enabled`, `/DriverStation/RobotMode` | Enabled flag + mode — use to find **enabled** transitions |
| `/DriverStation/OpMode`, `/DriverStation/OpModeId` | The selected OpMode (name + id) — which mode/routine was running |
| `/DriverStation/Joystick0..5/*` | Joystick/controller data (axes, buttons, POVs) |
| `/DriverStation/MatchType`, `MatchNumber`, `EventName`, `GameData`, `AllianceStation` | Match info |
| `/SystemStats/*` | Systemcore health: `BatteryVoltage`, `CPU/*`, `Memory/*`, `IMU/*`, `Network/CAN0..4/*`, `Faults/*`. **Dead in sim** — 150 keys, one record each, almost all zero, and `BatteryVoltage` is a flat 12.0. Only meaningful on real hardware |
| `/RealOutputs/Console` | Captured console output (`System.out` + errors) |
| `/RealOutputs/Logger/*` | AdvantageKit's own timing diagnostics, ten sub-timers. Real and useful |
| `/RealOutputs/LoggedRobot/FullCycleMS` | Whole-loop time — `UserCodeMS` + `LogPeriodicMS`. Sim median ≈ 0.64 ms of the 5 ms budget |
| `/RealOutputs/LoggedRobot/UserCodeMS` | `refreshAll` + the scheduler: our code. Sim median ≈ 0.41 ms. This is the one to watch |
| `/RealOutputs/LoggedRobot/LogPeriodicMS` | AdvantageKit's own share. Sim median ≈ 0.22 ms |
| `/RealMetadata/ProjectName` | Metadata recorded at startup |

> **Want a new key in the log?** Call `Logger.recordOutput("MySubsystem/MyKey", value)` from the
> main loop — anywhere in a subsystem, command, or OpMode. Or annotate a getter/field with
> `@AutoLogOutput` on any object reachable from `Robot`'s fields (`AutoLogOutputManager.addObject`
> is wired in `Robot`). **NetworkTables topics are NOT auto-recorded** — publishing to NT alone no
> longer puts a value in the log.

**Finding auto/teleop start:** `/DriverStation/Enabled` flips true at the start of the active mode;
`/DriverStation/OpMode` tells you by name which OpMode was selected.

**Vision keys:** `Hardware/Limelight/<name>/*` holds the whole frame, 49 keys per camera, in three
groups of parallel arrays tied together by frame index:

| Group | Entries per frame | Examples |
| --- | --- | --- |
| Frame | 1 | `TxDegrees`, `TargetDistanceMeters`, `CaptureLatencyMs`, `Imu*` (yaw/pitch/roll, gyro Z, accel XYZ) |
| Estimate | 2 — MegaTag1 then MegaTag2. No estimate-level index key: estimate `i` came from `FrameIndices[i / 2]` | `Poses`, `TagCounts`, `TagSpanMeters`, `StdDevX/Y/Theta`, `RejectionFlags` |
| Tag | one per tag seen | `TagIds`, `TagAmbiguity`, `RobotPoseTargetSpace`, `TargetPoseRobotSpace`, `RobotPoseFieldSpaceMegaTag2` |

All of it is an **input**, so a formula you write next season can use a value you never consumed this
season. `/RealOutputs/Vision/<name>/*` alongside it is our *decisions* — up to six keys per camera
(`Offered`, `Accepted`, `AcceptedPose`, `AcceptedIsMegaTag2`, `AcceptedStdDevXY`, `AcceptedStdDevOmega`),
outputs, which recompute on replay. In a sim log only `Offered` and `Accepted` appear, both zero: the
other four are written per accepted measurement, and there are none.

> **`AcceptedStdDevOmega` uses `Double.MAX_VALUE`** (1.798e308) as a "don't trust the heading"
> sentinel. Filter it before plotting or it destroys any auto-scaled axis.

LimelightLib's own NT tables (`/limelight-br/*`, `/limelight_telemetry/*`) are still live-only —
nothing routes NT topics to the log — but you no longer need them: everything the library parses out
of a frame is in the keys above.

> **There is no vision simulation.** In sim `LoggedLimelight` reports nothing, so all 49 keys per
> camera are present but empty and `connected` is false. To exercise vision code, replay a log
> recorded on the real robot.

## Reading `.wpilog` — AdvantageScope (interactive)

Open the `.wpilog` in **AdvantageScope** ("Open Log"). It decodes the struct schemas embedded in the
log, so `Drivetrain/Pose` drops onto the 2D/3D field view and `ModuleVelocities`/`ModulePositions`
render on the swerve widget; the Console tab shows `/RealOutputs/Console`. Best when you don't yet
know which keys matter.

**File → Export Data** writes CSV / WPILOG / MCAP for any selection, and its **"AdvantageKit
Cycles"** timestamp mode gives one row per synchronized loop with nothing dropped — the clean way
out to a spreadsheet or a script.

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
    if name == "/Drivetrain/Pose":  # an input, so NO /RealOutputs/ prefix
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

**Don't trust that table over the log.** Every wpilog carries its own layouts in `/.schema/struct:*`
entries — `/.schema/struct:ChassisVelocities` reads `"double vx;double vy;double omega"`. Read those
and the decoder never goes stale when WPILib renames a type (which it just did: `ChassisSpeeds` →
`ChassisVelocities`, `SwerveModuleState` → `SwerveModuleVelocity`). The table above is a convenience;
the schema records are the truth.

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
- **"Wheels fighting the target?"** Overlay `Drivetrain/ModuleVelocities` vs `ModuleTargets` per module.
- **"Brownout / CAN trouble?"** `/SystemStats/BatteryVoltage`, `/SystemStats/Faults/*`,
  `/SystemStats/Network/CAN0..4/*`. Real hardware only — these are all flat in sim.
- **"Are the wheels slipping?"** `Drivetrain/SkidRatio`, and `StatorCurrentAmps` against `kSlipCurrent`.
- **"How much traction did a launch use?"** `Drivetrain/WheelForceDemandNewtons` per module against that module's `StatorCurrentAmps`. Front and rear diverging under hard accel is load transfer, which a single `kSlipCurrent` cannot see.
- **"Where did the auto waste time?"** `python tools/auto_report.py <log>` - step durations, idle-while-arm-moves, arm motor headroom. Pass a `_replay.wpilog` and it diffs recording vs. replay.
- **"Did my one-line change move anything?"** Replay the log and diff `/RealOutputs/*` against
  `/ReplayOutputs/*` — see the `run-replay` skill.

## Don'ts

- Don't look for `NT:`-prefixed or `DS:`-prefixed keys — those were the old DataLogManager format.
  This template's keys live under `/RealOutputs/`, `/DriverStation/`, `/SystemStats/`.
- Don't expect `/ReplayOutputs/*` in an ordinary run — those keys only exist in a `_replay.wpilog`
  produced by replaying a recording (`run-replay` skill).
- Don't expect NT topics in the log — only `Logger.recordOutput` / `@AutoLogOutput` values are
  recorded (that includes the Limelight NT tables: live-only).
- Don't expect vision *sightings* from a sim log. The keys are all there; there is no vision sim, so
  they are empty.
- Don't assume 50 Hz or 20 ms anywhere. The loop is 200 Hz, and records are written only on change —
  read timestamps.
- Don't put a `/RealOutputs/` prefix on `Drivetrain/Pose`, `Velocity`, `ModulePositions`,
  `OdometryPeriodSeconds` or the `Sample*` arrays. Those are inputs and live at the top level.
- Don't look for per-command or per-subsystem timing — nothing logs command lifecycle yet.
- Don't look for swerve motor current or applied volts. The twelve drivetrain devices are inside
  CTRE's `SwerveDrivetrain` and are not wrapped, so 8 of 10 motors log none. Any "energy per
  subsystem" figure built from `/Hardware/TalonFX/*` alone covers the arm and flywheel only — the two
  smallest consumers on the robot.
- Don't scan `logs/` without excluding `*_replay.wpilog`. Of 109 files, 26 are replays; counting them
  as separate runs makes repeatability look far better than it is, because a replay's poses are
  near-identical to its parent's.
