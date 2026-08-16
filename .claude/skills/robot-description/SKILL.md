---
name: robot-description
description: High-level map of this FRC robot template — the OpMode/Commands-v3 wiring, the subsystems and commands, where each piece lives in the source tree, and the hardware/tooling stack (WPILib 2027 alpha, CTRE swerve, AdvantageKit, SystemCore). Use when asked where something lives, how the robot is wired, "where is RobotContainer" (there isn't one — this template uses OpModes), how OpModes are discovered, or before reasoning about any controller / autonomous / subsystem change.
---

# Robot Description

This is a Java FRC robot **template** on the **WPILib 2027 alpha** stack (`org.wpilib.*`
packages, GradleRIO `2027.0.0-alpha-6`, Java **25**). It is the 2026→2027 migration target:
**Commands v3 + the OpMode framework**, CTRE Phoenix 6 swerve, and **AdvantageKit** telemetry
(`WPILOGWriter` + `NT4Publisher`, wired in [Robot.java](src/main/java/frc/robot/Robot.java)). No IO
layer — devices are wrapped instead (see § Logging). `DataLogManager` is *not* used.

> All file links below are relative to the **repo root**, not to this skill's directory.

Entry point: [Main.java](src/main/java/frc/robot/Main.java) → [Robot.java](src/main/java/frc/robot/Robot.java).
Read [ONBOARDING.md](ONBOARDING.md) first if you haven't — it explains the single biggest
surprise: **there is no `RobotContainer`.**

> Heads-up: WPILib 2027 is **alpha**, pinned to `2027.0.0-alpha-6` on purpose. The OpMode /
> Commands-v3 APIs still move between alphas; don't bump the version casually, and treat this file
> + the code comments as the reference since official docs are incomplete. See the comment block at
> the top of [build.gradle](build.gradle).

## The wiring model (no RobotContainer)

[Robot.java](src/main/java/frc/robot/Robot.java) **owns the hardware** (subsystems as `public final`
fields) and runs the Commands-v3 scheduler in `robotPeriodic()`. That's it. There are no per-mode
`init`/`periodic` methods.

Each **mode** — a driving experience, an autonomous routine, a calibration task — is its **own
class** in [opmodes/](src/main/java/frc/robot/opmodes/), annotated `@Teleop`, `@Autonomous`, or
`@Utility`. The framework (`org.wpilib.framework.OpModeRobot`) scans `frc.robot` and
subpackages at runtime, registers every annotated class with the driver station, and the DS lists
them by name. Selecting one **constructs** it (that's when its button bindings / routine are built);
switching away **tears it down** (its bindings are scoped to it and removed automatically).

| Old way (`RobotContainer` + `TimedRobot`) | This template (OpMode) |
| --- | --- |
| Subsystems as fields in `RobotContainer` | `public final` fields on [Robot](src/main/java/frc/robot/Robot.java) |
| `configureBindings()` | each OpMode's **constructor** |
| `getAutonomousCommand()` + `SendableChooser` | one **`@Autonomous` class per routine** |
| `teleopInit()` | a **`@Teleop` class** |
| `testInit()` | a **`@Utility` class** |
| always-on bindings | created in the **`Robot` constructor** (global scope) |

**Binding scope is the one subtle rule:** a `Trigger` created inside an OpMode constructor is scoped
to that OpMode (auto-removed on exit); one created in the `Robot` constructor is global. There are
no global bindings today — [Robot.java](src/main/java/frc/robot/Robot.java) marks the spot for them.

"My OpMode doesn't show up on the DS" is a **runtime** discovery failure, not a compile error — the
class must be `public`, non-`abstract`, annotated with a `name`, in `frc.robot.*`, with a public
constructor taking `(Robot robot)` (or no args). Selecting a mode prints
`********** Starting OpMode <name> **********` to the console.

## OpModes — [src/main/java/frc/robot/opmodes/](src/main/java/frc/robot/opmodes/)

| File | Annotation | What it does |
| --- | --- | --- |
| [TeleopOpMode.java](src/main/java/frc/robot/opmodes/TeleopOpMode.java) | `@Teleop("Teleop")` | Driver experience. Xbox controller on port 0; field-centric swerve as the drivetrain default command. **LT** = `intake()`, **RB** = `score()`, **RT** = `stow()` (superstructure presets, `whileTrue`); **A** = `DriveToTag` align (camera `robot.limelightBR`); **Y** = `autoScore()` (arm to scoring pose + flywheel spin-up; releasing **Y** stops the flywheel). |
| [StateMachineTeleop.java](src/main/java/frc/robot/opmodes/StateMachineTeleop.java) | `@Teleop("State Machine (no driving)")` | The superstructure as a Commands-v3 `StateMachine`: named states (stowed/pickup/prep/scoring), `when(...)` / `whenComplete()` transitions, enter/exit hooks. No drive controls — a superstructure showcase. |
| [DriveDistanceOpMode.java](src/main/java/frc/robot/opmodes/DriveDistanceOpMode.java) | `@Autonomous("1 - Drive 2 Meters")` | The simplest auto and the first closed loop: one [DriveDistance](src/main/java/frc/robot/commands/DriveDistance.java) with a `.withTimeout(...)` seatbelt. No field frame, no alliance, no profile. |
| [AutonomousOpMode.java](src/main/java/frc/robot/opmodes/AutonomousOpMode.java) | `@Autonomous("2 - Drive To Pose")` | Sequences two `DriveToPose` legs with `Command.sequence(...).named(...)`. The sequential group inherits its children's requirement (the drivetrain), and the scheduler hands the drivetrain off between legs. `start()` schedules the routine; `end()` cancels it. |
| [DriveStowDriveOpMode.java](src/main/java/frc/robot/opmodes/DriveStowDriveOpMode.java) | `@Autonomous("3 - Drive Stow Drive")` | **The reference for multi-mechanism autos** — chaining: `sequence` + `.until(arm::atVertical)` (give a hold a finish line) + `Command.race(step, hold)` (do a step while holding a pose). This style is the team's teaching ceiling. |
| [UtilityOpMode.java](src/main/java/frc/robot/opmodes/UtilityOpMode.java) | `@Utility("Stow")` | Safe off-field pose (arm vertical, flywheel stopped). `@Utility` is the renamed 2027 "Test" mode. |
| [BringUpOpMode.java](src/main/java/frc/robot/opmodes/BringUpOpMode.java) | `@Utility("Bring-Up")` | **Moves the mechanisms.** Sweeps the arm through its three presets with a 2.5 s dwell each while the flywheel holds speed, so one run yields gear ratio, magnet offset and kG/kV. See the `device-bringup` skill. |

Add a routine = add another annotated class. `start()` schedules the command, `end()` cancels it.

## Subsystems — [src/main/java/frc/robot/subsystems/](src/main/java/frc/robot/subsystems/)

Subsystems extend Commands-v3 `Mechanism` (own the hardware, expose **commands**, hold an idle
default command when nothing else commands them).

**Hold convention:** every mechanism command is a persistent `runRepeatedly` hold that **never
finishes** — the names carry a `(hold)` suffix so this is visible in telemetry. Never put a hold
somewhere that waits on it (`Command.sequence`, coroutine `await`); to make one step finish, add
`.until(mech::atSomePose)` **at the call site** (there are no `...AndWait` methods) — the predicate
names the goal, because a no-argument `isAtTarget()` reads the previous move's answer on the first
loop of a chained move. The full rule
and the "which composition tool when" table live in `ONBOARDING.md` § "Holds never finish".

### Drive

- [CommandSwerveDrivetrain.java](src/main/java/frc/robot/subsystems/CommandSwerveDrivetrain.java) —
  extends the Tuner-generated `TunerSwerveDrivetrain` (CTRE Phoenix 6 `SwerveDrivetrain<TalonFX,
  TalonFX, CANcoder>`). Owns the swerve hardware + odometry. In simulation it runs a 4 ms `Notifier`
  calling `updateSimState(...)` (CTRE's swerve plant sim). `applyOperatorPerspective()` applies the
  alliance perspective (blue 0°, red 180°). It is **not** a `Mechanism` (already a class).
- [DriveMechanism.java](src/main/java/frc/robot/subsystems/DriveMechanism.java) — the Commands-v3
  `Mechanism` wrapper that *owns* a `CommandSwerveDrivetrain`. Exposes `applyRequest(Supplier<SwerveRequest>)`,
  `setControl(SwerveRequest)`, `addVisionMeasurement(...)`, `resetPose(Pose2d)` (an auto's starting
  waypoint — PathPlanner's `AutoBuilder` needs it; prefer vision when tags are visible), and read getters `getPose()` /
  `getFieldVelocity()` (both **blue-alliance-origin**, the Phoenix convention). `getPose()` is
  **our** `SwerveDrivePoseEstimator`, re-integrated from the logged samples, not CTRE's native
  answer — that is what makes a vision-trust change move the *replayed* robot and not just a graph.
  There is
  deliberately **no `seedFieldCentric()` on `DriveMechanism`** (the wrapper below has it) — re-zeroing the heading rewrites the pose estimator's
  rotation, which is the same heading MegaTag2 solves against, so one press silently corrupts every
  later vision fix. Registers
  `applyOperatorPerspective` on the scheduler and wraps the drivetrain in
  [LoggedSwerveDrivetrain](src/main/java/frc/robot/hardware/LoggedSwerveDrivetrain.java), which
  routes the whole `SwerveDriveState` through the log. Both getters read from there, not from the
  live drivetrain. This is the **logging surface** — see `log-reading`.
- `LoggedSwerveDrivetrain` mirrors **CTRE's whole `SwerveDrivetrain` API** on purpose, so anything a
  command might need is already replay-correct — you add it to `DriveMechanism`, not to the wrapper.
  Five methods are deliberately absent (`getState`/`getStateCopy`, `getModule`/`getModules`/
  `getPigeon2`, `registerTelemetry`, `updateSimState`, `optimizeBusUtilization`); the class javadoc
  says why for each.

The drivetrain uses CTRE's `SwerveRequest` types directly (`FieldCentric`, `ApplyFieldVelocity`,
`ApplyRobotVelocity`, `Idle`). There is **no PathPlanner / Choreo / maple-sim** in this template.

### Example mechanisms (copy-and-rename for real subsystems)

- [arm/Arm.java](src/main/java/frc/robot/subsystems/arm/Arm.java) — single `TalonFX` (CAN 31) +
  `CANcoder` (CAN 32), `MotionMagicVoltage` position control with `Arm_Cosine` gravity FF. Presets:
  `vertical()` (stow), `horizontal()` (intake), `scoring()`. Gains are **tuned against the sim
  plant, not a real arm** — re-tune on hardware. Has a `SingleJointedArmSim` model and logs
  `Arm/AngleDegrees`, `Arm/TargetDegrees`.
- [flywheel/Flywheel.java](src/main/java/frc/robot/subsystems/flywheel/Flywheel.java) — single
  `TalonFX` (CAN 21), `MotionMagicVelocityVoltage`, shooting speed 25 RPS. `spinUp()` / `stop()`,
  with `stop()` as its **default command** (a TalonFX otherwise holds its last command forever).
  Has a `FlywheelSim` model; logs `Flywheel/SpeedRps`, `Flywheel/AtTarget`.
- **Superstructure poses** — the arm + flywheel coordinator. These are plain methods at the bottom
  of [Robot.java](src/main/java/frc/robot/Robot.java) (NOT a separate class or `Mechanism`): each
  composes arm + flywheel into one command per robot pose — `stow()`, `intake()`, `score()`,
  `autoScore()`. An OpMode calls them directly on the `Robot` reference, e.g. `robot.stow()`.

### Vision

- Cameras are **LimelightLib 2** vendordep objects (`com.limelightvision.Limelight`), owned as
  `public final` fields on [Robot.java](src/main/java/frc/robot/Robot.java) like any other
  hardware: **two Limelights, NT names `"limelight-br"` and `"limelight-bl"`**
  (`robot.limelightBR` / `robot.limelightBL`).
- [vision/Vision.java](src/main/java/frc/robot/subsystems/vision/Vision.java) — feeds each
  camera's AprilTag pose estimates into the drivetrain's pose estimator (`addVisionMeasurement`).
  EVERY trust decision lives in `Vision`, not the library — `PERMISSIVE_MT1/MT2` switch the
  library's tunable gates off on purpose so the decision replays. `Vision` picks MegaTag1 for 2+
  tags, MegaTag2 for a lone tag (gyro heading — seed the gyro). `Vision.registerAll(...)` in
  `Robot` wires the cameras. The robot heading MegaTag2 needs is published to the `limelightshared`
  NT table every loop (200 Hz) by `LoggedLimelight.setSharedRobotOrientation`. The library
  auto-publishes accepted/rejected pose telemetry under `limelight_telemetry`.

There is **no PhotonVision and no vision sim**, so AprilTag-based commands see no targets in
simulation (see the `run-sim` skill).

## Commands — [src/main/java/frc/robot/commands/](src/main/java/frc/robot/commands/)

Two authoring styles coexist; pick whichever reads better. Both are Commands v3.

- **Classic** ([ClassicCommand.java](src/main/java/frc/robot/utils/ClassicCommand.java)) — a v2-style
  wrapper: override `initialize()` / `execute()` / `isFinished()` / `end(interrupted)` and the base
  drives the coroutine. Familiar for anyone coming from Commands v2.
- **Inline** — a single `Command.run(coroutine -> { ... })` with all logic in one linear body. More
  idiomatic v3.

| Command | Style | What it does |
| --- | --- | --- |
| [DriveToPose.java](src/main/java/frc/robot/commands/DriveToPose.java) | classic | Straight-line drive to a blue-origin `Pose2d` on **odometry**, via CTRE `LinearPath` (trapezoid profile feedforward) + per-axis PID feedback. The building block for autonomous. |
| [DriveToTag.java](src/main/java/frc/robot/commands/DriveToTag.java) | classic | **Vision-only** align to an AprilTag using LimelightLib's `FiducialTarget.getRobotPose_TargetSpace()`; three `ProfiledPIDController`s drive the tag-frame offset to the Limelight's POI standoff (2027 tag frame: +X out of the tag face, +Y tag-left — facing the tag is yaw ±π). |
| [DriveDistance.java](src/main/java/frc/robot/commands/DriveDistance.java) | classic | Drive forward a signed distance with one P controller, measured from the start pose. No profile, no field frame — the teaching rung below `DriveToPose`. |

## Hardware constants — [generated/TunerConstants.java](src/main/java/frc/robot/generated/TunerConstants.java)

Generated by the **2026 Tuner X Swerve Project Generator** (2027 Tuner not out yet) — the checked-in file is an **example
placeholder** with fake device IDs/gains; regenerate it (and `CommandSwerveDrivetrain`) from Tuner X
for a real robot. Key values:

- CAN bus: `new CANBus("canivore")` — a **CANivore** named `canivore`. Never pass a second argument:
  `new CANBus(name, hootPath)` *replays* that hoot instead of reading the bus. Hoot logging is
  `SignalLogger` (see the `log-reading` skill).
- `kSpeedAt12Volts = 4.54 m/s` (teleop normalizes to this). `kCANBus` is reused by `Arm`/`Flywheel`.
- Module/gyro IDs, gear ratios, steer/drive gains, slip current, sim inertias — all here.
- `TunerConstants.createDrivetrain()` is the factory `DriveMechanism` calls.

## Logging

**AdvantageKit with replay** (this branch descends from `advanced-replay`; plain `main` is
logging-only).
[Robot.java](src/main/java/frc/robot/Robot.java) starts the `Logger` (`WPILOGWriter` +
`NT4Publisher`) and ticks it in `robotPeriodic()` — this template extends `OpModeRobot`, not
`LoggedRobot`, so the tick is manual via `Logger.AdvancedHooks`.

Every sensor is read through a wrapper in
[frc/robot/hardware](src/main/java/frc/robot/hardware) and collected once per loop by
`LoggedHardware.refreshAll()`, so a recorded log can be replayed through changed code — see the
**`run-replay`** skill. `LoggedSwerveDrivetrain` logs the drive state under `Drivetrain/*`;
DS/joystick data, system stats, and console output are
captured by AdvantageKit itself. **NT topics are not auto-recorded** — new values go in the log via
`Logger.recordOutput` or `@AutoLogOutput`. Phoenix devices also log to `./logs/example.hoot`.
Full details (paths, key list, how to read both formats) are in the **`log-reading`** skill.

## Simulation

`./gradlew simulateJava` is the normal GUI sim. `./gradlew simulateJavaAgent` runs **headless and
auto-enables** for agent / CI use, via [SimStartup.java](src/main/java/frc/robot/utils/SimStartup.java).
Physics is CTRE's Phoenix 6 swerve plant sim (no maple-sim). Full details in the **`run-sim`** skill.

## Build / tooling

- **GradleRIO `2027.0.0-alpha-6`**, deploy target **SystemCore** (`./gradlew deploy` →
  `deploysystemcore`; the host is `/home/systemcore`, *not* a roboRIO). `team` comes from
  `.wpilib/wpilib_preferences.json`.
- **Java 25** source/target. Gradle must run on a Java 25 JDK (e.g. the WPILib 2027 toolchain JDK);
  an older JVM fails with `invalid source release: 25`.
- Vendordeps: [Phoenix6](vendordeps/Phoenix6-26.50.0-alpha-1.json) (`26.50.0-alpha-1`),
  [CommandsV3](vendordeps/CommandsV3.json) (`1.0.0`),
  [LimelightLib](vendordeps/LimelightLib.json) (`2.0.0-beta2`, Java-only), and
  [AdvantageKit](vendordeps/AdvantageKit.json) (`27.0.0-alpha-4`).
  No PathPlanner/Choreo/maple-sim/PhotonVision.
- Spotless (Google Java Format) runs on every `JavaCompile` (`dependsOn 'spotlessApply'`). Build/format
  from the WPILib VS Code extension or a Java-25 Gradle invocation.

## What lives where (cheat sheet)

| Topic | File |
| --- | --- |
| Hardware ownership + scheduler + logging start + superstructure poses | [Robot.java](src/main/java/frc/robot/Robot.java) |
| Teleop / autonomous / utility modes | [opmodes/](src/main/java/frc/robot/opmodes/) |
| Swerve wrapper (Mechanism) | [DriveMechanism.java](src/main/java/frc/robot/subsystems/DriveMechanism.java) |
| Swerve hardware (CTRE) | [CommandSwerveDrivetrain.java](src/main/java/frc/robot/subsystems/CommandSwerveDrivetrain.java) |
| Example mechanisms | [arm/Arm.java](src/main/java/frc/robot/subsystems/arm/Arm.java), [flywheel/Flywheel.java](src/main/java/frc/robot/subsystems/flywheel/Flywheel.java) |
| Drive commands | [commands/](src/main/java/frc/robot/commands/) |
| v2-style command base | [utils/ClassicCommand.java](src/main/java/frc/robot/utils/ClassicCommand.java) |
| Swerve constants / IDs / gains | [generated/TunerConstants.java](src/main/java/frc/robot/generated/TunerConstants.java) |
| Logged device wrappers (replay) | [hardware/](src/main/java/frc/robot/hardware/) — `LoggedTalonFX`, `LoggedCANcoder`, `LoggedCANrange`, `LoggedLimelight`, `LoggedSwerveDrivetrain` |
| Per-loop sensor read + log | [hardware/LoggedHardware.java](src/main/java/frc/robot/hardware/LoggedHardware.java) |
| Bring-up measurements (ratio, offset, kG/kV) | [hardware/BringUp.java](src/main/java/frc/robot/hardware/BringUp.java) + [opmodes/BringUpOpMode.java](src/main/java/frc/robot/opmodes/BringUpOpMode.java) |
| REAL / SIM / REPLAY mode | [utils/RunMode.java](src/main/java/frc/robot/utils/RunMode.java) |
| Replay regression check | [ReplayCheck.java](src/test/java/frc/robot/ReplayCheck.java) (`./gradlew replayCheck`) |
| Patched WPILib copies (delete once upstream) | `src/main/java/org/wpilib/` |
| Headless sim auto-enable | [utils/SimStartup.java](src/main/java/frc/robot/utils/SimStartup.java) |
| Vision → pose estimator (per-camera) | [vision/Vision.java](src/main/java/frc/robot/subsystems/vision/Vision.java) |
| Limelight camera objects (hardware) | [Robot.java](src/main/java/frc/robot/Robot.java) (`limelightBR` / `limelightBL`, `LoggedLimelight` wrapping the LimelightLib vendordep) |
| TalonFX config helper | [utils/TalonFXUtil.java](src/main/java/frc/robot/utils/TalonFXUtil.java) |
