---
name: robot-description
description: High-level map of this FRC robot template — the OpMode/Commands-v3 wiring, the subsystems and commands, where each piece lives in the source tree, and the hardware/tooling stack (WPILib 2027 alpha, CTRE swerve, SystemCore). Read this before reasoning about any controller / autonomous / subsystem change.
---

# Robot Description

This is a Java FRC robot **template** on the **WPILib 2027 alpha** stack (`org.wpilib.*`
packages, GradleRIO `2027.0.0-alpha-6`, Java **25**). It is the 2026→2027 migration target:
**Commands v3 + the OpMode framework**, CTRE Phoenix 6 swerve, and **logging-only** telemetry
(WPILib `DataLogManager` — there is *no* AdvantageKit and no log-replay in this project).

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
`@Utility`. The framework ([OpModeRobot](https://github.com/wpilibsuite)) scans `frc.robot` and
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
to that OpMode (auto-removed on exit); one created in the `Robot` constructor is global. The
brake-while-disabled binding in [Robot.java](src/main/java/frc/robot/Robot.java)
(`RobotModeTriggers.disabled().whileTrue(...)`) is global on purpose.

"My OpMode doesn't show up on the DS" is a **runtime** discovery failure, not a compile error — the
class must be `public`, non-`abstract`, annotated with a `name`, in `frc.robot.*`, with a public
constructor taking `(Robot robot)` (or no args). Selecting a mode prints
`********** Starting OpMode <name> **********` to the console.

## OpModes — [src/main/java/frc/robot/opmodes/](src/main/java/frc/robot/opmodes/)

| File | Annotation | What it does |
| --- | --- | --- |
| [TeleopOpMode.java](src/main/java/frc/robot/opmodes/TeleopOpMode.java) | `@Teleop("Teleop")` | Driver experience. Xbox controller on port 0; field-centric swerve as the drivetrain default command; bumpers/triggers map to superstructure presets; **A** = `DriveToTag` align; **Y** = `autoScore()` (raise arm to scoring pose + spin up flywheel concurrently; releasing **Y** stops the flywheel). |
| [AutonomousOpMode.java](src/main/java/frc/robot/opmodes/AutonomousOpMode.java) | `@Autonomous("Drive To Pose")` | Example routine: sequences two `DriveToPose` legs with `coroutine.await(...)`. The routine owns no mechanisms (`Command.noRequirements`); the requirement lives on each `DriveToPose`. |
| [UtilityOpMode.java](src/main/java/frc/robot/opmodes/UtilityOpMode.java) | `@Utility("Stow")` | Safe off-field pose (arm vertical, flywheel stopped). `@Utility` is the renamed 2027 "Test" mode. |

Add a routine = add another annotated class. `start()` schedules the command, `end()` cancels it.

## Subsystems — [src/main/java/frc/robot/subsystems/](src/main/java/frc/robot/subsystems/)

Subsystems extend Commands-v3 `Mechanism` (own the hardware, expose **commands**, hold an idle
default command when nothing else commands them).

### Drive

- [CommandSwerveDrivetrain.java](src/main/java/frc/robot/subsystems/CommandSwerveDrivetrain.java) —
  extends the Tuner-generated `TunerSwerveDrivetrain` (CTRE Phoenix 6 `SwerveDrivetrain<TalonFX,
  TalonFX, CANcoder>`). Owns the swerve hardware + odometry. In simulation it runs a 4 ms `Notifier`
  calling `updateSimState(...)` (CTRE's swerve plant sim). `applyOperatorPerspective()` applies the
  alliance perspective (blue 0°, red 180°). It is **not** a `Mechanism` (already a class).
- [DriveMechanism.java](src/main/java/frc/robot/subsystems/DriveMechanism.java) — the Commands-v3
  `Mechanism` wrapper that *owns* a `CommandSwerveDrivetrain`. Exposes `applyRequest(Supplier<SwerveRequest>)`,
  `seedFieldCentric()`, `setControl(SwerveRequest)`, and read getters `getPose()` /
  `getFieldVelocity()` (both **blue-alliance-origin**, the Phoenix convention). Registers
  `applyOperatorPerspective` on the scheduler and registers [Telemetry](src/main/java/frc/robot/utils/Telemetry.java)
  with the drivetrain. This is the **logging surface** — see the `log-reading` skill.

The drivetrain uses CTRE's `SwerveRequest` types directly (`FieldCentric`, `ApplyFieldVelocity`,
`ApplyRobotVelocity`, `Idle`). There is **no PathPlanner / Choreo / maple-sim** in this template.

### Example mechanisms (copy-and-rename for real subsystems)

- [arm/Arm.java](src/main/java/frc/robot/subsystems/arm/Arm.java) — single `TalonFX` (CAN 31) +
  `CANcoder` (CAN 32), `MotionMagicVoltage` position control with `Arm_Cosine` gravity FF. Presets:
  `vertical()` (stow), `horizontal()` (intake), `scoring()` / `scoringAndWait()`. **All gains are
  zeroed and marked "NEEDS TUNING"** — this is a template.
- [flywheel/Flywheel.java](src/main/java/frc/robot/subsystems/flywheel/Flywheel.java) — single
  `TalonFX` (CAN 21), `MotionMagicVelocityVoltage`, shooting speed 25 RPS. `spinUp()` / `stop()`.
- **Superstructure poses** — the arm + flywheel coordinator. These are plain methods at the bottom
  of [Robot.java](src/main/java/frc/robot/Robot.java) (NOT a separate class or `Mechanism`): each
  composes arm + flywheel into one command per robot pose — `stow()`, `intake()`, `score()`,
  `autoScore()`. An OpMode calls them directly on the `Robot` reference, e.g. `robot.stow()`.

### Vision

- [vision/LimelightHelpers.java](src/main/java/frc/robot/subsystems/vision/LimelightHelpers.java) —
  vendored Limelight NT helper. The camera NT name is `"limelight"`. There is **no PhotonVision and
  no vision sim**, so AprilTag-based commands see no targets in simulation (see the `run-sim` skill).

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
| [DriveToTag.java](src/main/java/frc/robot/commands/DriveToTag.java) | classic | **Vision-only** align to an AprilTag using `LimelightHelpers.getBotPose3d_TargetSpace`; three `ProfiledPIDController`s drive the tag-frame offset to the Limelight's POI standoff. |
| [DriveToTagInline.java](src/main/java/frc/robot/commands/DriveToTagInline.java) | inline | The same behavior as `DriveToTag`, written inline, kept as a reference example of the inline style. Not used by any OpMode. |

## Hardware constants — [generated/TunerConstants.java](src/main/java/frc/robot/generated/TunerConstants.java)

Generated by the **2027 Tuner X Swerve Project Generator** — the checked-in file is an **example
placeholder** with fake device IDs/gains; regenerate it (and `CommandSwerveDrivetrain`) from Tuner X
for a real robot. Key values:

- CAN bus: `new CANBus("canivore", "./logs/example.hoot")` — a **CANivore** named `canivore`, with
  Phoenix signal logging to `./logs/example.hoot` (see the `log-reading` skill for `.hoot` files).
- `kSpeedAt12Volts = 4.54 m/s` (teleop normalizes to this). `kCANBus` is reused by `Arm`/`Flywheel`.
- Module/gyro IDs, gear ratios, steer/drive gains, slip current, sim inertias — all here.
- `TunerConstants.createDrivetrain()` is the factory `DriveMechanism` calls.

## Logging

No AdvantageKit. [Robot.java](src/main/java/frc/robot/Robot.java) starts WPILib `DataLogManager` +
`DriverStation.startDataLog(...)`, which record **every NetworkTables value change** (including
everything [Telemetry](src/main/java/frc/robot/utils/Telemetry.java) publishes under `Drivetrain/*`),
console output, and DS/joystick data to a `.wpilog`. Phoenix devices also log to `./logs/example.hoot`.
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
- Vendordeps: only [Phoenix6](vendordeps/Phoenix6-26.50.0-alpha-1.json) (`26.50.0-alpha-1`) and
  [CommandsV3](vendordeps/CommandsV3.json) (`1.0.0`). No PathPlanner/Choreo/AdvantageKit/maple-sim/PhotonVision.
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
| Telemetry → NT/WPILOG | [utils/Telemetry.java](src/main/java/frc/robot/utils/Telemetry.java) |
| Headless sim auto-enable | [utils/SimStartup.java](src/main/java/frc/robot/utils/SimStartup.java) |
| Limelight helper | [vision/LimelightHelpers.java](src/main/java/frc/robot/subsystems/vision/LimelightHelpers.java) |
| TalonFX config helper | [utils/TalonFXUtil.java](src/main/java/frc/robot/utils/TalonFXUtil.java) |
