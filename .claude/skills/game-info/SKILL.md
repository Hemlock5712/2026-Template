---
name: game-info
description: Season/field conventions this codebase enforces — the blue-origin field frame, how alliance flipping works (CTRE operator perspective), AprilTag/vision conventions, and the alliance-aware bits an agent must respect rather than re-derive. Read whenever a task involves field positions, alliance color, or game-piece scoring. Field-zone and game-piece specifics are TODO until a real game is wired.
---

# Game Info

What an agent needs to reason about robot behavior *in match context*: the field coordinate frame,
how alliance color changes things, and where game-specific constants will live. Anything purely
mechanical / software is in the **`robot-description`** skill instead.

## Season & status

This is a **WPILib 2027-alpha** template (the 2026→2027 migration target). It ships as a **starting
point, not a game-specific robot**: the [Arm](src/main/java/frc/robot/subsystems/arm/Arm.java),
[Flywheel](src/main/java/frc/robot/subsystems/flywheel/Flywheel.java), and superstructure
[poses](src/main/java/frc/robot/Robot.java) (`stow`/`intake`/`score`) are
**illustrative examples** of intake-and-shoot mechanics, not the real season's mechanisms. There are
**no field-dimension constants, no scoring-zone poses, and no `AprilTagFieldLayout` wired in yet.**

When you implement the real game, fill in the TODO sections below and update this skill.

## Field & alliance conventions this codebase enforces

Respect these as project conventions rather than re-deriving them:

- **Field origin is blue alliance.** Odometry pose ([DriveMechanism.getPose](src/main/java/frc/robot/subsystems/DriveMechanism.java))
  is always in the **blue-alliance-origin** frame (the CTRE/Phoenix convention — the origin does
  **not** move with alliance). Field poses you hand to [DriveToPose](src/main/java/frc/robot/commands/DriveToPose.java)
  are blue-origin (see the comments in [AutonomousOpMode.java](src/main/java/frc/robot/opmodes/AutonomousOpMode.java):
  "x forward from the blue wall, y left").
- **Alliance affects the driver's *perspective*, not the field origin.** The drivetrain applies an
  **operator perspective** in [CommandSwerveDrivetrain.applyOperatorPerspective](src/main/java/frc/robot/subsystems/CommandSwerveDrivetrain.java):
  blue sees forward as `0°`, red sees forward as `180°`. So "push the stick away from you" means
  "downfield" for both drivers, even though the field frame is fixed. This is driven every loop from
  [DriveMechanism](src/main/java/frc/robot/subsystems/DriveMechanism.java) and only re-applies while
  disabled (or once at startup), so it won't change behavior mid-enable.
- **Read alliance via `MatchState`, not `DriverStation` on the hot path.** The perspective code uses
  `org.wpilib.driverstation.MatchState.getAlliance()`. Follow that pattern; don't scatter alliance
  reads through subsystem code.
- **There is no automatic pose flipping for red-alliance autos yet.** `DriveToPose` drives to the
  literal blue-origin pose you give it. A red-side autonomous that should mirror its blue counterpart
  needs you to flip the goal poses yourself (rotate about field center: `x → fieldLength - x`,
  `y → fieldWidth - y`, `θ → θ + 180°`, the **ROTATE** symmetry recent fields use). The field
  dimensions to do that aren't in code yet — pull them from the game manual / `AprilTagFieldLayout`
  when you add them. **Don't copy MIRROR-symmetry flip code from older (2024/2025) projects.**

## AprilTags / vision

- The robot reads AprilTags only through **Limelights** (two cameras, NT names `"limelight-br"` /
  `"limelight-bl"`, registered in `Robot` via `Limelight.registerAll`) using
  [LimelightHelpers](src/main/java/frc/robot/subsystems/vision/LimelightHelpers.java).
  [Limelight.java](src/main/java/frc/robot/subsystems/vision/Limelight.java) fuses their AprilTag
  pose estimates into the drivetrain's pose estimator.
  [DriveToTag](src/main/java/frc/robot/commands/DriveToTag.java) works in the **tag's frame**
  (`getBotPose3d_TargetSpace`) and drives to the Limelight's configured POI standoff — so it is
  **alliance-agnostic** (it doesn't care about field origin at all).
- There is **no vision in simulation** (no PhotonVision sim), so tag-based behavior can only be
  validated on real hardware. See the `run-sim` skill.
- For canonical tag IDs / poses, use the season's WPILib `AprilTagFieldLayout` (not yet loaded in
  code) and the Limelight's field map. Don't assume a prior season's tag layout.

## Field zones / scoring locations

> **Status: TODO.** No named field positions exist in code yet. When autos are added, document the
> scoring/staging/defensive locations here, keyed to the `Pose2d` constants (blue-origin) or named
> waypoints, e.g.:
>
> | Name | Blue pose (x, y, θ) | Defined in |
> | --- | --- | --- |
> | `STATION_LEFT` | … | … |
>
> Until then, the **official game manual** and the season `AprilTagFieldLayout` are the source of
> truth for dimensions and zone names. Don't invent coordinates from memory.

## Game-piece handling

> **Status: TODO.** The arm + flywheel are placeholder mechanisms. When real manipulators land,
> document: what pieces the robot holds/scores, the named states (e.g. `EMPTY`/`STAGED`/`SCORING`),
> which sensor/NT key indicates each, and what auto routines assume about the starting piece.

## Authoritative references (when this skill is silent or stale)

1. **The current season's FRC game manual** — rules, field dimensions, scoring. Source of truth over
   anything here.
2. **The season `AprilTagFieldLayout`** — tag IDs/poses.
3. The code itself — [TunerConstants](src/main/java/frc/robot/generated/TunerConstants.java) (robot
   geometry), the OpModes ([opmodes/](src/main/java/frc/robot/opmodes/)) for what routines exist.

## Anti-patterns

- Don't hardcode red-alliance poses. Author blue-origin; flip about field center when red autos are
  added.
- Don't move the field origin with alliance — only the *driver perspective* flips.
- Don't read `DriverStation.getAlliance()` directly in subsystem `periodic`/hot paths — go through
  `MatchState` like `CommandSwerveDrivetrain` does.
- Don't assume a previous season's field/tag layout, and don't reuse MIRROR-symmetry flip math.
