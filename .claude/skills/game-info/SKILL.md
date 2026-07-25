---
name: game-info
description: Season/field conventions this codebase enforces — the blue-origin field frame, automatic red-alliance pose flipping (AllianceFlip), the CTRE operator perspective, and AprilTag/target-space conventions. Use whenever a task involves field positions, Pose2d goals, alliance color, red-vs-blue autos, AprilTag IDs, or where the field origin is.
---

# Game Info

> File links below are relative to the **repo root**, not to this skill's directory.

What an agent needs to reason about robot behavior *in match context*: the field coordinate frame,
how alliance color changes things, and where game-specific constants will live. Anything purely
mechanical / software is in the **`robot-description`** skill instead.

## Season & status

This is a **WPILib 2027-alpha** template (the 2026→2027 migration target). It ships as a **starting
point, not a game-specific robot**: the [Arm](src/main/java/frc/robot/subsystems/arm/Arm.java),
[Flywheel](src/main/java/frc/robot/subsystems/flywheel/Flywheel.java), and superstructure
[poses](src/main/java/frc/robot/Robot.java) (`stow`/`intake`/`score`) are
**illustrative examples** of intake-and-shoot mechanics, not the real season's mechanisms. There are
**no scoring-zone poses yet.** Field dimensions come from the `AprilTagFieldLayout` loaded in
[AllianceFlip](src/main/java/frc/robot/utils/AllianceFlip.java), currently the **2026** field
(`AprilTagFields.kDefaultField`) because WPILib has not shipped a 2027 layout.

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
- **Pose flipping is automatic — author every pose blue-origin.**
  [AllianceFlip](src/main/java/frc/robot/utils/AllianceFlip.java) does the **ROTATE** symmetry recent
  fields use (`x → fieldLength - x`, `y → fieldWidth - y`, `θ → θ + 180°`), reading alliance via
  `MatchState.getAlliance()` and treating an unknown alliance as blue. Field dimensions come from
  `AprilTagFieldLayout.loadField(AprilTagFields.kDefaultField)`, so they track the season instead of
  being hardcoded — **`kDefaultField` is still the 2026 field; swap it when WPILib ships 2027.**
  [DriveToPose](src/main/java/frc/robot/commands/DriveToPose.java) calls `AllianceFlip.apply` in
  `initialize()` (not in the constructor — the alliance isn't known when an OpMode is built), so
  passing it a blue pose is all a routine ever needs to do. **Don't copy MIRROR-symmetry flip code
  from older (2024/2025) projects.**

## AprilTags / vision

- The robot reads AprilTags only through **Limelights** (two cameras, NT names `"limelight-br"` /
  `"limelight-bl"`), via the official **LimelightLib 2** vendordep (`com.limelightvision.Limelight`
  objects owned as fields on `Robot`).
  [Vision.java](src/main/java/frc/robot/subsystems/vision/Vision.java) fuses their AprilTag pose
  estimates into the drivetrain's pose estimator (`Vision.registerAll` in `Robot`).
  [DriveToTag](src/main/java/frc/robot/commands/DriveToTag.java) works in the **tag's frame**
  (`FiducialTarget.getRobotPose_TargetSpace()`) and drives to the Limelight's configured POI
  standoff — so it is **alliance-agnostic** (it doesn't care about field origin at all).
- **2027 Limelight coordinate convention (breaking vs 2026):** everything is unified NWU
  right-handed. Target space is **+X out of the tag face, +Y to the tag's left, +Z up**, so a robot
  parked facing the tag has yaw **±180°** in tag space, not 0 — `DriveToTag` drives yaw to π.
  Camera firmware must be 2027.0+, and camera-side config (mount pitch/side signs, POI z-component)
  must be re-entered per the migration guide.
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

- Don't hardcode red-alliance poses, and don't flip by hand. Author blue-origin and let
  `AllianceFlip` / `DriveToPose` handle red.
- Don't move the field origin with alliance — only the *driver perspective* flips.
- Don't read `DriverStation.getAlliance()` directly in subsystem `periodic`/hot paths — go through
  `MatchState` like `CommandSwerveDrivetrain` does.
- Don't assume a previous season's field/tag layout, and don't reuse MIRROR-symmetry flip math.
