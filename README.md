# 2027-Template

Hemlock 5712's robot-code template for the WPILib **2027 alpha** stack:

- **Commands v3** (`org.wpilib.command3`) — mechanisms, coroutine command bodies, staged builders with compile-time `.named(...)` enforcement
- **OpModes** (`org.wpilib.opmode`) — `Robot extends OpModeRobot`; each mode is its own `@Teleop`/`@Autonomous`/`@Utility` class. No `RobotContainer`, no `SendableChooser`
- **Java 25**, deploys to **SystemCore** (not roboRIO)
- **GradleRIO** `2027.0.0-alpha-6`, **Phoenix 6** `26.50.0-alpha-1`
- Logging via **AdvantageKit** (`27.0.0-alpha-4`, logging-only — no IO layers / replay)
- Autonomous with CTRE `DriveToPose` / `LinearPath` (no PathPlanner)

> `2027-dev` is the default branch. The `main` branch is the older 2026-season template.

## Getting started

1. Install the [WPILib 2027 alpha](https://github.com/wpilibsuite/allwpilib/releases) tools (includes JDK 25).
2. Clone this repo:

   ```bash
   git clone https://github.com/Hemlock5712/2027-Template
   ```

3. Set your team number in `.wpilib/wpilib_preferences.json`.
4. Build: `./gradlew build`

## Learn with it

This template is the reference code for the Gray Matter Workshop:
**[frc5712.com](https://frc5712.com)** — mechanisms, commands, triggers, PID/Motion Magic, swerve, logging, vision, and state machines, all taught against this stack.
