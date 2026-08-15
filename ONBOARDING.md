# How this robot project is wired (2027 · Commands v3 · OpModes)

New to this template? Read this once. The biggest surprise for anyone who has seen FRC
code before: **there is no `RobotContainer`.** This template uses WPILib's *OpMode*
framework — the same idea FTC uses. The subsystems and commands underneath are normal
Commands v3; only the top-level *wiring* is different.

> Why this way? It's the structure WPILib's own Commands v3 template uses for 2027, and it
> lets FTC and FRC code share one shape. See `memory`/the team design notes for the full
> rationale. Trade-off: most online FRC tutorials still show `RobotContainer` — this file
> is the translation guide.

## The one-paragraph mental model

- **`Robot.java`** owns the hardware (the subsystems) and runs the command scheduler. That's it.
- Each **"mode"** — driving, an autonomous routine, a calibration task — is its **own class**
  in `opmodes/`, tagged `@Teleop`, `@Autonomous`, or `@Utility`.
- The **driver station lists those classes by name**. Selecting one *constructs* it; that's
  when its button bindings / routine get set up. No `RobotContainer`, no `SendableChooser`.

## Reading order (if you're new, go in this order)

The files are **not** equally hard. Start with the small, self-contained ones and work up; the
drive and vision code is a real step up in difficulty, so it comes last on purpose.

**Start here — the core pattern:**

1. **`ONBOARDING.md`** (this file) — the mental model above.
2. **`subsystems/arm/Arm.java`** — the simplest subsystem. A subsystem owns its motor, hides its
   setters, and hands out **commands**. Learn this one shape and most of the codebase follows.
3. **`subsystems/flywheel/Flywheel.java`** — the same shape again, so the pattern sticks.
4. **`Robot.java`** — how the subsystems are owned in one place, and how `stow()` / `intake()` /
   `score()` combine two mechanisms into one command.
5. **`opmodes/TeleopOpMode.java`** — how controller buttons get wired to those commands.
6. **`opmodes/UtilityOpMode.java`** — the third mode kind, tiny.
7. **`opmodes/DriveDistanceOpMode.java`** + **`commands/DriveDistance.java`** — the simplest auto,
   and your first **closed loop**: drive 2 meters and stop.
8. **`opmodes/AutonomousOpMode.java`** — the same idea against real field positions.
9. **`opmodes/DriveStowDriveOpMode.java`** — a multi-mechanism auto by **chaining**
   (`sequence` + `.until` + `race`). This is as advanced as most routines need to be — see
   "Holds never finish" below.
10. **`opmodes/StateMachineTeleop.java`** — teleop as named states + transitions instead of
    hold-a-button.

The autonomous OpModes are **numbered on the driver station** (`1 - Drive 2 Meters`,
`2 - Drive To Pose`, `3 - Drive Stow Drive`), so the list you pick from is the order to learn them
in.

**Advanced — don't start here.** This is real, working code, but it layers PID, feedforward, motion
profiling, coordinate frames, and vision all at once. Come back once the pattern above feels
comfortable:

- `subsystems/DriveMechanism.java` — the swerve wrapper.
- `commands/DriveToPose.java`, `commands/DriveToTag.java` — drive to a field pose / to an AprilTag.
- `subsystems/vision/Vision.java` — feeds AprilTag pose estimates from the cameras (LimelightLib
  vendordep) into the drivetrain's pose estimator.
- `subsystems/CommandSwerveDrivetrain.java`, `generated/TunerConstants.java` — generated
  infrastructure you rarely edit by hand.
- `hardware/` — the `Logged*` device wrappers. Every sensor goes through one so a recorded log can
  be replayed through changed code; see the `run-replay` skill. Use `LoggedTalonFX`, never a bare
  `TalonFX` — the build fails if you do.

## If you know `RobotContainer`, here's the map

| Old way (`RobotContainer` + `TimedRobot`) | This template (OpMode) |
| --- | --- |
| Subsystems as fields in `RobotContainer` | `public final` fields on `Robot` |
| `configureBindings()` | each OpMode's **constructor** |
| `getAutonomousCommand()` + `SendableChooser` | one **`@Autonomous` class per routine** |
| `teleopInit()` | a **`@Teleop` class** |
| `testInit()` | a **`@Utility` class** |
| `robotPeriodic()` runs `CommandScheduler` | `Robot.robotPeriodic()` runs `Scheduler` |

## Lifecycle of an OpMode

```
Driver selects it on the DS ─► constructor runs      (build bindings / the routine here)
   │
   ├─ robot disabled & selected ─► disabledPeriodic()  (rarely needed)
   │
   ├─ robot ENABLED ───────────► start()  (once)  ─► periodic()  (every 5 ms - see PERIOD_SECONDS)
   │
   └─ disabled OR another mode picked ─► end()  ─► close()   (object thrown away)

Meanwhile, every loop no matter what: Robot.robotPeriodic() runs Scheduler.getDefault().run()
```

## The one subtle concept: binding scope

A `Trigger` (a button binding) is automatically **scoped to wherever you create it**:

- Created **inside an OpMode constructor** → scoped to that OpMode → **automatically removed
  when you leave the mode.** This is why you never write cleanup code for bindings.
- Created **inside `Robot`'s constructor** (no OpMode selected yet) → **global** → always
  active. Use this only for things like the brake-while-disabled binding.

**Rule of thumb:** per-mode bindings go in the OpMode; always-on bindings go in `Robot`.

**The trap:** a *default command* is not a binding and is **not** scoped. `setDefaultCommand` sets
it on the shared mechanism, so one set in an OpMode keeps running in the next one. If an OpMode
sets a default command, put it back in `close()` — see `TeleopOpMode`.

## Where things go

| Thing | Where it lives |
| --- | --- |
| Subsystem hardware (motors, sensors) | `public final` fields on `Robot` |
| Controller + button bindings | the `@Teleop` OpMode constructor |
| The joystick *default* drive command | the `@Teleop` OpMode constructor (it needs the controller) |
| An autonomous routine | an `@Autonomous` class — `schedule(...)` in `start()`, `cancel(...)` in `end()` |
| A calibration / transport task | a `@Utility` class |
| Always-on (e.g. brake while disabled) | the `Robot` constructor |

A mechanism with nothing else commanding it automatically holds its **idle** default command
(set up by `Mechanism`), so you don't need to write an explicit "stop."

## Holds never finish (the #1 "my robot is stuck" trap)

Our mechanism commands — `arm.scoring()`, `flywheel.spinUp()`, `robot.stow()` — are **holds**:
they keep re-sending their setpoint forever, so the motor stays actively commanded. That's the
right thing for closed-loop control, but it has one consequence you must know:

> **A hold never finishes, so nothing may ever *wait* on a hold.**

Put a hold inside `Command.sequence(...)` (or `await` it in a coroutine) and the robot parks
there forever. Every hold is named with **`(hold)`** so you can catch this: if a stuck routine
is sitting on a `(hold)` command on the dashboard or in the log, that's the bug.

When one step *does* need to finish, give it a finish line **at the call site** — don't go
add a "...AndWait" version to the subsystem:

```java
arm.scoring().until(arm::isAtTarget)   // same hold, but finishes when the arm arrives
```

Which tool for "do things in order" — each one is stuck-proof for its job. **Chaining (the first
three rows) is as far as most routines ever need to go:**

| Situation | Tool | Why it can't hang |
| --- | --- | --- |
| Drivetrain-only auto legs | `Command.sequence` | `DriveToPose` finishes on its own |
| One step that must finish | `.until(sensor)` on the hold | the finish line is explicit, right there |
| Do a step *while* holding a pose | `Command.race(step, hold)` | the step finishes → the race cancels the hold |
| Advanced: a hold spanning many steps, loops, branches | coroutine: `fork` holds, `await` finishers | `fork` never waits; only `await` waits |

See `opmodes/AutonomousOpMode.java` (sequence) and `opmodes/DriveStowDriveOpMode.java` (chaining
with `.until` + `race`) — those two cover almost everything. Coroutines are taught in the
**workshop repo** (branch `6-Coroutines`); `opmodes/StateMachineTeleop.java` here shows teleop as a
state machine.

Tip: `.withTimeout(seconds)` on any `.until(...)` step is the seatbelt — if a mechanism never
quite reaches its setpoint, the auto moves on instead of burning the whole period stuck.

## "My OpMode doesn't show up on the driver station!"

OpModes are discovered by **scanning classes at runtime**, so a mistake here is **not a
compile error** — the mode just silently doesn't appear. Check, in order:

1. Is the class **`public`** and **not `abstract`**?
2. Is it annotated **`@Teleop` / `@Autonomous` / `@Utility`** with a `name = "..."`?
3. Is it in **`frc.robot`** or a subpackage (we use `frc.robot.opmodes`)?
4. Does it have a **public constructor** taking `(Robot robot)` (or no arguments)?
5. **Read the driver station console.** Selecting a mode prints
   `********** Starting OpMode <name> **********`, so a missing one is easy to spot.

## Hooks we deliberately left out (keep starter OpModes simple)

The framework also offers `addPeriodic`, a custom `getPeriod`, `driverStationConnected`,
`nonePeriodic`, `disabledPeriodic`, watchdog timing, etc. You don't need any of them to start
— ignore them until you have a specific reason.

## Heads-up: this is alpha software

WPILib 2027 is in **alpha** and the OpMode API is still changing (for example, the
`UserControls` annotation was removed upstream — **don't use it**; each OpMode builds its own
controller). This project is pinned to `2027.0.0-alpha-6`. Official docs are still in
progress, so **this file and the comments in the code are your reference.**
