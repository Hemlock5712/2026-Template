---
name: add-a-mechanism
description: Step-by-step procedure for adding a new mechanism (subsystem) or a new OpMode to this robot template — the file to create, the Mechanism/Command shape to copy, how to wire it into Robot, how to give it a simulation model, and how to check it works. Use when asked to add an elevator, intake, climber, shooter, wrist, turret, or any new subsystem; to add a new teleop/autonomous/utility mode; or when a student asks "how do I add a ___ to the robot".
---

# Adding a mechanism or an OpMode

> File links below are relative to the **repo root**, not to this skill's directory.

This is the most common task in the repo. Copy the shape that already exists — don't invent a new
one. [Arm.java](src/main/java/frc/robot/subsystems/arm/Arm.java) is the reference mechanism;
[Flywheel.java](src/main/java/frc/robot/subsystems/flywheel/Flywheel.java) is the same shape again.

The *concepts* (what a command is, what a gain does) are taught at **frc5712.com**. This skill is
the procedure.

## Part 1 — A new mechanism

### 1. Create the class

`src/main/java/frc/robot/subsystems/<name>/<Name>.java`, `extends Mechanism`.

The three rules that define the shape:

- **Own the hardware.** The `TalonFX` / `CANcoder` are `private final` fields here and nowhere else.
- **Keep setters private.** No public `setSpeed()`. The scheduler can only stop two things fighting
  over a motor if the only way to move it is a command.
- **Hand out commands.** Public methods return `Command`, built with `runRepeatedly(...)`.

### 2. Configure in the constructor

Follow `Arm`'s constructor order: build a `TalonFXConfiguration`, set neutral mode and inversion,
set `Slot0` gains and Motion Magic limits **inline** (no named constants for one-use config
values), set the feedback source, then `TalonFXUtil.applyConfigWithRetries(motor, config)`.

If you configure a **CANcoder**, `refresh()` the existing config first — `apply()` writes every
field, so building a fresh config zeroes the `MagnetOffset` set in Tuner X. See `Arm`.

### 3. Write the commands

```java
public Command scoring() {
  return runRepeatedly(() -> setPosition(SCORING_POSITION)).named("scoring (hold)");
}
```

**Every one of these is a hold — it never finishes.** Name it `(hold)` so a stuck routine is
obvious on the dashboard. Never add an "...AndWait" variant; the finish line goes at the call site
with `.until(...)`. See "Holds never finish" in [ONBOARDING.md](ONBOARDING.md).

### 4. Write `isAtTarget()`

Ask the motor controller, don't mirror state in Java. For a Motion Magic **position** mechanism:

```java
return motor.getMotionMagicAtTarget().getValue()
    && Math.abs(motor.getClosedLoopError().getValueAsDouble()) <= TOLERANCE.in(Rotations);
```

`getMotionMagicAtTarget()` is false before anything commands the mechanism, which is what stops a
routine reading "already arrived" at startup and skipping its wait. For **velocity**, compare
`getClosedLoopError()` to a tolerance (see `Flywheel`).

Do **not** use `getClosedLoopReference()` as the target — with Motion Magic that's the profile's
instantaneous setpoint, not the final goal, so it reads "at target" during the whole travel.

### 5. Add a default command, if idling isn't safe

A mechanism with nothing commanding it falls back to `Mechanism`'s `idle()`, which does **not**
stop the motor — a TalonFX obeys its last command forever. If yours must stop when unused:

```java
setDefaultCommand(stop());
```

An arm holding position is fine. A spinning flywheel is not.

### 6. Give it a simulation model

Without one it never moves in sim, and any routine waiting on `isAtTarget()` hangs. Copy the
"Simulation only" block at the bottom of `Arm` or `Flywheel`:

```java
if (Utils.isSimulation()) {
  Scheduler.getDefault().addPeriodic(this::updateSimulation);
}
```

Pick the matching WPILib plant (`SingleJointedArmSim`, `ElevatorSim`, `FlywheelSim`,
`DCMotorSim`), feed it `motorSim.getMotorVoltage()`, then write the result back to the sim state
of whichever sensor the closed loop actually reads.

**Sim cannot test feedback configuration** — ratios, fused-vs-remote, magnet offset, inversion —
because the plant writes the same sensor the loop reads. Verify those on hardware.

### 7. Add it to `Robot`

```java
public final Elevator elevator = new Elevator();
```

That's the whole wiring step. If it combines with other mechanisms into a pose, add a method at
the bottom of [Robot.java](src/main/java/frc/robot/Robot.java) next to `stow()` / `score()`.

### 8. Log it

```java
@AutoLogOutput(key = "Elevator/HeightMeters")
public double getHeightMeters() { ... }
```

`AutoLogOutputManager.addObject(this)` in `Robot` picks these up automatically. Log the measured
value, the target, and `AtTarget` — that trio is what makes tuning possible. Add the new keys to
the `log-reading` skill.

## Part 2 — A new OpMode

Create `src/main/java/frc/robot/opmodes/<Name>OpMode.java`:

```java
@Autonomous(name = "4 - Score And Leave")
public class ScoreAndLeaveOpMode extends PeriodicOpMode {
  private final Command routine;

  public ScoreAndLeaveOpMode(Robot robot) {
    routine = Command.sequence(...).named("Score And Leave");
  }

  @Override public void start() { Scheduler.getDefault().schedule(routine); }
  @Override public void end() { Scheduler.getDefault().cancel(routine); }
}
```

- Annotation is `@Teleop`, `@Autonomous`, or `@Utility`. **Number autonomous names** so the driver
  station list reads as the learning order.
- Build bindings and routines in the **constructor** — that's the `configureBindings()` equivalent,
  and bindings made there are automatically scoped to this OpMode.
- Put a `.withTimeout(...)` on any step that waits on a sensor.
- Author field poses **blue-origin**; [DriveToPose](src/main/java/frc/robot/commands/DriveToPose.java)
  flips them for red. See the `game-info` skill.

**It doesn't appear on the driver station?** Public, non-abstract, annotated with a `name`, in
`frc.robot` or a subpackage, with a `(Robot robot)` or no-arg public constructor. Discovery is at
runtime, so a mistake is silent — check the DS console for
`********** Starting OpMode <name> **********`.

## Check it works

1. `./gradlew build` — see the `build-and-deploy` skill for the JDK requirement.
2. `./gradlew simulateJavaAgent '-Pmode=auto:<your name>'` — see the `run-sim` skill.
3. Read the log and confirm the mechanism actually moved — see the `log-reading` skill. A clean
   console is not proof; a routine can sit forever on a hold and look fine.
