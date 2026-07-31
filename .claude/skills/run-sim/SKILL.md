---
name: run-sim
description: Run this robot in simulation — both the normal GUI sim and a headless, auto-enabling mode for agent / CI loops (the simulateJavaAgent Gradle task). Use whenever you need to actually run the robot code in sim, drive an autonomous routine without a human clicking Enable, or produce a log for analysis.
---

# Running the robot in simulation

> File links below are relative to the **repo root**, not to this skill's directory.

This template runs in WPILib desktop simulation with CTRE's Phoenix 6 swerve plant sim. There are
two ways to launch it.

## Normal (interactive) sim

```powershell
./gradlew simulateJava
```

Opens the Swing sim GUI + a Driver Station window. A human picks an OpMode and clicks **Enable**.
Use this when you want the GUI, Glass widgets, AdvantageScope live view, or a physical joystick.

## Headless agent sim (auto-enable) — `simulateJavaAgent`

For agent loops / CI: runs **without the GUI or socket Driver Station**, and **auto-enables** the
robot in the mode you ask for, so it actually starts playing instead of sitting disabled.

```powershell
# Headless, robot enters AUTONOMOUS immediately (the default).
./gradlew simulateJavaAgent

# Headless, robot enters TELEOP immediately.
./gradlew simulateJavaAgent -Pmode=teleop

# Headless, run the Stow utility OpMode.
./gradlew simulateJavaAgent -Pmode=utility

# Pick a SPECIFIC OpMode by name: "<mode>:<@Autonomous/@Teleop/@Utility name>".
./gradlew simulateJavaAgent '-Pmode=auto:2 - Drive To Pose'
```

You can also apply the headless behavior to the base task: `./gradlew simulateJava -Pheadless -Pmode=auto`.
`simulateJavaAgent` just makes headless + `mode=auto` the default.

**Each mode has a named default OpMode** (the constants at the top of
[SimStartup.java](src/main/java/frc/robot/utils/SimStartup.java)) — deliberately named rather than
"whichever the class scan finds first", because discovery order silently changes the moment anyone
adds an OpMode. If a default name no longer matches, the run warns, lists the available names, and
falls back rather than sitting silently disabled.

| Property | Effect |
| --- | --- |
| `-Pheadless` | Skip the sim GUI and socket Driver Station. Implied by `simulateJavaAgent`. |
| `-Pmode=auto` | Auto-enable in AUTONOMOUS (default for `simulateJavaAgent`). Runs **"3 - Drive Stow Drive"** — it exercises the drivetrain *and* the arm, so it's the useful regression test. |
| `-Pmode=teleop` | Auto-enable in TELEOPERATED. Runs **"Teleop"**. |
| `-Pmode=utility` | Auto-enable in UTILITY (the renamed "Test"). Runs **"Stow"** — arm only, no drivetrain, so it's the clean way to isolate mechanism behavior. |
| `-Pmode=utility:Bring-Up` | Sweep the arm through its presets with the flywheel spinning, to measure gear ratio / kG / kV. See the `device-bringup` skill. |
| `-Pmode=<mode>:<name>` | Pick the OpMode of `<mode>` whose annotation `name` matches `<name>`. |
| `-PstopAfter=<seconds>` | Exit on its own after N seconds. Without it the sim runs until killed, so scripts and CI need this. |
| (omitted / `-Pmode=disabled`) | Stay disabled. |

The robot **stays disabled for the first 1.5 s**, then enables. That is deliberate: an OpMode only
schedules its commands on the disabled → enabled edge, and a log that is already enabled on its
first entry cannot be replayed — nothing would ever run. See the `run-replay` skill.

### How auto-enable works

It's wired in [build.gradle](build.gradle), [SimStartup.java](src/main/java/frc/robot/utils/SimStartup.java),
and [Robot.java](src/main/java/frc/robot/Robot.java):

1. `build.gradle` passes the chosen mode to the sim JVM as the system property `frc.sim.startMode`,
   and skips `wpi.sim.addGui()` / `wpi.sim.addDriverstation()` when headless so nothing competes
   with the programmatic enable.
2. `Robot.simulationInit()` calls `SimStartup.arm()` (simulation only).
3. `SimStartup` looks up the OpMode the framework registered, then drives `DriverStationSim`:
   `setDsAttached(true)`, `setRobotMode(mode)`, `setOpMode(id)`, `setEnabled(true)`, `notifyNewData()`.
   (`setRobotMode` **and** `setOpMode` are both required — the opmode id the framework matches on
   encodes the robot mode in its high bits.)

### Verifying it worked

In the console output you should see, in order:

```
********** Robot program startup complete **********
[SimStartup] Headless start: enabled=true mode=AUTONOMOUS opmode="3 - Drive Stow Drive"
********** Starting OpMode 3 - Drive Stow Drive **********
```

If `[SimStartup]` is missing, the task wasn't `simulateJavaAgent` and you didn't pass `-Pmode`. If
you see `No <mode> OpMode named "..." found`, the name didn't match a registered OpMode (check the
`@Autonomous/@Teleop/@Utility` `name`). `No OpMode found for mode <number>` from the framework means
the id wasn't set with its mode bits — that's the bug `SimStartup.setRobotMode` exists to prevent.

## A typical agent loop

1. `./gradlew simulateJavaAgent` (headless, starts in auto).
2. Wait until the routine finishes — watch the console (the auto command completes / the drivetrain
   idles), or set your own timeout. The robot does **not** disable itself when an auto routine ends.
3. Stop the process to flush the log. On Windows the Gradle daemon spawns the sim JVM; send
   `Ctrl+Break` (not `Ctrl+C`) so the WPILOG flushes, or `Stop-Process -Name java`. From a script,
   send `CTRL_BREAK_EVENT` to the process group.
4. Hand the newest `logs/akit_*.wpilog` to the **`log-reading`** skill.

## Simulation gotchas specific to this template

- **No vision in sim.** There's no PhotonVision / Limelight sim, so `DriveToTag` (the **A** teleop
  binding, and any AprilTag align) sees **no targets** and won't converge in sim.
  Exercise vision on real hardware; use `DriveToPose` / autonomous routines for sim testing.
- **Physics is CTRE Phoenix 6 swsim only** (no maple-sim rigid-body). The 4 ms sim `Notifier` lives
  in [CommandSwerveDrivetrain](src/main/java/frc/robot/subsystems/CommandSwerveDrivetrain.java).
  The arm and flywheel have their own WPILib plants (see the "Simulation only" block at the bottom
  of each subsystem), ticked from `Scheduler.addPeriodic` at 50 Hz.
- **Sim cannot test motor *feedback configuration*.** The arm's plant writes the CANcoder position
  directly (`encoderSim.setRawPosition(...)`) and the closed loop reads that same sensor, so
  anything about how rotor and sensor relate — `RotorToSensorRatio`, `SensorToMechanismRatio`,
  fused vs. plain remote, CANcoder magnet offset, sensor inversion — **never enters the control
  path a sim run exercises.** A green sim proves the control logic, not the device config. Verify
  that class of change on hardware.
- **The arm plant updates at 50 Hz, faster than a real CANcoder is read.** Damping (`kD`) that is
  stable in sim can behave differently on hardware, and vice versa. Treat sim gains as a starting
  point.
- **"CAN message is stale" spam at startup** is normal in sim while signals spin up — ignore it.
- **No log-replay.** AdvantageKit here is logging-only (no IO layer); there is no `-Preplay`. You
  re-run the sim to test a change, then compare logs.
- **Gradle needs a Java 25 JDK.** If you see `invalid source release: 25`, point Gradle at the
  WPILib 2027 toolchain JDK (`-Dorg.gradle.java.home=...` or `org.gradle.java.home` in
  `gradle.properties`). Building from the WPILib VS Code extension handles this for you.
- **Gains are sim-tuned, not robot-tuned.** Arm/Flywheel gains are real values that work against
  the sim plants, so mechanisms do move — but re-tune them on hardware (see `robot-description`).

## When NOT to use headless

- Iterating on visualization (Glass, AdvantageScope live) — keep the GUI sim.
- You want a physical joystick — the headless DS has no joystick remap.
- Testing real-hardware behavior that sim doesn't model (CTRE closed-loop response, real Limelight
  tags).
