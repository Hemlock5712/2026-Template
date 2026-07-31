// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.opmodes;

import frc.robot.Robot;
import org.littletonrobotics.junction.Logger;
import org.wpilib.command3.Command;
import org.wpilib.command3.Scheduler;
import org.wpilib.command3.StateMachine;
import org.wpilib.command3.StateMachine.State;
import org.wpilib.command3.button.CommandNiDsXboxController;
import org.wpilib.opmode.PeriodicOpMode;
import org.wpilib.opmode.Teleop;

/**
 * The superstructure run as a {@link StateMachine} - the lesson at <a
 * href="https://frc5712.com/state-based">frc5712.com/state-based</a> as real code.
 *
 * <p>Unlike {@link TeleopOpMode} (buttons hold presets), the robot is always in exactly one named
 * state, and only the declared transitions can happen.
 *
 * <p>Build in four steps (numbered below). No drive controls here - select "Teleop" for driving.
 */
@Teleop(name = "State Machine (no driving)")
public class StateMachineTeleop extends PeriodicOpMode {
  private final CommandNiDsXboxController driver = new CommandNiDsXboxController(0);
  private final Command machine;

  public StateMachineTeleop(Robot robot) {
    // 1. Construct - the name shows up in telemetry.
    StateMachine sm = new StateMachine("Superstructure");

    // 2. Add states - each owns one command. Holds are fine: a when(...) transition CANCELS the
    //    old state's command.
    State stowed = sm.addState(robot.stow());
    State pickup = sm.addState(robot.intake());
    State prep =
        sm.addState(
            robot.arm.scoring().until(robot.arm::atScoring).named("scoring until at target"));
    State scoring = sm.addState(robot.score());

    // 3. Every machine needs a starting state.
    sm.setInitialState(stowed);

    // 4. Wire transitions. when(...) is checked every tick and fires on false -> true.
    stowed.switchTo(pickup).when(driver.leftTrigger());
    pickup.switchTo(prep).when(robot.arm::atHorizontal); // real robot: a game-piece sensor

    // prep's command finishes on its own, so use whenComplete() instead of when(...).
    prep.switchTo(scoring).whenComplete();

    scoring.switchTo(stowed).when(driver.rightTrigger());

    // B = "get safe now". switchFromAny() only covers states added so far - declare it last!
    sm.switchFromAny().to(stowed).when(driver.b());

    // onEnter/onExit hooks - logs a true/false trace of time spent in Scoring.
    scoring.onEnter(() -> Logger.recordOutput("Superstructure/Scoring", true));
    scoring.onExit(() -> Logger.recordOutput("Superstructure/Scoring", false));

    // More power, when you need it (uncomment and adapt):
    //
    // whenComplete + an extra check:
    //   prep.switchTo(stowed).whenCompleteAnd(() -> !hasGamePiece());
    //
    // Pick the target at transition time:
    //   prep.switchTo(() -> hasGamePiece() ? scoring : stowed).whenComplete();
    //
    // End the whole machine instead of switching:
    //   sm.switchFromAny().toExitStateMachine().when(driver.back());

    machine = sm; // a StateMachine is just a Command - schedule it like any other
  }

  @Override
  public void start() {
    Scheduler.getDefault().schedule(machine);
  }

  @Override
  public void end() {
    Scheduler.getDefault().cancel(machine);
  }
}
