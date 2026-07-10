// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.opmodes;

import frc.robot.Robot;
import org.wpilib.command3.Command;
import org.wpilib.command3.Scheduler;
import org.wpilib.command3.StateMachine;
import org.wpilib.command3.StateMachine.State;
import org.wpilib.command3.button.CommandNiDsXboxController;
import org.wpilib.opmode.PeriodicOpMode;
import org.wpilib.opmode.Teleop;
import org.wpilib.system.DataLogManager;

/**
 * The superstructure run as a {@link StateMachine} - the state-machines lesson at <a
 * href="https://frc5712.com/state-based">frc5712.com/state-based</a>, as real code.
 *
 * <p>Compare with {@link TeleopOpMode}: there, each button <i>holds</i> a superstructure preset
 * (whileTrue). Here the robot is always in exactly one named state, and buttons/sensors <i>move it
 * between states</i>. The machine cancels the old state's command and starts the new one for you;
 * illegal jumps simply don't exist because no transition was declared for them.
 *
 * <p>Building one takes four steps (numbered below): construct, add states, pick the initial state,
 * wire transitions. This demo shows both kinds of transition - {@code when(...)} (checked every
 * tick while the state runs) and {@code whenComplete()} (checked once, when the state's command
 * finishes) - plus {@code onEnter}/{@code onExit} hooks for side effects. {@code setInitialState}
 * is enforced at build time - forget it and the build fails, not the match.
 *
 * <p>This demo binds no drive controls; it is a superstructure showcase. Select "Teleop" on the
 * driver station for the full driving layout.
 */
@Teleop(name = "StateMachine Demo")
public class StateMachineTeleop extends PeriodicOpMode {
  private final CommandNiDsXboxController driver = new CommandNiDsXboxController(0);
  private final Command machine;

  public StateMachineTeleop(Robot robot) {
    // 1. Construct - the name is required and shows up in telemetry.
    StateMachine sm = new StateMachine("Superstructure");

    // 2. Add states. Each state owns one command. The presets hold their pose forever (the machine
    //    cancels them on a transition); prep's command FINISHES when the arm reaches the scoring
    //    angle, which is what lets it use a completion transition below.
    State stowed = sm.addState(robot.stow());
    State pickup = sm.addState(robot.intake());
    State prep = sm.addState(robot.arm.scoringAndWait()); // finishes when the arm arrives
    State scoring = sm.addState(robot.score());

    // 3. Every machine needs a starting state.
    sm.setInitialState(stowed);

    // 4. Wire transitions. when(...) conditions are checked every scheduler tick while their state
    //    is active, and fire on the rising edge (false -> true).
    stowed.switchTo(pickup).when(driver.leftTrigger()); // driver asks to intake
    pickup.switchTo(prep).when(robot.arm::isAtTarget); // arm reached the ground - on a real
    // robot this would be a game-piece sensor ("we have a piece"), not the arm angle

    // Completion transition: prep's command ends on its own (the arm reached the scoring angle),
    // so use whenComplete() - checked once, when the command finishes - instead of when(...).
    prep.switchTo(scoring).whenComplete();

    scoring.switchTo(stowed).when(driver.rightTrigger()); // shot taken - pack up

    // Any-state interrupt: B means "get safe now", no matter which state is active.
    // switchFromAny() with no args applies to every state added so far, so declare it last.
    sm.switchFromAny().to(stowed).when(driver.b());

    // Entry/exit hooks: side effects on the way in and out of a state, without touching the
    // state's command. These write markers into the .wpilog, so when you read the log later you
    // can see exactly when the machine entered and left Scoring (see the log-reading skill).
    scoring.onEnter(() -> DataLogManager.log("Superstructure: entered Scoring"));
    scoring.onExit(() -> DataLogManager.log("Superstructure: left Scoring"));

    // More power, when you need it (uncomment and adapt):
    //
    // whenCompleteAnd = whenComplete + an extra check; it wins over plain whenComplete().
    //   prep.switchTo(stowed).whenCompleteAnd(() -> !hasGamePiece()); // lost the piece - bail
    //
    // The target can be picked at transition time (a Supplier<State> instead of a State):
    //   prep.switchTo(() -> hasGamePiece() ? scoring : stowed).whenComplete();
    //
    // A transition can also end the whole machine instead of moving to another state:
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
