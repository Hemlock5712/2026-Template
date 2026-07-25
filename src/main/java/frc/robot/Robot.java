// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.ctre.phoenix6.swerve.SwerveRequest;
import com.limelightvision.Limelight;
import frc.robot.subsystems.DriveMechanism;
import frc.robot.subsystems.arm.Arm;
import frc.robot.subsystems.flywheel.Flywheel;
import frc.robot.subsystems.vision.Vision;
import frc.robot.utils.SimStartup;
import org.littletonrobotics.junction.AutoLogOutputManager;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.networktables.NT4Publisher;
import org.littletonrobotics.junction.wpilog.WPILOGWriter;
import org.wpilib.command3.Command;
import org.wpilib.command3.Scheduler;
import org.wpilib.command3.button.RobotModeTriggers;
import org.wpilib.framework.OpModeRobot;

/**
 * Owns the robot's shared hardware. There is no {@code RobotContainer}: subsystems live here as
 * public fields, and each OpMode in {@code frc.robot.opmodes} reaches them through the {@link
 * Robot} it is constructed with.
 */
public class Robot extends OpModeRobot {
  public final DriveMechanism drivetrain = new DriveMechanism();

  /* Example mechanisms. The poses that combine them (stow/intake/score) are at the bottom. */
  public final Arm arm = new Arm();
  public final Flywheel flywheel = new Flywheel();

  /* One Limelight per camera, named by its NT name. Vision.registerAll wires them into the pose
   * estimator; an OpMode can also hand one to DriveToTag. */
  public final Limelight limelightBR = new Limelight("limelight-br");
  public final Limelight limelightBL = new Limelight("limelight-bl");

  public Robot() {
    // AdvantageKit logging: .wpilog file (./logs in sim, USB on the robot) plus live
    // NetworkTables for AdvantageScope. See the log-reading skill.
    Logger.recordMetadata("ProjectName", "2027-Template");
    Logger.addDataReceiver(new WPILOGWriter());
    Logger.addDataReceiver(new NT4Publisher());
    Logger.AdvancedHooks.disableRobotBaseCheck(); // we extend OpModeRobot, not LoggedRobot
    Logger.start();
    AutoLogOutputManager.addObject(this);

    // Brake while disabled, in every mode. Made here so the binding survives OpMode switches.
    final var idle = new SwerveRequest.Idle();
    RobotModeTriggers.disabled().whileTrue(drivetrain.applyRequest(() -> idle));

    Vision.registerAll(drivetrain, limelightBR, limelightBL);
  }

  @Override
  public void simulationInit() {
    // Headless auto-enable for agent/CI runs; no-op unless -Dfrc.sim.startMode is set.
    // See the run-sim skill.
    SimStartup.arm();
  }

  @Override
  public void robotPeriodic() {
    // AdvantageKit: flush the previous logging cycle, start this one.
    Logger.AdvancedHooks.invokePeriodicAfterUser(0, 0);
    Logger.AdvancedHooks.invokePeriodicBeforeUser();
    Scheduler.getDefault().run();
  }

  // ---------------------------------------------------------------------------
  // Superstructure: one method per robot "pose" so an OpMode can just call
  // robot.stow(). Each runs the arm and flywheel in parallel and requires both.
  // ---------------------------------------------------------------------------

  /** Stow for travel: arm vertical, flywheel stopped. Holds forever - never wait on it. */
  public Command stow() {
    return Command.parallel(arm.vertical(), flywheel.stop()).named("Stow (hold)");
  }

  /** Ground intake: arm down, flywheel stopped. Holds forever. */
  public Command intake() {
    return Command.parallel(arm.horizontal(), flywheel.stop()).named("Intake (hold)");
  }

  /** Prepare to score: arm up, flywheel spinning. Holds forever. */
  public Command score() {
    return Command.parallel(arm.scoring(), flywheel.spinUp()).named("Score (hold)");
  }

  /**
   * Like {@link #score()}, but {@code .until(...)} gives the arm's hold a finish line. The flywheel
   * still spins forever, so the whole group is still a hold.
   */
  public Command autoScore() {
    return Command.parallel(
            arm.scoring().until(arm::isAtTarget).named("scoring until at target"),
            flywheel.spinUp())
        .named("AutoScore (hold)");
  }
}
