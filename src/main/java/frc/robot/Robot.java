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
 * Owns the robot's shared hardware in one place. With the OpMode framework there is no {@code
 * RobotContainer}: the subsystems live here as public fields, and each OpMode in {@code
 * frc.robot.opmodes} reaches them through the {@link Robot} reference it is constructed with.
 *
 * <p>The framework auto-discovers the {@code @Teleop}/{@code @Autonomous} classes in this package
 * (and subpackages) and handles every mode transition, so this class has no per-mode init/periodic
 * methods - only the always-on scheduler tick. Selecting a different mode on the driver station
 * constructs that OpMode and tears down the previous one (its button bindings are scoped to it and
 * removed automatically).
 */
public class Robot extends OpModeRobot {
  public final DriveMechanism drivetrain = new DriveMechanism();

  /* Example mechanisms. The superstructure poses that coordinate them live at the bottom of this
   * class (stow / intake / score / autoScore) so an OpMode can just call robot.stow(). */
  public final Arm arm = new Arm();
  public final Flywheel flywheel = new Flywheel();

  /* Vision hardware: one LimelightLib object per camera, constructed with the camera's NT name.
   * Owned here like any other hardware - Vision.registerAll wires them into the pose estimator,
   * and an OpMode can hand one to DriveToTag (robot.limelightBR) to align to a tag. */
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

    // Brake while disabled, in every mode. Created here (before any OpMode is selected) so the
    // binding is global; the opmodes' bindings are scoped to their OpMode and removed on a switch.
    final var idle = new SwerveRequest.Idle();
    RobotModeTriggers.disabled().whileTrue(drivetrain.applyRequest(() -> idle));

    // Vision: wire up every Limelight in one call.
    Vision.registerAll(drivetrain, limelightBR, limelightBL);
  }

  @Override
  public void simulationInit() {
    // Headless auto-enable for agent / CI runs. No-op unless -Dfrc.sim.startMode is set (the
    // simulateJavaAgent Gradle task sets it). See SimStartup and the run-sim skill.
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
  // Superstructure - coordinates the Arm and Flywheel so an OpMode gets one
  // method per robot "pose" instead of juggling both mechanisms by hand. These
  // live here (rather than in a separate class) so an OpMode reaches them the
  // same way it reaches the hardware: robot.stow(), robot.score(), and so on.
  //
  // Each method returns a command composed of arm and flywheel commands. Because
  // a command inherits its children's requirements, the result requires both
  // subsystems, and Command.parallel(...) runs them at the same time.
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
   * Auto-score prep: raise the arm to its scoring pose and hold shooting speed. Like {@link
   * #score()}, but {@code .until(...)} gives the arm's hold a finish line - that's the pattern for
   * making any hold finish, applied at the call site. {@code spinUp} still runs forever, so the
   * group as a whole is a hold too.
   */
  public Command autoScore() {
    return Command.parallel(
            arm.scoring().until(arm::isAtTarget).named("scoring until at target"),
            flywheel.spinUp())
        .named("AutoScore (hold)");
  }
}
