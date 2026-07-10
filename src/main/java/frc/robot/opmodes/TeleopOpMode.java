// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.opmodes;

import static org.wpilib.units.Units.MetersPerSecond;
import static org.wpilib.units.Units.RadiansPerSecond;
import static org.wpilib.units.Units.RotationsPerSecond;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;
import frc.robot.Robot;
import frc.robot.commands.DriveToTag;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.DriveMechanism;
import org.wpilib.command3.button.CommandNiDsXboxController;
import org.wpilib.opmode.PeriodicOpMode;
import org.wpilib.opmode.Teleop;

/**
 * Driver teleop. This is the OpMode-model replacement for {@code RobotContainer.configureBindings}:
 * a self-contained class for one driver experience. The framework constructs it when "Teleop" is
 * selected on the driver station and discards it on a mode switch; the button bindings created in
 * the constructor are scoped to this OpMode, so they are removed automatically - no cleanup needed.
 *
 * <p>The drivetrain's default command (joystick drive) lives here rather than on the {@link Robot}
 * because it depends on this OpMode's controller. Add a second {@code @Teleop} class (e.g. a demo
 * or single-driver layout) and it shows up as another choice on the driver station.
 */
@Teleop(name = "Teleop")
public class TeleopOpMode extends PeriodicOpMode {
  // Which Limelight to align with, and the AprilTag to align to. TODO: pick the real scoring tag
  // (and flip per alliance) once the game is wired - see the game-info conventions.
  private static final String ALIGN_CAMERA = "limelight";
  private static final int ALIGN_TAG_ID = 1;

  private final double maxSpeed = TunerConstants.kSpeedAt12Volts.in(MetersPerSecond); // top speed
  private final double maxAngularRate = RotationsPerSecond.of(0.75).in(RadiansPerSecond); // 3/4 rps

  private final SwerveRequest.FieldCentric drive =
      new SwerveRequest.FieldCentric()
          .withDeadband(maxSpeed * 0.1)
          .withRotationalDeadband(maxAngularRate * 0.1) // 10% stick deadband
          .withDriveRequestType(DriveRequestType.OpenLoopVoltage); // open-loop drive motors

  private final CommandNiDsXboxController driver = new CommandNiDsXboxController(0);

  public TeleopOpMode(Robot robot) {
    final DriveMechanism drivetrain = robot.drivetrain;

    // Note that X is defined as forward according to WPILib convention,
    // and Y is defined as to the left according to WPILib convention.
    drivetrain.setDefaultCommand(
        drivetrain.applyRequest(
            () ->
                drive
                    .withVelocityX(-driver.getLeftY() * maxSpeed) // forward with negative Y
                    .withVelocityY(-driver.getLeftX() * maxSpeed) // left with negative X
                    .withRotationalRate(-driver.getRightX() * maxAngularRate))); // CCW with -X

    // Reset the field-centric heading on left bumper press.
    driver.leftBumper().onTrue(drivetrain.seedFieldCentric());

    // Superstructure presets (arm + flywheel move together), held while the button is down.
    driver.leftTrigger().whileTrue(robot.intake()); // pick up game piece
    driver.rightBumper().whileTrue(robot.score()); // prepare to score
    driver.rightTrigger().whileTrue(robot.stow()); // back to safe travel pose

    // Hold A: vision-only auto-align to the tag standoff.
    driver.a().whileTrue(new DriveToTag(drivetrain, ALIGN_CAMERA, ALIGN_TAG_ID));

    // Hold Y: auto-score prep - raise the arm and spin up the flywheel together.
    driver.y().whileTrue(robot.autoScore()).whileFalse(robot.flywheel.stop());
  }
}
