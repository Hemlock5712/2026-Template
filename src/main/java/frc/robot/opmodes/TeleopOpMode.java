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
 * Driver teleop - the OpMode replacement for {@code RobotContainer.configureBindings}. Bindings
 * made in the constructor are scoped to this OpMode and removed on a mode switch. Add another
 * {@code @Teleop} class for a second driver layout.
 */
@Teleop(name = "Teleop")
public class TeleopOpMode extends PeriodicOpMode {
  // Which AprilTag to auto-align to. TODO: pick the real camera and tag ID (flipped per
  // alliance) once the game is wired - see game-info.
  private static final int ALIGN_TAG_ID = 1;

  private final double maxSpeed = TunerConstants.kSpeedAt12Volts.in(MetersPerSecond);
  private final double maxAngularRate = RotationsPerSecond.of(0.75).in(RadiansPerSecond);

  private final SwerveRequest.FieldCentric drive =
      new SwerveRequest.FieldCentric()
          .withDeadband(maxSpeed * 0.1)
          .withRotationalDeadband(maxAngularRate * 0.1) // 10% stick deadband
          .withDriveRequestType(DriveRequestType.OpenLoopVoltage);

  private final CommandNiDsXboxController driver = new CommandNiDsXboxController(0);

  public TeleopOpMode(Robot robot) {
    final DriveMechanism drivetrain = robot.drivetrain;

    // WPILib axes: X is forward, Y is left - hence the minus signs on the sticks.
    drivetrain.setDefaultCommand(
        drivetrain.applyRequest(
            () ->
                drive
                    .withVelocityX(-driver.getLeftY() * maxSpeed) // forward with negative Y
                    .withVelocityY(-driver.getLeftX() * maxSpeed) // left with negative X
                    .withRotationalRate(-driver.getRightX() * maxAngularRate))); // CCW with -X

    // Left bumper: reset which way is "forward".
    driver.leftBumper().onTrue(drivetrain.seedFieldCentric());

    // Superstructure presets (arm + flywheel move together), held while the button is down.
    driver.leftTrigger().whileTrue(robot.intake()); // pick up game piece
    driver.rightBumper().whileTrue(robot.score()); // prepare to score
    driver.rightTrigger().whileTrue(robot.stow()); // back to safe travel pose

    // Hold A: vision-only auto-align to the tag standoff.
    driver.a().whileTrue(new DriveToTag(drivetrain, robot.limelightBR, ALIGN_TAG_ID));

    // Hold Y: auto-score prep.
    driver.y().whileTrue(robot.autoScore()).whileFalse(robot.flywheel.stop());
  }
}
