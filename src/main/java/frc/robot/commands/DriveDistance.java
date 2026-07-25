// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;
import frc.robot.subsystems.DriveMechanism;
import frc.robot.utils.ClassicCommand;
import org.wpilib.math.controller.PIDController;
import org.wpilib.math.geometry.Pose2d;
import org.wpilib.math.kinematics.ChassisVelocities;

/**
 * Drive straight forward a set distance, then stop. <b>Your first closed-loop drive command.</b>
 *
 * <p>"Closed loop" means the robot keeps checking where it actually is and adjusts. The loop is:
 *
 * <ol>
 *   <li>How far have I gone? (ask odometry)
 *   <li>How far is that from where I want to be? (that difference is the <b>error</b>)
 *   <li>Drive at a speed based on that error - big error, drive fast; small error, slow down.
 *   <li>Repeat, about 50 times a second, until the error is tiny.
 * </ol>
 *
 * <p>Step 3 is all a {@link PIDController} with just a <b>P</b> gain does: speed = kP × error.
 *
 * <p>Everything is measured relative to where the robot started, and it drives in its own forward
 * direction, so this command doesn't care about the field origin or which alliance you're on.
 * {@link DriveToPose} is the grown-up version that does: it drives to a specific spot on the field
 * and adds a motion profile so the speed ramps smoothly instead of just being proportional.
 */
public class DriveDistance extends ClassicCommand {
  private final DriveMechanism drivetrain;
  private final double targetMeters;

  // Speed cap, m/s - the error is huge at the start.
  private static final double MAX_SPEED_MPS = 2.0;

  // Bigger = tries harder to close the gap. Too big overshoots; too small stops short, because
  // the last slow crawl can't beat the drivetrain's friction. Re-tune on the real robot.
  private final PIDController controller = new PIDController(6.0, 0.0, 0.0);

  // Robot-relative: +x is straight out the front of the robot, whichever way it's pointing.
  private final SwerveRequest.ApplyRobotVelocity driveRequest =
      new SwerveRequest.ApplyRobotVelocity().withDriveRequestType(DriveRequestType.OpenLoopVoltage);

  // Where we were when the command started - the distance is measured from here.
  private Pose2d startPose = Pose2d.kZero;

  /**
   * @param drivetrain the swerve drive to command
   * @param targetMeters how far forward to drive, in meters
   */
  public DriveDistance(DriveMechanism drivetrain, double targetMeters) {
    super("DriveDistance", drivetrain); // name + requirement
    this.drivetrain = drivetrain;
    this.targetMeters = targetMeters;
    // Done when within 2 cm AND slower than 0.1 m/s - the speed part stops the robot counting as
    // "arrived" while still coasting through the target.
    controller.setTolerance(0.02, 0.1);
  }

  /** Remembers the starting spot, so "how far have I gone" has something to measure from. */
  @Override
  protected void initialize() {
    startPose = drivetrain.getPose();
    controller.reset();
  }

  @Override
  protected void execute() {
    // Step 1 + 2: how far we've gone, and how much is left.
    double traveled = distanceTraveled();
    double speed = controller.calculate(traveled, targetMeters);

    // Step 3: don't let it ask for a silly speed at the start, when the error is large.
    speed = Math.max(-MAX_SPEED_MPS, Math.min(MAX_SPEED_MPS, speed));

    drivetrain.setControl(driveRequest.withVelocity(new ChassisVelocities(speed, 0.0, 0.0)));
  }

  /** Done once we're within tolerance of the target distance. */
  @Override
  protected boolean isFinished() {
    return controller.atSetpoint();
  }

  /** Stops the drivetrain. Runs on both finish and interruption. */
  @Override
  protected void end(boolean interrupted) {
    drivetrain.setControl(new SwerveRequest.Idle());
  }

  /**
   * How far we've gone in the direction we started out facing. <b>Signed</b> - negative means we're
   * behind the start. Plain "distance between two points" is never negative, so being pushed
   * backwards would look like driving forwards and the robot would chase itself the wrong way.
   */
  private double distanceTraveled() {
    var moved = drivetrain.getPose().getTranslation().minus(startPose.getTranslation());
    return moved.rotateBy(startPose.getRotation().unaryMinus()).getX();
  }
}
