// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;
import com.limelightvision.Limelight;
import com.limelightvision.Limelight.FiducialTarget;
import frc.robot.subsystems.DriveMechanism;
import frc.robot.utils.ClassicCommand;
import org.wpilib.math.controller.ProfiledPIDController;
import org.wpilib.math.geometry.Pose3d;
import org.wpilib.math.geometry.Rotation2d;
import org.wpilib.math.kinematics.ChassisVelocities;
import org.wpilib.math.trajectory.TrapezoidProfile;

/**
 * Drive to a standoff point in front of an AprilTag, vision-only (no odometry).
 *
 * <p>The standoff distance is set on the camera (the Limelight's point-of-interest offset, in the
 * LL web UI), so this command just drives the measured distance to zero.
 *
 * <p>Classic-style Commands v3 command on {@link ClassicCommand}: the familiar {@code
 * initialize/execute/isFinished/end} hooks. {@link DriveToTagInline} is the same command written as
 * one coroutine body - compare the two styles.
 */
public class DriveToTag extends ClassicCommand {
  private final DriveMechanism drivetrain;

  // Which camera to read and which AprilTag to align to.
  private final Limelight camera;
  private final int targetTagId;

  // One trapezoidal PID per axis; the profile inside each limits speed and acceleration. kP is 0:
  // the feedforward in execute() does the work, PID only corrects drift. TODO: tune the limits
  // and kP.
  private final ProfiledPIDController distance =
      new ProfiledPIDController(0.0, 0.0, 0.0, new TrapezoidProfile.Constraints(2.5, 3.0));
  private final ProfiledPIDController lateral =
      new ProfiledPIDController(0.0, 0.0, 0.0, new TrapezoidProfile.Constraints(2.5, 3.0));
  private final ProfiledPIDController heading =
      new ProfiledPIDController(
          0.0, 0.0, 0.0, new TrapezoidProfile.Constraints(Math.PI, 2.0 * Math.PI));

  // Robot-relative velocity request; open-loop so no drive-velocity PID tuning is needed.
  private final SwerveRequest.ApplyRobotVelocity driveRequest =
      new SwerveRequest.ApplyRobotVelocity().withDriveRequestType(DriveRequestType.OpenLoopVoltage);

  // Latest reading, cached by execute() so isFinished() can reuse it. Null = tag not in view.
  private Pose3d robotInTag = null;

  public DriveToTag(DriveMechanism drivetrain, Limelight camera, int targetTagId) {
    super("DriveToTag", drivetrain); // name + requirement, like v2 addRequirements(drivetrain)
    this.drivetrain = drivetrain;
    this.camera = camera;
    this.targetTagId = targetTagId;
    heading.enableContinuousInput(-Math.PI, Math.PI);
    distance.setTolerance(0.03); // meters
    lateral.setTolerance(0.03); // meters
    heading.setTolerance(Math.toRadians(2.0)); // radians
  }

  /**
   * Seeds the profiles to the current measurement so the approach starts from a standstill. No tag
   * in view? Skip it - {@link #execute} holds still until the tag appears.
   */
  @Override
  protected void initialize() {
    // Make our tag the camera's primary target.
    camera.setPriorityTagID(targetTagId);

    robotInTag = readRobotInTag();
    if (robotInTag == null) {
      return;
    }

    distance.reset(robotInTag.getX());
    lateral.reset(robotInTag.getY());
    heading.reset(robotInTag.getRotation().getZ());
  }

  @Override
  protected void execute() {
    // No tag in view - hold still and wait. robotInTag stays null so isFinished() can't fire.
    robotInTag = readRobotInTag();
    if (robotInTag == null) {
      drivetrain.setControl(new SwerveRequest.Idle());
      return;
    }

    // Robot pose in the tag's frame: +X out of the tag face, +Y to the tag's left. At the goal
    // the robot faces the tag, so its yaw there is ±pi, not 0.
    double measuredDistance = robotInTag.getX();
    double measuredLateral = robotInTag.getY();
    double measuredYaw = robotInTag.getRotation().getZ();

    // PID + profile-velocity feedforward: the feedforward does the work, PID corrects drift.
    double vx = distance.calculate(measuredDistance, 0.0) + distance.getSetpoint().velocity;
    double vy = lateral.calculate(measuredLateral, 0.0) + lateral.getSetpoint().velocity;
    double omega = heading.calculate(measuredYaw, Math.PI) + heading.getSetpoint().velocity;

    // The velocities are in the tag's frame - rotate them into the robot's frame.
    ChassisVelocities body =
        new ChassisVelocities(vx, vy, omega).toRobotRelative(Rotation2d.fromRadians(measuredYaw));
    drivetrain.setControl(driveRequest.withVelocity(body));
  }

  /**
   * Done when we see our tag AND all three controllers are at-goal. The visibility check matters: a
   * fresh controller reports at-goal before its first calculate().
   */
  @Override
  protected boolean isFinished() {
    return robotInTag != null && distance.atGoal() && lateral.atGoal() && heading.atGoal();
  }

  /** Stops the drivetrain and clears the tag priority. Runs on finish and interruption. */
  @Override
  protected void end(boolean interrupted) {
    drivetrain.setControl(new SwerveRequest.Idle());
    camera.setPriorityTagID(-1); // -1 = no priority
  }

  /**
   * The robot's pose in our tag's frame, or null when the camera doesn't see that tag. We search
   * the list for our ID so we can never align to the wrong tag.
   */
  private Pose3d readRobotInTag() {
    if (!camera.hasTarget()) { // also false if the camera is disconnected
      return null;
    }
    for (FiducialTarget target : camera.getLatestResults().fiducialTargets) {
      if (target.fiducialId == targetTagId) {
        Pose3d pose = target.getRobotPose_TargetSpace();
        // All-zero pose = no target-space data for this tag yet.
        return pose.equals(Pose3d.kZero) ? null : pose;
      }
    }
    return null;
  }
}
