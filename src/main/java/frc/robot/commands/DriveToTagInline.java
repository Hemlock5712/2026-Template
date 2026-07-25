// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;
import com.limelightvision.Limelight;
import com.limelightvision.Limelight.FiducialTarget;
import frc.robot.subsystems.DriveMechanism;
import org.wpilib.command3.Command;
import org.wpilib.command3.Coroutine;
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
 * <p>Inline-style Commands v3 command: one coroutine body does everything. {@link DriveToTag} is
 * the same command written with classic lifecycle hooks - compare the two styles.
 */
public final class DriveToTagInline {
  private DriveToTagInline() {}

  /**
   * Creates the drive-to-tag command. The controllers are locals, so each schedule starts fresh.
   */
  public static Command create(DriveMechanism drivetrain, Limelight camera, int targetTagId) {
    // One trapezoidal PID per axis; the profile inside each limits speed and acceleration. kP is
    // 0: the feedforward below does the work, PID only corrects drift. TODO: tune the limits
    // and kP.
    ProfiledPIDController distance =
        new ProfiledPIDController(0.0, 0.0, 0.0, new TrapezoidProfile.Constraints(2.5, 3.0));
    ProfiledPIDController lateral =
        new ProfiledPIDController(0.0, 0.0, 0.0, new TrapezoidProfile.Constraints(2.5, 3.0));
    ProfiledPIDController heading =
        new ProfiledPIDController(
            0.0, 0.0, 0.0, new TrapezoidProfile.Constraints(Math.PI, 2.0 * Math.PI));

    // Robot-relative velocity request; open-loop so no drive-velocity PID tuning is needed.
    SwerveRequest.ApplyRobotVelocity driveRequest =
        new SwerveRequest.ApplyRobotVelocity()
            .withDriveRequestType(DriveRequestType.OpenLoopVoltage);

    heading.enableContinuousInput(-Math.PI, Math.PI);
    distance.setTolerance(0.03); // meters
    lateral.setTolerance(0.03); // meters
    heading.setTolerance(Math.toRadians(2.0)); // radians

    return drivetrain
        .run(
            (Coroutine coroutine) -> {
              // Make our tag the camera's primary target. In the lambda (not create()) so it
              // re-applies on every schedule.
              camera.setPriorityTagID(targetTagId);

              // initialize: seed the profiles to the current measurement so the approach starts
              // from a standstill. No tag in view? Skip it - the loop holds still until it
              // appears.
              Pose3d robotInTag = readRobotInTag(camera, targetTagId);
              if (robotInTag != null) {
                distance.reset(robotInTag.getX());
                lateral.reset(robotInTag.getY());
                heading.reset(robotInTag.getRotation().getZ());
              }

              // execute + isFinished: one pass per robot loop.
              while (true) {
                // No tag in view - hold still and wait.
                robotInTag = readRobotInTag(camera, targetTagId);
                if (robotInTag == null) {
                  drivetrain.setControl(new SwerveRequest.Idle());
                  coroutine.yield();
                  continue;
                }

                // Robot pose in the tag's frame: +X out of the tag face, +Y to the tag's left. At
                // the goal the robot faces the tag, so its yaw there is ±pi, not 0.
                double measuredDistance = robotInTag.getX();
                double measuredLateral = robotInTag.getY();
                double measuredYaw = robotInTag.getRotation().getZ();

                // PID + profile-velocity feedforward: the feedforward does the work, PID corrects
                // drift.
                double vx =
                    distance.calculate(measuredDistance, 0.0) + distance.getSetpoint().velocity;
                double vy =
                    lateral.calculate(measuredLateral, 0.0) + lateral.getSetpoint().velocity;
                double omega =
                    heading.calculate(measuredYaw, Math.PI) + heading.getSetpoint().velocity;

                // The velocities are in the tag's frame - rotate them into the robot's frame.
                ChassisVelocities body =
                    new ChassisVelocities(vx, vy, omega)
                        .toRobotRelative(Rotation2d.fromRadians(measuredYaw));
                drivetrain.setControl(driveRequest.withVelocity(body));

                // Done when all three controllers are at-goal. The visibility check above
                // matters: a fresh controller reports at-goal before its first calculate().
                if (distance.atGoal() && lateral.atGoal() && heading.atGoal()) {
                  break;
                }
                coroutine.yield();
              }

              // end: stop the drivetrain and clear the tag priority.
              drivetrain.setControl(new SwerveRequest.Idle());
              camera.setPriorityTagID(-1); // -1 = no priority
            })
        .whenCanceled(
            () -> {
              drivetrain.setControl(new SwerveRequest.Idle());
              camera.setPriorityTagID(-1);
            })
        .named("DriveToTag");
  }

  /**
   * The robot's pose in our tag's frame, or null when the camera doesn't see that tag. We search
   * the list for our ID so we can never align to the wrong tag.
   */
  private static Pose3d readRobotInTag(Limelight camera, int targetTagId) {
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
