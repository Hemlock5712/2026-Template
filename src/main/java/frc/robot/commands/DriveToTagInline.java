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
 * <p>The standoff isn't a code parameter: it's the Limelight's 3D point-of-interest offset (set in
 * the LL web UI, per-fiducial in the field map), so this command just drives the measured distance
 * to zero. Change the standoff by editing the POI offset on the camera.
 *
 * <p>"Inline-style" Commands v3 command: a single linear coroutine body does the work - seed the
 * profiles, then a while-loop that reads the camera, drives the three trapezoidal PIDs, and yields
 * each iteration until at-goal. The whole feature - Limelight read, three trapezoidal PIDs, drive
 * request, done-condition - lives in this one file.
 *
 * <p><b>Two styles, same behavior:</b> {@link DriveToTag} is this exact command written with the
 * classic {@code initialize/execute/isFinished/end} lifecycle on {@link
 * frc.robot.utils.ClassicCommand}. Compare the two to see the trade-off - one linear body here vs.
 * explicit lifecycle hooks there.
 */
public final class DriveToTagInline {
  private DriveToTagInline() {}

  /**
   * Creates the drive-to-tag command. The controllers are locals, so each schedule starts fresh.
   */
  public static Command create(DriveMechanism drivetrain, Limelight camera, int targetTagId) {
    // One trapezoidal PID per axis. The profile inside each (max velocity, max accel) gives
    // acceleration limiting - a plain PIDController would command full output instantly.
    // Translation
    // limits are m/s and m/s^2, rotation rad/s and rad/s^2. The kP gains default to 0: the
    // feedforward below does the work, PID only corrects drift. TODO: tune both - the limits to the
    // drivetrain's real capability, and kP (raise if the bot trails the profile; lower, or add kD,
    // if it oscillates near the goal).
    ProfiledPIDController distance =
        new ProfiledPIDController(0.0, 0.0, 0.0, new TrapezoidProfile.Constraints(2.5, 3.0));
    ProfiledPIDController lateral =
        new ProfiledPIDController(0.0, 0.0, 0.0, new TrapezoidProfile.Constraints(2.5, 3.0));
    ProfiledPIDController heading =
        new ProfiledPIDController(
            0.0, 0.0, 0.0, new TrapezoidProfile.Constraints(Math.PI, 2.0 * Math.PI));

    // Robot-relative velocity request: open-loop drive so no drive-velocity PID tuning is required.
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
              // Make our tag the camera's primary target (its own crosshair/telemetry track it
              // too). In the lambda (not create()) so it re-applies on every schedule.
              camera.setPriorityTagID(targetTagId);

              // initialize: seed the profiles to the current measurement so the approach starts
              // from a standstill. If no tag is in view, skip seeding: the loop below holds still
              // until the tag appears and the finish check gates on visibility, so we won't
              // falsely report "done."
              Pose3d robotInTag = readRobotInTag(camera, targetTagId);
              if (robotInTag != null) {
                distance.reset(robotInTag.getX());
                lateral.reset(robotInTag.getY());
                heading.reset(robotInTag.getRotation().getZ());
              }

              // execute + isFinished: runs every robot loop while the command is active.
              while (true) {
                // If the camera doesn't see our tag (wrong tag, or none in view), don't drive -
                // idle and wait for it.
                robotInTag = readRobotInTag(camera, targetTagId);
                if (robotInTag == null) {
                  drivetrain.setControl(new SwerveRequest.Idle());
                  coroutine.yield();
                  continue;
                }

                // Robot pose in the tag's frame (2027 convention: +X out of the tag face, +Y to
                // the tag's left, +Z up). At the goal the robot sits at the POI standoff (X = 0,
                // Y = 0) facing the tag - and since robot-forward then points INTO the tag face,
                // "facing the tag" is yaw = ±pi, not 0.
                double measuredDistance = robotInTag.getX(); // + = meters out from the tag face
                double measuredLateral = robotInTag.getY(); // + = meters toward the tag's left
                double measuredYaw = robotInTag.getRotation().getZ(); // robot yaw in the tag frame

                // Tag-frame velocities: PID + profile-velocity feedforward. FF commands the
                // profile's velocity directly; PID only corrects drift. Without FF the robot
                // trails the setpoint.
                double vx =
                    distance.calculate(measuredDistance, 0.0) + distance.getSetpoint().velocity;
                double vy =
                    lateral.calculate(measuredLateral, 0.0) + lateral.getSetpoint().velocity;
                double omega =
                    heading.calculate(measuredYaw, Math.PI) + heading.getSetpoint().velocity;

                // The velocities above are in the tag's frame; the robot sits rotated measuredYaw
                // within it, so rotate them into the body frame and command the swerve.
                ChassisVelocities body =
                    new ChassisVelocities(vx, vy, omega)
                        .toRobotRelative(Rotation2d.fromRadians(measuredYaw));
                drivetrain.setControl(driveRequest.withVelocity(body));

                // Done when we see our tag AND all three controllers are at-goal. The visibility
                // gate above avoids a false "done" before the first calculate() sets the goal - a
                // fresh controller reports at-goal because its goal and setpoint both default to
                // zero.
                if (distance.atGoal() && lateral.atGoal() && heading.atGoal()) {
                  break;
                }
                coroutine.yield();
              }

              // end: idle the drivetrain and clear the tag priority.
              drivetrain.setControl(new SwerveRequest.Idle());
              camera.setPriorityTagID(-1); // back to normal targeting
            })
        .whenCanceled(
            () -> {
              drivetrain.setControl(new SwerveRequest.Idle());
              camera.setPriorityTagID(-1);
            })
        .named("DriveToTag");
  }

  /**
   * The robot's pose in the target tag's frame, or null when the camera doesn't currently see that
   * tag. Searching the fiducial list for our ID (instead of trusting whichever tag the camera calls
   * "primary") means we can never align to the wrong tag.
   */
  private static Pose3d readRobotInTag(Limelight camera, int targetTagId) {
    if (!camera.hasTarget()) { // also false when the camera is disconnected or the frame is stale
      return null;
    }
    for (FiducialTarget target : camera.getLatestResults().fiducialTargets) {
      if (target.fiducialId == targetTagId) {
        Pose3d pose = target.getRobotPose_TargetSpace();
        // An all-zero pose means the camera hasn't filled target-space data for this tag yet.
        return pose.equals(Pose3d.kZero) ? null : pose;
      }
    }
    return null;
  }
}
