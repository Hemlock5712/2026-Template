// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import com.ctre.phoenix6.Utils;
import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;
import com.ctre.phoenix6.swerve.utility.LinearPath;
import frc.robot.subsystems.DriveMechanism;
import frc.robot.utils.ClassicCommand;
import org.wpilib.math.controller.PIDController;
import org.wpilib.math.geometry.Pose2d;
import org.wpilib.math.kinematics.ChassisVelocities;
import org.wpilib.math.trajectory.TrapezoidProfile;

/**
 * Drive in a straight line to a field pose using odometry - the odometry twin of {@link DriveToTag}
 * (which is vision-only). Each loop samples a straight-line profile: the profile's velocity is the
 * feedforward that moves the robot, and X/Y/heading PID trims drift back onto the line.
 *
 * <p>Classic-style Commands v3 command on {@link ClassicCommand}, like {@link DriveToTag}.
 */
public class DriveToPose extends ClassicCommand {
  private final DriveMechanism drivetrain;
  private final Pose2d goal;

  // Straight-line profile limits: translation (m/s, m/s^2) and rotation (rad/s, rad/s^2).
  // TODO: tune to the drivetrain's real capability.
  private final LinearPath path =
      new LinearPath(
          new TrapezoidProfile.Constraints(2.5, 3.0),
          new TrapezoidProfile.Constraints(Math.PI, 2.0 * Math.PI));

  // Trims drift back onto the profile (the feedforward does the real work). Raise kP if the robot
  // lags or stops short; lower it if it oscillates. TODO: tune.
  private final PIDController xController = new PIDController(3.0, 0.0, 0.0);
  private final PIDController yController = new PIDController(3.0, 0.0, 0.0);
  private final PIDController headingController = new PIDController(4.0, 0.0, 0.0);

  // Field-relative velocity request, blue-origin (the same frame as odometry); open-loop so no
  // drive-velocity PID tuning is needed.
  private final SwerveRequest.ApplyFieldVelocity driveRequest =
      new SwerveRequest.ApplyFieldVelocity()
          .withForwardPerspective(SwerveRequest.ForwardPerspectiveValue.BlueAlliance)
          .withDriveRequestType(DriveRequestType.OpenLoopVoltage);

  // Captured once at start: the pose + velocity the trajectory is generated from.
  private LinearPath.State startState = new LinearPath.State();
  // Trajectory time t = now - startTime.
  private double startTime;

  /**
   * @param drivetrain the swerve drive to command
   * @param goal the field pose (blue-origin) to drive to, including the goal heading
   */
  public DriveToPose(DriveMechanism drivetrain, Pose2d goal) {
    super("DriveToPose", drivetrain); // name + requirement, like v2 addRequirements(drivetrain)
    this.drivetrain = drivetrain;
    this.goal = goal;
    headingController.enableContinuousInput(-Math.PI, Math.PI);
  }

  /** Captures the start state and starts the clock. */
  @Override
  protected void initialize() {
    startState = new LinearPath.State(drivetrain.getPose(), drivetrain.getFieldVelocity());
    startTime = Utils.getCurrentTimeSeconds();
    xController.reset();
    yController.reset();
    headingController.reset();
  }

  @Override
  protected void execute() {
    // Sample the profile at the elapsed time since start.
    double t = Utils.getCurrentTimeSeconds() - startTime;
    LinearPath.State setpoint = path.calculate(t, startState, goal);

    Pose2d measuredPose = drivetrain.getPose();

    // Feedforward = the profile's velocity; PID adds a small correction toward the profiled pose.
    ChassisVelocities feedforward = setpoint.velocity;
    double vx = feedforward.vx + xController.calculate(measuredPose.getX(), setpoint.pose.getX());
    double vy = feedforward.vy + yController.calculate(measuredPose.getY(), setpoint.pose.getY());
    double omega =
        feedforward.omega
            + headingController.calculate(
                measuredPose.getRotation().getRadians(), setpoint.pose.getRotation().getRadians());

    drivetrain.setControl(driveRequest.withVelocity(new ChassisVelocities(vx, vy, omega)));
  }

  /** Done when the profile's total time has elapsed. */
  @Override
  protected boolean isFinished() {
    return path.isFinished(Utils.getCurrentTimeSeconds() - startTime);
  }

  /** Stops the drivetrain. Runs on both finish and interruption. */
  @Override
  protected void end(boolean interrupted) {
    drivetrain.setControl(new SwerveRequest.Idle());
  }
}
