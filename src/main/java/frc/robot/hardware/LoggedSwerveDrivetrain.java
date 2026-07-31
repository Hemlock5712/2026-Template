// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.hardware;

import com.ctre.phoenix6.BaseStatusSignal;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import org.littletonrobotics.junction.AutoLog;
import org.littletonrobotics.junction.Logger;
import org.wpilib.math.geometry.Pose2d;
import org.wpilib.math.geometry.Rotation2d;
import org.wpilib.math.kinematics.ChassisVelocities;
import org.wpilib.math.kinematics.SwerveModulePosition;
import org.wpilib.math.kinematics.SwerveModuleVelocity;

/**
 * The swerve drivetrain as a single logged device: its whole state goes through the log, so
 * everything downstream - autos, drive-to-pose, alignment - replays.
 *
 * <p>The twelve swerve devices are not wrapped individually. CTRE runs them on its own 250 Hz
 * odometry thread inside the drivetrain, which this code cannot get between. So the odometry
 * *answer* is the input, not the sensors behind it. What that costs: replay cannot re-derive the
 * pose, so changing the pose estimator or a vision trust number will not move the logged pose.
 * Everything that consumes the pose still replays exactly.
 */
public class LoggedSwerveDrivetrain implements LoggedHardware.Device {
  /** The drivetrain's state for a single loop. */
  @AutoLog
  public static class SwerveInputs {
    public Pose2d pose = new Pose2d();
    public ChassisVelocities velocity = new ChassisVelocities();
    public Rotation2d rawHeading = new Rotation2d();
    public SwerveModuleVelocity[] moduleVelocities = new SwerveModuleVelocity[0];
    public SwerveModuleVelocity[] moduleTargets = new SwerveModuleVelocity[0];
    public SwerveModulePosition[] modulePositions = new SwerveModulePosition[0];
    public double odometryPeriodSeconds;
  }

  private final CommandSwerveDrivetrain drivetrain;
  private final SwerveInputsAutoLogged inputs = new SwerveInputsAutoLogged();

  public LoggedSwerveDrivetrain(CommandSwerveDrivetrain drivetrain) {
    this.drivetrain = drivetrain;
    // No signals to batch: CTRE reads the modules itself on its odometry thread.
    LoggedHardware.register(this, "Drivetrain");
  }

  /** Field pose from odometry, blue-origin (the origin never flips with alliance). */
  public Pose2d getPose() {
    return inputs.pose;
  }

  /** Velocity in the robot frame. */
  public ChassisVelocities getVelocity() {
    return inputs.velocity;
  }

  /** Velocity rotated into the field frame. */
  public ChassisVelocities getFieldVelocity() {
    return inputs.velocity.toFieldRelative(inputs.pose.getRotation());
  }

  @Override
  public BaseStatusSignal[] signals() {
    return new BaseStatusSignal[0];
  }

  @Override
  public void updateInputs() {
    var state = drivetrain.getState();
    inputs.pose = state.Pose;
    inputs.velocity = state.Velocity;
    inputs.rawHeading = state.RawHeading;
    inputs.moduleVelocities = state.ModuleVelocities;
    inputs.moduleTargets = state.ModuleTargets;
    inputs.modulePositions = state.ModulePositions;
    inputs.odometryPeriodSeconds = state.OdometryPeriod;
  }

  @Override
  public void logInputs() {
    Logger.processInputs("Drivetrain", inputs);

    // Derived from the inputs above, so these are outputs - they recompute during replay, and
    // changing the maths here changes what a replay reports.
    Logger.recordOutput(
        "Drivetrain/TranslationSpeedMps", Math.hypot(inputs.velocity.vx, inputs.velocity.vy));
    Logger.recordOutput("Drivetrain/RotationSpeedRadPerSec", inputs.velocity.omega);
    Logger.recordOutput(
        "Drivetrain/OdometryFrequencyHz",
        inputs.odometryPeriodSeconds > 0 ? 1.0 / inputs.odometryPeriodSeconds : 0.0);
  }
}
