// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.utils;

import com.ctre.phoenix6.swerve.SwerveDrivetrain.SwerveDriveState;
import org.littletonrobotics.junction.Logger;

/**
 * Logs the swerve drivetrain state with AdvantageKit under {@code Drivetrain/*}. Called once per
 * loop from {@link frc.robot.subsystems.DriveMechanism}. See the log-reading skill.
 */
public class Telemetry {
  /** Logs one drivetrain state. */
  public void telemeterize(SwerveDriveState state) {
    Logger.recordOutput("Drivetrain/Pose", state.Pose);
    Logger.recordOutput("Drivetrain/Velocity", state.Velocity);
    Logger.recordOutput("Drivetrain/RawHeading", state.RawHeading);

    // AdvantageScope renders these natively on the swerve widget.
    Logger.recordOutput("Drivetrain/ModuleStates", state.ModuleVelocities);
    Logger.recordOutput("Drivetrain/ModuleTargets", state.ModuleTargets);
    Logger.recordOutput("Drivetrain/ModulePositions", state.ModulePositions);

    Logger.recordOutput(
        "Drivetrain/TranslationSpeedMps", Math.hypot(state.Velocity.vx, state.Velocity.vy));
    Logger.recordOutput("Drivetrain/RotationSpeedRadPerSec", state.Velocity.omega);
    Logger.recordOutput("Drivetrain/OdometryPeriodSeconds", state.OdometryPeriod);
    Logger.recordOutput(
        "Drivetrain/OdometryFrequencyHz",
        state.OdometryPeriod > 0 ? 1.0 / state.OdometryPeriod : 0.0);
  }
}
