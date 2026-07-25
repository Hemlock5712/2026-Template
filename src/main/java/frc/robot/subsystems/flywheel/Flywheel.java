// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.flywheel;

import static org.wpilib.units.Units.RotationsPerSecond;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicVelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import frc.robot.generated.TunerConstants;
import frc.robot.utils.TalonFXUtil;
import org.wpilib.command3.Command;
import org.wpilib.command3.Mechanism;
import org.wpilib.units.measure.AngularVelocity;

/**
 * Flywheel - second example subsystem. Same pattern as {@link frc.robot.subsystems.arm.Arm}: owns
 * its motor, hides setters, exposes commands.
 */
public class Flywheel extends Mechanism {
  // Shooting speed (rotations per second).
  private static final double SHOOTING_SPEED_RPS = 25.0;

  // How close the measured speed needs to be to count as "at target".
  private static final double VELOCITY_TOLERANCE_RPS = 0.25;

  // PID + feedforward gains.
  private static final double kS = 0.0; // static friction compensation
  private static final double kV = 0.125; // velocity feedforward (volts per rps)
  private static final double kP = 0.0; // proportional gain on velocity error

  // Motion Magic speed limits.
  private static final double MOTION_MAGIC_CRUISE_VELOCITY = 100.0; // max rps
  private static final double MOTION_MAGIC_ACCELERATION = 1000.0; // rps² ramp

  private final TalonFX motor = new TalonFX(21, TunerConstants.kCANBus);

  private final MotionMagicVelocityVoltage velocityOut = new MotionMagicVelocityVoltage(0);
  private final AngularVelocity tolerance = RotationsPerSecond.of(VELOCITY_TOLERANCE_RPS);

  public Flywheel() {
    TalonFXConfiguration config = new TalonFXConfiguration();
    config.MotorOutput.NeutralMode = NeutralModeValue.Coast;
    config.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;
    config.Slot0.kS = kS;
    config.Slot0.kV = kV;
    config.Slot0.kP = kP;
    config.MotionMagic.MotionMagicCruiseVelocity = MOTION_MAGIC_CRUISE_VELOCITY;
    config.MotionMagic.MotionMagicAcceleration = MOTION_MAGIC_ACCELERATION;

    TalonFXUtil.applyConfigWithRetries(motor, config);
  }

  // Holds never finish - never make a sequence wait on one. Need a finish line? Add it at the
  // call site: flywheel.spinUp().until(flywheel::isAtTarget). (Full rule in Arm.java.)

  /** Command the flywheel to shooting speed and hold it there. Never finishes. */
  public Command spinUp() {
    return runRepeatedly(() -> setVelocity(SHOOTING_SPEED_RPS)).named("spinUp (hold)");
  }

  /** Stop the flywheel and keep it stopped. Never finishes. */
  public Command stop() {
    return runRepeatedly(motor::stopMotor).named("stop (hold)");
  }

  /** True when the flywheel is within tolerance of its target speed. */
  public boolean isAtTarget() {
    return motor.getVelocity().getValue().isNear(velocityOut.getVelocityMeasure(), tolerance);
  }

  private void setVelocity(double rps) {
    motor.setControl(velocityOut.withVelocity(RotationsPerSecond.of(rps)));
  }
}
