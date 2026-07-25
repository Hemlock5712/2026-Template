// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.arm;

import static org.wpilib.units.Units.Degrees;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.GravityTypeValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import frc.robot.generated.TunerConstants;
import frc.robot.utils.TalonFXUtil;
import org.wpilib.command3.Command;
import org.wpilib.command3.Mechanism;
import org.wpilib.units.measure.Angle;

/**
 * Arm - an example subsystem on a TalonFX + CANcoder. The pattern: own the hardware, keep setters
 * private, expose commands - that's how the scheduler stops two things from fighting over the
 * motor.
 */
public class Arm extends Mechanism {
  // Position setpoints (rotations, 1.0 = full turn).
  private static final double VERTICAL_POSITION = 0.25; // 90°  - stowed / safe transport
  private static final double HORIZONTAL_POSITION = 0.5; // 180° - ground intake
  private static final double SCORING_POSITION = 0.083; // ~30° - scoring

  // How close counts as "at target".
  private static final double POSITION_TOLERANCE_DEGREES = 1.0;

  // PID + feedforward gains.
  // TODO: CRITICAL - tune on the real robot before driving the arm under power.
  // Safe starting values: kG=0.2, kS=0.2, kP=160, kD=30. Too jerky or fast? Make them smaller.
  private static final double kG = 0.0; // NEEDS TUNING - gravity feedforward
  private static final double kS = 0.0; // NEEDS TUNING - static friction feedforward
  private static final double kP = 0.0; // NEEDS TUNING - proportional gain
  private static final double kD = 0.0; // NEEDS TUNING - derivative gain

  // Motion Magic speed limits.
  // TODO: CRITICAL - set how fast the arm can move. Start: cruise=2 rot/s, accel=4 rot/s².
  private static final double MOTION_MAGIC_CRUISE_VELOCITY = 0.0; // NEEDS SETTING
  private static final double MOTION_MAGIC_ACCELERATION = 0.0; // NEEDS SETTING

  private final TalonFX motor = new TalonFX(31, TunerConstants.kCANBus);
  private final CANcoder encoder = new CANcoder(32, TunerConstants.kCANBus);

  // Drives the arm to a target angle with a smooth Motion Magic profile.
  private final MotionMagicVoltage positionOut = new MotionMagicVoltage(0);

  private final Angle tolerance = Degrees.of(POSITION_TOLERANCE_DEGREES);

  public Arm() {
    TalonFXConfiguration config = new TalonFXConfiguration();
    config.MotorOutput.NeutralMode = NeutralModeValue.Coast;
    config.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;
    config.Slot0.GravityType = GravityTypeValue.Arm_Cosine; // fights gravity automatically

    config.Slot0.kG = kG;
    config.Slot0.kS = kS;
    config.Slot0.kP = kP;
    config.Slot0.kD = kD;

    config.MotionMagic.MotionMagicCruiseVelocity = MOTION_MAGIC_CRUISE_VELOCITY;
    config.MotionMagic.MotionMagicAcceleration = MOTION_MAGIC_ACCELERATION;
    config.Feedback.withRemoteCANcoder(encoder);

    TalonFXUtil.applyConfigWithRetries(motor, config);
  }

  // THE ONE RULE: a hold never finishes, so never WAIT on a hold - it sticks in a sequence
  // forever. Need a finish line? Add it at the call site:
  //
  //   arm.scoring().until(arm::isAtTarget)   // finishes when the arm arrives
  //
  // The "(hold)" in each name shows up on the dashboard and in logs - a stuck sequence sitting
  // on a "(hold)" is the bug.

  /** Move to the vertical (stowed) position and hold it. Never finishes - see the rule above. */
  public Command vertical() {
    return runRepeatedly(() -> setPosition(VERTICAL_POSITION)).named("vertical (hold)");
  }

  /** Move to the horizontal (ground intake) position and hold it. Never finishes. */
  public Command horizontal() {
    return runRepeatedly(() -> setPosition(HORIZONTAL_POSITION)).named("horizontal (hold)");
  }

  /** Move to the scoring position and hold it. Never finishes. */
  public Command scoring() {
    return runRepeatedly(() -> setPosition(SCORING_POSITION)).named("scoring (hold)");
  }

  /** True when the arm has reached its target angle. */
  public boolean isAtTarget() {
    return getPosition().isNear(getTargetPosition(), tolerance);
  }

  /** Current measured arm angle. */
  public Angle getPosition() {
    return encoder.getPosition().getValue();
  }

  /** Angle the arm is currently driving toward. */
  public Angle getTargetPosition() {
    return positionOut.getPositionMeasure();
  }

  private void setPosition(double rotations) {
    motor.setControl(positionOut.withPosition(rotations));
  }
}
