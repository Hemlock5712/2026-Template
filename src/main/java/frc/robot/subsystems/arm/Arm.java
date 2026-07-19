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
 * Arm - an example subsystem driven by a Phoenix 6 TalonFX + CANcoder.
 *
 * <p>Pattern to teach: the subsystem owns the hardware, keeps its setters {@code private}, and
 * exposes <b>commands</b> (each returns a {@link Command}). Anything that wants to move the arm
 * does it through a command, which is how the scheduler prevents two things fighting over the
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
  // Safe starting values: kG=0.2 (fights gravity), kS=0.2 (overcomes friction),
  //                       kP=160 (correction strength), kD=30 (smoothness).
  // If the arm jerks or moves too fast, make these smaller.
  private static final double kG = 0.0; // NEEDS TUNING - gravity feedforward
  private static final double kS = 0.0; // NEEDS TUNING - static friction feedforward
  private static final double kP = 0.0; // NEEDS TUNING - proportional gain
  private static final double kD = 0.0; // NEEDS TUNING - derivative gain

  // Motion Magic speed limits.
  // TODO: CRITICAL - set how fast the arm can move.
  // Recommended start: cruise=2 rot/s, accel=4 rot/s².
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

  // The "move and hold" factories use runRepeatedly, which re-sends the Motion Magic request every
  // loop. Phoenix already holds the last request; re-sending just re-asserts it after a reboot.
  //
  // THE ONE RULE: a hold never finishes, so nothing may ever WAIT on a hold. A hold inside
  // Command.sequence (or awaited in a coroutine) sticks there forever. When one step needs to
  // finish, give it a finish line AT THE CALL SITE instead of adding a second method here:
  //
  //   arm.scoring().until(arm::isAtTarget)   // finishes when the arm arrives
  //
  // The "(hold)" in each command name shows up on the dashboard and in logs - if a stuck
  // sequence is sitting on a "(hold)", that's the bug.

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
