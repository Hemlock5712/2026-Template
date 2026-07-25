// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.arm;

import static org.wpilib.units.Units.Degrees;
import static org.wpilib.units.Units.Rotations;

import com.ctre.phoenix6.Utils;
import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.GravityTypeValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import frc.robot.generated.TunerConstants;
import frc.robot.utils.TalonFXUtil;
import org.littletonrobotics.junction.AutoLogOutput;
import org.wpilib.command3.Command;
import org.wpilib.command3.Mechanism;
import org.wpilib.command3.Scheduler;
import org.wpilib.math.system.DCMotor;
import org.wpilib.simulation.SingleJointedArmSim;
import org.wpilib.system.RobotController;
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

  private final TalonFX motor = new TalonFX(31, TunerConstants.kCANBus);
  private final CANcoder encoder = new CANcoder(32, TunerConstants.kCANBus);

  // Drives the arm to a target angle with a smooth Motion Magic profile.
  private final MotionMagicVoltage positionOut = new MotionMagicVoltage(0);

  // How close counts as "at target".
  private static final Angle TOLERANCE = Degrees.of(1.0);

  public Arm() {
    // Report angles as 0 to 1 rotations, not -0.5 to +0.5. The default range puts its seam right
    // on HORIZONTAL_POSITION (0.5), where the arm could read -0.5 and drive a full turn the
    // wrong way.
    //
    // refresh() first: apply() writes EVERY field, so building a fresh config here would zero the
    // MagnetOffset you set in Tuner X. Read the device's settings, change the one we care about,
    // write it back.
    CANcoderConfiguration encoderConfig = new CANcoderConfiguration();
    encoder.getConfigurator().refresh(encoderConfig);
    encoderConfig.MagnetSensor.AbsoluteSensorDiscontinuityPoint = 1.0;
    encoder.getConfigurator().apply(encoderConfig);

    TalonFXConfiguration config = new TalonFXConfiguration();
    // Brake, not coast - a coasting arm falls to its hard stop whenever the robot is disabled.
    config.MotorOutput.NeutralMode = NeutralModeValue.Brake;
    config.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;
    config.Slot0.GravityType = GravityTypeValue.Arm_Cosine; // fights gravity automatically

    // Gains that work in sim. Re-tune on the real robot.
    config.Slot0.kG = 0.2; // holds the arm up against gravity
    config.Slot0.kS = 0.2; // the nudge to get moving
    config.Slot0.kP = 160.0; // push harder the bigger the miss
    config.Slot0.kD = 2.0; // damping. Keep it small - too big and the arm shakes.

    // How fast the arm may move (rot/s) and speed up (rot/s²).
    config.MotionMagic.MotionMagicCruiseVelocity = 2.0;
    config.MotionMagic.MotionMagicAcceleration = 4.0;

    config.Feedback.withRemoteCANcoder(encoder);

    TalonFXUtil.applyConfigWithRetries(motor, config);

    if (Utils.isSimulation()) {
      Scheduler.getDefault().addPeriodic(this::updateSimulation);
    }
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

  /**
   * True when the arm has reached its target angle. Both halves matter: the motor's own
   * "MotionMagicAtTarget" says the planned move has finished (and is false before anything commands
   * the arm at all, so this can't report success at startup), and the closed-loop error says the
   * arm really is there and not just tracking a plan that ended.
   */
  @AutoLogOutput(key = "Arm/AtTarget")
  public boolean isAtTarget() {
    return motor.getMotionMagicAtTarget().getValue()
        && Math.abs(motor.getClosedLoopError().getValueAsDouble()) <= TOLERANCE.in(Rotations);
  }

  /** Where the arm is right now, in degrees. Logged so you can graph it in AdvantageScope. */
  @AutoLogOutput(key = "Arm/AngleDegrees")
  public double getAngleDegrees() {
    return getPosition().in(Degrees);
  }

  /**
   * The angle Motion Magic is aiming at <i>this instant</i>, in degrees. Motion Magic ramps toward
   * the final target, so this slides up to meet it - graph it against Arm/AngleDegrees to see how
   * well the arm is keeping up.
   */
  @AutoLogOutput(key = "Arm/TargetDegrees")
  public double getTargetDegrees() {
    return getTargetPosition().in(Degrees);
  }

  /** Current measured arm angle. */
  public Angle getPosition() {
    return encoder.getPosition().getValue();
  }

  /** The angle the motor's closed loop is currently driving toward. */
  public Angle getTargetPosition() {
    return Rotations.of(motor.getClosedLoopReference().getValueAsDouble());
  }

  private void setPosition(double rotations) {
    motor.setControl(positionOut.withPosition(rotations));
  }

  // ---------------------------------------------------------------------------
  // Simulation only: a physics model pretends to be the arm, so the same control
  // code works with no robot plugged in.
  // ---------------------------------------------------------------------------

  // 50 motor turns = 1 arm turn.
  private static final double GEAR_RATIO = 50.0;

  private static final double ARM_LENGTH_METERS = 0.5;

  // Angles are radians here, and 0 = straight out horizontally (matches Arm_Cosine above).
  private final SingleJointedArmSim armSim =
      new SingleJointedArmSim(
          DCMotor.getKrakenX60(1),
          GEAR_RATIO,
          SingleJointedArmSim.estimateMOI(ARM_LENGTH_METERS, 4.0), // 4 kg arm
          ARM_LENGTH_METERS,
          Math.toRadians(-10.0), // min - just past the low end
          Math.toRadians(190.0), // max - just past the high end
          true, // simulate gravity, so kG has something to fight
          0.0); // starts hanging straight out

  private void updateSimulation() {
    var motorSim = motor.getSimState();
    var encoderSim = encoder.getSimState();

    motorSim.setSupplyVoltage(RobotController.getBatteryVoltage());
    encoderSim.setSupplyVoltage(RobotController.getBatteryVoltage());

    // Motor voltage in -> physics -> new arm angle out.
    armSim.setInputVoltage(motorSim.getMotorVoltage());
    armSim.update(0.020); // one 20 ms robot loop

    // Report the pretend arm back to both sensors, in rotations.
    double armRotations = armSim.getAngle() / (2 * Math.PI);
    double armRotationsPerSec = armSim.getVelocity() / (2 * Math.PI);
    encoderSim.setRawPosition(armRotations);
    encoderSim.setVelocity(armRotationsPerSec);
    motorSim.setRawRotorPosition(armRotations * GEAR_RATIO);
    motorSim.setRotorVelocity(armRotationsPerSec * GEAR_RATIO);
  }
}
