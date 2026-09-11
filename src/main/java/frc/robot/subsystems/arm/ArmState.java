// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.arm;

import static org.wpilib.units.Units.Degrees;
import static org.wpilib.units.Units.Rotations;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.signals.GravityTypeValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import frc.robot.Robot;
import frc.robot.generated.TunerConstants;
import frc.robot.hardware.LoggedCANcoder;
import frc.robot.hardware.LoggedTalonFX;
import frc.robot.utils.RunMode;
import org.littletonrobotics.junction.AutoLogOutput;
import org.wpilib.command3.Command;
import org.wpilib.command3.Mechanism;
import org.wpilib.command3.Scheduler;
import org.wpilib.command3.StateMachine;
import org.wpilib.command3.StateMachine.State;
import org.wpilib.math.system.DCMotor;
import org.wpilib.simulation.SingleJointedArmSim;
import org.wpilib.system.RobotController;

/**
 * Arm as a state machine. The arm is always in exactly one state. The rest of the robot only ASKS
 * for a state by scheduling arm.vertical / arm.horizontal / arm.scoring; the transitions in the
 * constructor decide whether the arm goes. Nobody outside this class drives the motor.
 */
public class ArmState extends Mechanism {
  // 50 motor turns = 1 arm turn. Confirm on hardware with the "Bring-Up" utility OpMode.
  private static final double GEAR_RATIO = 50.0;

  // Magnet offset and 0..1 range are set on the CANcoder itself in Tuner X, not here.
  private final LoggedTalonFX motor = new LoggedTalonFX(31, TunerConstants.kCANBus, "Arm");
  private final LoggedCANcoder encoder = new LoggedCANcoder(32, TunerConstants.kCANBus, "Arm");
  private final MotionMagicVoltage positionOut = new MotionMagicVoltage(0);

  private final StateMachine machine = new StateMachine("Arm");
  private State requested; // last asked for
  private State current; // where the machine is now, set by onEnter

  // One line builds states and commands
  public final Command vertical = buildCommand(0.25, "vertical");
  public final Command horizontal = buildCommand(0.5, "horizontal");
  public final Command scoring = buildCommand(0.083, "scoring");

  public ArmState() {
    TalonFXConfiguration config = new TalonFXConfiguration();
    config.MotorOutput.NeutralMode = NeutralModeValue.Brake; // coast = arm falls when disabled
    config.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;
    config.Slot0.GravityType = GravityTypeValue.Arm_Cosine;
    // Gains that work in sim. Re-tune on the real robot - see device-bringup.
    config.Slot0.kG = 0.34;
    config.Slot0.kS = 0.2;
    config.Slot0.kP = 160.0;
    config.Slot0.kD = 2.0;
    config.MotionMagic.MotionMagicCruiseVelocity = 2.0; // rot/s
    config.MotionMagic.MotionMagicAcceleration = 4.0; // rot/s^2
    config.Feedback.withRemoteCANcoder(encoder.device());
    config.Feedback.RotorToSensorRatio = GEAR_RATIO;
    motor.configure(config);

    // Not isSimulation(): that is also true in replay, where the log supplies sensor values.
    if (RunMode.current() == RunMode.SIM) {
      Scheduler.getDefault().addPeriodic(this::updateSimulation);
    }

    // Any state may switch to any other: when a different state is requested, go there. To forbid
    // a move, replace this with one line per allowed move, e.g.
    //   verticalState.switchTo(scoringState).when(() -> requested == scoringState);
    machine.switchFromAny().to(() -> requested).when(() -> requested != current);

    // The machine IS the arm's default command, so it runs whenever the robot is enabled.
    // Scheduling any other arm command interrupts it; when that ends it restarts at vertical.
    setDefaultCommand(machine);
  }

  /** True once the arm is in the requested state and has stopped moving. */
  public boolean atPosition() {
    return current == requested
        && motor.getMotionMagicAtTarget()
        && Math.abs(getPositionRot() - positionOut.Position) <= Degrees.of(1.0).in(Rotations);
  }

  @AutoLogOutput(key = "Arm/AngleDegrees")
  public double getAngleDegrees() {
    return Rotations.of(getPositionRot()).in(Degrees);
  }

  /** Motion Magic's target THIS instant - it ramps up to meet the pose. */
  @AutoLogOutput(key = "Arm/TargetDegrees")
  public double getTargetDegrees() {
    return Rotations.of(motor.getClosedLoopReference()).in(Degrees);
  }

  private double getPositionRot() {
    return encoder.getPositionRot();
  }

  // Adds one state (a hold at one position - holds never finish, the machine cancels them on a
  // switch) and returns the command that asks for it.
  private Command buildCommand(double rotations, String name) {
    Command hold =
        runRepeatedly(() -> motor.setControl(positionOut.withPosition(rotations)))
            .named(name + " (hold)");
    State state = machine.addState(hold);
    state.onEnter(() -> current = state);
    if (requested == null) { // first pose listed = starting pose
      requested = state;
      machine.setInitialState(state);
    }
    return Command.noRequirements(c -> requested = state).named("request " + name);
  }

  // ---------------------------------------------------------------------------
  // Simulation only: a physics model pretends to be the arm.
  // ---------------------------------------------------------------------------

  private static final double ARM_LENGTH_METERS = 0.5;

  // Radians here, 0 = horizontal (matches Arm_Cosine above).
  private final SingleJointedArmSim armSim =
      new SingleJointedArmSim(
          DCMotor.getKrakenX60(1),
          GEAR_RATIO,
          SingleJointedArmSim.estimateMOI(ARM_LENGTH_METERS, 4.0), // 4 kg arm
          ARM_LENGTH_METERS,
          Math.toRadians(-10.0),
          Math.toRadians(190.0),
          true, // gravity on, so kG has something to fight
          0.0); // starts hanging straight out

  private void updateSimulation() {
    var motorSim = motor.device().getSimState();
    var encoderSim = encoder.device().getSimState();

    motorSim.setSupplyVoltage(RobotController.getBatteryVoltage());
    encoderSim.setSupplyVoltage(RobotController.getBatteryVoltage());

    armSim.setInputVoltage(motorSim.getMotorVoltage());
    armSim.update(Robot.PERIOD_SECONDS);

    double armRotations = armSim.getAngle() / (2 * Math.PI);
    double armRotationsPerSec = armSim.getVelocity() / (2 * Math.PI);
    encoderSim.setRawPosition(armRotations);
    encoderSim.setVelocity(armRotationsPerSec);
    motorSim.setRawRotorPosition(armRotations * GEAR_RATIO);
    motorSim.setRotorVelocity(armRotationsPerSec * GEAR_RATIO);
  }
}
