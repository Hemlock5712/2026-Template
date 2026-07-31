// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.flywheel;

import static org.wpilib.units.Units.RotationsPerSecond;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicVelocityVoltage;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import frc.robot.generated.TunerConstants;
import frc.robot.hardware.LoggedTalonFX;
import frc.robot.utils.RunMode;
import org.littletonrobotics.junction.AutoLogOutput;
import org.wpilib.command3.Command;
import org.wpilib.command3.Mechanism;
import org.wpilib.command3.Scheduler;
import org.wpilib.math.system.DCMotor;
import org.wpilib.math.system.Models;
import org.wpilib.simulation.FlywheelSim;
import org.wpilib.system.RobotController;
import org.wpilib.units.measure.AngularVelocity;

/**
 * Flywheel - second example subsystem. Same pattern as {@link frc.robot.subsystems.arm.Arm}: owns
 * its motor, hides setters, exposes commands.
 */
public class Flywheel extends Mechanism {
  // Shooting speed (rotations per second).
  private static final double SHOOTING_SPEED_RPS = 25.0;

  private final LoggedTalonFX motor = new LoggedTalonFX(21, TunerConstants.kCANBus, "Flywheel");

  private final MotionMagicVelocityVoltage velocityOut = new MotionMagicVelocityVoltage(0);
  // How close the measured speed needs to be to count as "at target".
  private final AngularVelocity tolerance = RotationsPerSecond.of(0.25);

  public Flywheel() {
    TalonFXConfiguration config = new TalonFXConfiguration();
    config.MotorOutput.NeutralMode = NeutralModeValue.Coast;
    config.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;
    // Gains that work in sim. Re-tune on the real robot.
    config.Slot0.kS = 0.05; // the nudge to get moving
    config.Slot0.kV = 0.119; // volts per rotation-per-second; measured, see device-bringup
    config.Slot0.kP = 0.1; // push harder the bigger the speed miss

    // How fast the wheel may spin (rps) and speed up (rps²).
    config.MotionMagic.MotionMagicCruiseVelocity = 100.0;
    config.MotionMagic.MotionMagicAcceleration = 1000.0;

    motor.configure(config);

    // When nothing else is using the flywheel, keep it stopped. A TalonFX obeys its last command
    // forever, so without this, letting go of the shoot button leaves the wheel spinning.
    setDefaultCommand(stop());

    // Not isSimulation(): that is also true during replay, where the log supplies the sensor
    // values and re-running the physics would fight it.
    if (RunMode.current() == RunMode.SIM) {
      Scheduler.getDefault().addPeriodic(this::updateSimulation);
    }
  }

  // Holds never finish - never make a sequence wait on one. Need a finish line? Add it at the
  // call site: flywheel.spinUp().until(flywheel::isAtTarget). (Full rule in Arm.java.)

  /** Command the flywheel to shooting speed and hold it there. Never finishes. */
  public Command spinUp() {
    return runRepeatedly(() -> setVelocity(SHOOTING_SPEED_RPS)).named("spinUp (hold)");
  }

  /** Hold the flywheel at a stop. Never finishes. */
  public Command stop() {
    return runRepeatedly(() -> setVelocity(0.0)).named("stop (hold)");
  }

  /** True when the flywheel is within tolerance of its commanded speed. */
  @AutoLogOutput(key = "Flywheel/AtTarget")
  public boolean isAtTarget() {
    return Math.abs(motor.getClosedLoopError()) <= tolerance.in(RotationsPerSecond);
  }

  /** How fast the wheel is actually spinning, in rotations per second. */
  @AutoLogOutput(key = "Flywheel/SpeedRps")
  public double getSpeedRps() {
    return motor.getVelocityRps();
  }

  private void setVelocity(double rps) {
    motor.setControl(velocityOut.withVelocity(RotationsPerSecond.of(rps)));
  }

  // ---------------------------------------------------------------------------
  // Simulation only - see the same section in Arm.java.
  // ---------------------------------------------------------------------------

  private static final DCMotor GEARBOX = DCMotor.getKrakenX60(1);

  private final FlywheelSim wheelSim =
      // 0.001 = how hard the wheel is to spin up (kg·m²); bigger is slower to reach speed.
      new FlywheelSim(Models.flywheelFromPhysicalConstants(GEARBOX, 0.001, 1.0), GEARBOX);

  private void updateSimulation() {
    var motorSim = motor.device().getSimState();
    motorSim.setSupplyVoltage(RobotController.getBatteryVoltage());

    wheelSim.setInputVoltage(motorSim.getMotorVoltage());
    wheelSim.update(0.020); // one 20 ms robot loop

    // Report the pretend wheel speed back to the motor, in rotations per second.
    motorSim.setRotorVelocity(wheelSim.getAngularVelocity() / (2 * Math.PI));
  }
}
