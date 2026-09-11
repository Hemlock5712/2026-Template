// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.flywheel;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicVelocityVoltage;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import frc.robot.Robot;
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

/**
 * Flywheel - second example subsystem, on a TalonFX alone. Same shape as {@link
 * frc.robot.subsystems.arm.Arm}, but velocity instead of position.
 */
public class Flywheel extends Mechanism {
  private final LoggedTalonFX motor = new LoggedTalonFX(21, TunerConstants.kCANBus, "Flywheel");
  private final MotionMagicVelocityVoltage velocityOut = new MotionMagicVelocityVoltage(0);

  public Flywheel() {
    TalonFXConfiguration config = new TalonFXConfiguration();
    config.MotorOutput.NeutralMode = NeutralModeValue.Coast;
    config.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;
    // Gains that work in sim. Re-tune on the real robot - see device-bringup.
    config.Slot0.kS = 0.05;
    config.Slot0.kV = 0.119; // volts per rot/s
    config.Slot0.kP = 0.1;
    config.MotionMagic.MotionMagicCruiseVelocity = 100.0; // rot/s
    config.MotionMagic.MotionMagicAcceleration = 1000.0; // rot/s^2
    motor.configure(config);

    // A TalonFX obeys its last command forever - without this, letting go of the shoot button
    // leaves the wheel spinning.
    setDefaultCommand(stop());

    // Not isSimulation(): that is also true in replay, where the log supplies sensor values.
    if (RunMode.current() == RunMode.SIM) {
      Scheduler.getDefault().addPeriodic(this::updateSimulation);
    }
  }

  // Holds never finish - never WAIT on one. Finish line goes at the call site:
  //   flywheel.spinUp().until(flywheel::atSpeed)
  // "(hold)" in the name shows on the dashboard - a stuck sequence sitting on a "(hold)" is the
  // bug.

  /** 25 rot/s, shooting. */
  public Command spinUp() {
    return runRepeatedly(() -> motor.setControl(velocityOut.withVelocity(25.0)))
        .named("spinUp (hold)");
  }

  /** 0 rot/s, stopped. */
  public Command stop() {
    return runRepeatedly(() -> motor.stopMotor()).named("stop (hold)");
  }

  /** True once the wheel is within 0.25 rot/s of the last speed a hold asked for. */
  @AutoLogOutput(key = "Flywheel/AtSpeed")
  public boolean atSpeed() {
    return Math.abs(getSpeedRps() - velocityOut.Velocity) <= 0.25;
  }

  @AutoLogOutput(key = "Flywheel/SpeedRps")
  public double getSpeedRps() {
    return motor.getVelocityRps();
  }

  /** Motion Magic's target THIS instant - it ramps up to meet the speed. */
  @AutoLogOutput(key = "Flywheel/TargetRps")
  public double getTargetRps() {
    return motor.getClosedLoopReference();
  }

  // ---------------------------------------------------------------------------
  // Simulation only: a physics model pretends to be the wheel.
  // ---------------------------------------------------------------------------

  private final FlywheelSim wheelSim =
      // 0.001 kg·m² = how hard the wheel is to spin up; bigger is slower to reach speed.
      new FlywheelSim(
          Models.flywheelFromPhysicalConstants(DCMotor.getKrakenX60(1), 0.001, 1.0),
          DCMotor.getKrakenX60(1));

  private void updateSimulation() {
    var motorSim = motor.device().getSimState();
    motorSim.setSupplyVoltage(RobotController.getBatteryVoltage());

    wheelSim.setInputVoltage(motorSim.getMotorVoltage());
    wheelSim.update(Robot.PERIOD_SECONDS);

    // Advance the rotor too, or RotorPositionRot logs a wheel that never turns.
    double rotationsPerSecond = wheelSim.getAngularVelocity() / (2 * Math.PI);
    motorSim.setRotorVelocity(rotationsPerSecond);
    motorSim.addRotorPosition(rotationsPerSecond * Robot.PERIOD_SECONDS);
  }
}
