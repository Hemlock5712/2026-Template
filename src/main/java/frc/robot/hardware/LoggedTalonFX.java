// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.hardware;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.ControlRequest;
import com.ctre.phoenix6.hardware.TalonFX;
import frc.robot.utils.RunMode;
import frc.robot.utils.TalonFXUtil;
import org.littletonrobotics.junction.AutoLog;
import org.littletonrobotics.junction.Logger;

/**
 * A TalonFX whose sensor readings go through the log, so they can be replayed. Use it exactly like
 * a TalonFX; the getters return plain numbers instead of {@code StatusSignal}s.
 *
 * <p>Reading a real motor gives one answer live and a different one during replay, so every read
 * has to come from {@link #refreshAll}: on a robot it fills {@link TalonFXInputs} from CAN and
 * writes it to the log, and during replay it fills the same fields from the log instead. Everything
 * downstream cannot tell the difference, which is the whole point - the same code runs both times.
 *
 * <p>Anything read straight off {@link #device()} skips the log and will NOT replay.
 */
public class LoggedTalonFX implements LoggedHardware.Device {
  /** One motor's readings for a single loop. @AutoLog generates the log/replay plumbing. */
  @AutoLog
  public static class TalonFXInputs {
    public boolean connected;
    public double positionRot;
    public double rotorPositionRot;
    public double velocityRps;
    public double motorVoltage;
    public double supplyCurrentAmps;
    public double statorCurrentAmps;
    public double torqueCurrentAmps;
    public double deviceTempCelsius;
    public double closedLoopReference;
    public double closedLoopError;
    public boolean motionMagicAtTarget;
  }

  private final TalonFX device;
  private final String logKey;
  private final TalonFXInputsAutoLogged inputs = new TalonFXInputsAutoLogged();

  private final StatusSignal<?> position;
  private final StatusSignal<?> rotorPosition;
  private final StatusSignal<?> velocity;
  private final StatusSignal<?> motorVoltage;
  private final StatusSignal<?> supplyCurrent;
  private final StatusSignal<?> statorCurrent;
  private final StatusSignal<?> torqueCurrent;
  private final StatusSignal<?> deviceTemp;
  private final StatusSignal<?> closedLoopReference;
  private final StatusSignal<?> closedLoopError;
  private final StatusSignal<?> motionMagicAtTarget;

  /**
   * @param name what this motor is called in the log, e.g. "Flywheel". Must be unique.
   */
  public LoggedTalonFX(int deviceId, CANBus bus, String name) {
    this.device = new TalonFX(deviceId, bus);
    this.logKey = "Hardware/TalonFX/" + name;

    position = device.getPosition();
    rotorPosition = device.getRotorPosition();
    velocity = device.getVelocity();
    motorVoltage = device.getMotorVoltage();
    supplyCurrent = device.getSupplyCurrent();
    statorCurrent = device.getStatorCurrent();
    torqueCurrent = device.getTorqueCurrent();
    deviceTemp = device.getDeviceTemp();
    closedLoopReference = device.getClosedLoopReference();
    closedLoopError = device.getClosedLoopError();
    motionMagicAtTarget = device.getMotionMagicAtTarget();

    LoggedHardware.register(
        this,
        logKey,
        bus,
        position,
        rotorPosition,
        velocity,
        motorVoltage,
        supplyCurrent,
        statorCurrent,
        torqueCurrent,
        deviceTemp,
        closedLoopReference,
        closedLoopError,
        motionMagicAtTarget);
    BringUp.add(name, this);
  }

  /** Applies a config, retrying on CAN hiccups. Does nothing during replay. */
  public void configure(TalonFXConfiguration config) {
    if (RunMode.isReplay()) {
      return;
    }
    TalonFXUtil.applyConfigWithRetries(device, config);
  }

  /** Commands the motor. Logged either way; only reaches hardware outside replay. */
  public void setControl(ControlRequest request) {
    Logger.recordOutput(logKey + "/Request", request.getName());
    if (RunMode.isReplay()) {
      return;
    }
    device.setControl(request);
  }

  /** True if the motor answered on CAN this loop. */
  public boolean isConnected() {
    return inputs.connected;
  }

  /** Motor position in rotations, after the feedback config - so this is MECHANISM position. */
  public double getPositionRot() {
    return inputs.positionRot;
  }

  /**
   * Raw rotor turns, before any gearing or remote sensor. Divide by mechanism turns for a ratio.
   */
  public double getRotorPositionRot() {
    return inputs.rotorPositionRot;
  }

  /** Motor speed in rotations per second. */
  public double getVelocityRps() {
    return inputs.velocityRps;
  }

  /** Volts the motor is applying right now. */
  public double getMotorVoltage() {
    return inputs.motorVoltage;
  }

  /** Current drawn from the battery, in amps. */
  public double getSupplyCurrentAmps() {
    return inputs.supplyCurrentAmps;
  }

  /** Current through the motor windings, in amps. Higher than supply current when geared down. */
  public double getStatorCurrentAmps() {
    return inputs.statorCurrentAmps;
  }

  /** Torque-producing current, in amps. Roughly proportional to the force the motor is making. */
  public double getTorqueCurrentAmps() {
    return inputs.torqueCurrentAmps;
  }

  /** Motor temperature in Celsius. */
  public double getDeviceTempCelsius() {
    return inputs.deviceTempCelsius;
  }

  /** What the motor's closed loop is aiming at, in its control mode's units. */
  public double getClosedLoopReference() {
    return inputs.closedLoopReference;
  }

  /** How far the closed loop is missing by, in its control mode's units. */
  public double getClosedLoopError() {
    return inputs.closedLoopError;
  }

  /** True when Motion Magic says the planned move has finished. False before anything runs. */
  public boolean getMotionMagicAtTarget() {
    return inputs.motionMagicAtTarget;
  }

  /** The raw TalonFX, for simulation and anything this class does not wrap. Does NOT replay. */
  public TalonFX device() {
    return device;
  }

  @Override
  public void updateInputs() {
    inputs.connected =
        BaseStatusSignal.isAllGood(
            position, velocity, motorVoltage, closedLoopReference, closedLoopError);
    inputs.positionRot = position.getValueAsDouble();
    inputs.rotorPositionRot = rotorPosition.getValueAsDouble();
    inputs.velocityRps = velocity.getValueAsDouble();
    inputs.motorVoltage = motorVoltage.getValueAsDouble();
    inputs.supplyCurrentAmps = supplyCurrent.getValueAsDouble();
    inputs.statorCurrentAmps = statorCurrent.getValueAsDouble();
    inputs.torqueCurrentAmps = torqueCurrent.getValueAsDouble();
    inputs.deviceTempCelsius = deviceTemp.getValueAsDouble();
    inputs.closedLoopReference = closedLoopReference.getValueAsDouble();
    inputs.closedLoopError = closedLoopError.getValueAsDouble();
    inputs.motionMagicAtTarget = motionMagicAtTarget.getValue() == Boolean.TRUE;
  }

  @Override
  public void logInputs() {
    Logger.processInputs(logKey, inputs);
  }
}
