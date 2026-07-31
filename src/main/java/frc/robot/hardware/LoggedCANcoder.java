// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.hardware;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.hardware.CANcoder;
import org.littletonrobotics.junction.AutoLog;
import org.littletonrobotics.junction.Logger;

/**
 * A CANcoder whose readings go through the log, so they can be replayed. See {@link LoggedTalonFX}
 * for how this works; anything read straight off {@link #device()} will NOT replay.
 */
public class LoggedCANcoder implements LoggedHardware.Device {
  /** One encoder's readings for a single loop. */
  @AutoLog
  public static class CANcoderInputs {
    public boolean connected;
    public double positionRot;
    public double absolutePositionRot;
    public double velocityRps;
  }

  private final CANcoder device;
  private final String logKey;
  private final CANcoderInputsAutoLogged inputs = new CANcoderInputsAutoLogged();

  private final StatusSignal<?> position;
  private final StatusSignal<?> absolutePosition;
  private final StatusSignal<?> velocity;

  /**
   * @param name what this encoder is called in the log, e.g. "Arm". Must be unique.
   */
  public LoggedCANcoder(int deviceId, CANBus bus, String name) {
    device = new CANcoder(deviceId, bus);
    logKey = "Hardware/CANcoder/" + name;

    position = device.getPosition();
    absolutePosition = device.getAbsolutePosition();
    velocity = device.getVelocity();

    LoggedHardware.register(this, logKey, bus);
    BringUp.add(name, this);
  }

  /** True if the encoder answered on CAN this loop. */
  public boolean isConnected() {
    return inputs.connected;
  }

  /** Accumulated position in rotations - keeps counting past one full turn. */
  public double getPositionRot() {
    return inputs.positionRot;
  }

  /** Where the magnet points, in rotations. Always within one turn, and survives a reboot. */
  public double getAbsolutePositionRot() {
    return inputs.absolutePositionRot;
  }

  /** Rotations per second. */
  public double getVelocityRps() {
    return inputs.velocityRps;
  }

  /**
   * The raw CANcoder, for configuration, simulation, and fusing into a TalonFX. Does NOT replay.
   */
  public CANcoder device() {
    return device;
  }

  @Override
  public BaseStatusSignal[] signals() {
    return new BaseStatusSignal[] {position, absolutePosition, velocity};
  }

  @Override
  public void updateInputs() {
    inputs.connected = BaseStatusSignal.isAllGood(position, absolutePosition, velocity);
    inputs.positionRot = position.getValueAsDouble();
    inputs.absolutePositionRot = absolutePosition.getValueAsDouble();
    inputs.velocityRps = velocity.getValueAsDouble();
  }

  @Override
  public void logInputs() {
    Logger.processInputs(logKey, inputs);
  }
}
