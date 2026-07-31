// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.hardware;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.CANrangeConfiguration;
import com.ctre.phoenix6.hardware.CANrange;
import frc.robot.utils.RunMode;
import org.littletonrobotics.junction.AutoLog;
import org.littletonrobotics.junction.Logger;

/**
 * A CANrange distance sensor whose readings go through the log, so they can be replayed. See {@link
 * LoggedTalonFX} for how this works; anything read straight off {@link #device()} will NOT replay.
 *
 * <p>This is the sensor you replay for "did we actually have a game piece" questions - record a
 * match, then change the detection rule and re-run it against the same readings.
 */
public class LoggedCANrange implements LoggedHardware.Device {
  /** One sensor's readings for a single loop. */
  @AutoLog
  public static class CANrangeInputs {
    public boolean connected;
    public boolean detected;
    public double distanceMeters;
    public double distanceStdDevMeters;
    public double signalStrength;
  }

  private final CANrange device;
  private final String logKey;
  private final CANrangeInputsAutoLogged inputs = new CANrangeInputsAutoLogged();

  private final StatusSignal<?> distance;
  private final StatusSignal<?> distanceStdDev;
  private final StatusSignal<?> detected;
  private final StatusSignal<?> signalStrength;

  /**
   * @param name what this sensor is called in the log, e.g. "Intake". Must be unique.
   */
  public LoggedCANrange(int deviceId, CANBus bus, String name) {
    device = new CANrange(deviceId, bus);
    logKey = "Hardware/CANrange/" + name;

    distance = device.getDistance();
    distanceStdDev = device.getDistanceStdDev();
    detected = device.getIsDetected();
    signalStrength = device.getSignalStrength();

    LoggedHardware.register(this, logKey);
  }

  /** Applies a config, retrying on CAN hiccups. Does nothing during replay. */
  public void configure(CANrangeConfiguration config) {
    if (RunMode.current() == RunMode.REPLAY) {
      return;
    }
    device.getConfigurator().apply(config);
  }

  /** True if the sensor answered on CAN this loop. */
  public boolean isConnected() {
    return inputs.connected;
  }

  /** True when the sensor sees something inside its configured threshold. */
  public boolean isDetected() {
    return inputs.detected;
  }

  /** Distance to whatever it sees, in meters. Meaningless when {@link #isDetected()} is false. */
  public double getDistanceMeters() {
    return inputs.distanceMeters;
  }

  /** How unsure the sensor is of that distance, in meters. Grows with range and stray light. */
  public double getDistanceStdDevMeters() {
    return inputs.distanceStdDevMeters;
  }

  /** How strong the return is. A weak signal means the distance is not worth trusting. */
  public double getSignalStrength() {
    return inputs.signalStrength;
  }

  /** The raw CANrange, for simulation and anything this class does not wrap. Does NOT replay. */
  public CANrange device() {
    return device;
  }

  @Override
  public BaseStatusSignal[] signals() {
    return new BaseStatusSignal[] {distance, distanceStdDev, detected, signalStrength};
  }

  @Override
  public void updateInputs() {
    inputs.connected = BaseStatusSignal.isAllGood(distance, detected, signalStrength);
    inputs.detected = detected.getValue() == Boolean.TRUE;
    inputs.distanceMeters = distance.getValueAsDouble();
    inputs.distanceStdDevMeters = distanceStdDev.getValueAsDouble();
    inputs.signalStrength = signalStrength.getValueAsDouble();
  }

  @Override
  public void logInputs() {
    Logger.processInputs(logKey, inputs);
  }
}
