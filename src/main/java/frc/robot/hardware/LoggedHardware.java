// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.hardware;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.CANBus;
import frc.robot.utils.RunMode;
import java.util.ArrayList;
import java.util.LinkedHashMap;
import java.util.List;
import java.util.Map;

/**
 * Reads every logged device once per loop. {@link frc.robot.Robot} calls {@link #refreshAll} at the
 * top of the loop; individual mechanisms never call it.
 *
 * <p>On a robot this asks the CAN bus for every signal in a single round trip, then writes the
 * answers to the log. During replay it skips the bus and fills the same fields from the log, so
 * mechanism code cannot tell which run it is in.
 *
 * <p>NAMING: a wrapper's method is the vendor's method name plus a unit suffix where the unit is
 * ambiguous - {@code getPosition} becomes {@code getPositionRot}, {@code getDeviceTemp} becomes
 * {@code getDeviceTempCelsius}, {@code getMotorVoltage} stays as it is. Booleans take {@code is}
 * ({@code getIsDetected} becomes {@code isDetected}). The point is that every name is findable in
 * the vendor's docs. Anything invented here is a combination of vendor values and says so.
 */
public final class LoggedHardware {
  private LoggedHardware() {}

  /** One device that can be read and logged. Implemented by the Logged* wrappers. */
  interface Device {
    void updateInputs();

    void logInputs();
  }

  // Registration order = construction order, so replay reads devices in the same sequence.
  private static final Map<String, Device> ALL = new LinkedHashMap<>();

  // ONE BATCH PER CAN BUS. Phoenix's refreshAll takes the bus of the first signal and, if any
  // other signal is on a different one, marks them all InvalidNetwork and refreshes NOTHING -
  // silently freezing every sensor on the robot. Mixing buses in one call is not an option.
  private static final Map<String, List<BaseStatusSignal>> SIGNALS_BY_BUS = new LinkedHashMap<>();

  /** Registers a device with no CAN signals to batch, such as the drivetrain or a Limelight. */
  static void register(Device device, String logKey) {
    if (ALL.putIfAbsent(logKey, device) != null) {
      throw new IllegalArgumentException("Two logged devices named \"" + logKey + "\"");
    }
  }

  /** Registers a device and adds its signals to its bus's batch. */
  static void register(Device device, String logKey, CANBus bus, BaseStatusSignal... signals) {
    register(device, logKey);
    SIGNALS_BY_BUS
        .computeIfAbsent(bus.getName(), name -> new ArrayList<>())
        .addAll(List.of(signals));
  }

  // 250 Hz, matching CTRE's odometry rate on CAN FD. Devices default to 100 Hz, which is slower
  // than the 200 Hz loop, so every other read would just repeat the previous value. Phoenix
  // promotes unsupported rates to the next one up and caps at 1000 Hz. This costs CAN bandwidth -
  // check CANBus.getStatus().BusUtilization after changing it, and keep the swerve bus to itself
  // if it climbs.
  private static final double SIGNAL_FREQUENCY_HZ = 250.0;

  /**
   * Sets every device's CAN signal rate. Call once from {@link frc.robot.Robot}'s constructor - by
   * then every device has registered, and these are blocking calls that have no business in the
   * periodic path.
   *
   * <p>MUST come after the last device registers. One that registers later keeps Phoenix's 100 Hz
   * default and silently returns the same value on half the loops.
   */
  public static void initialize() {
    if (RunMode.isReplay()) {
      return; // no devices to configure; the log already holds what they said
    }
    for (List<BaseStatusSignal> busSignals : SIGNALS_BY_BUS.values()) {
      BaseStatusSignal.setUpdateFrequencyForAll(SIGNAL_FREQUENCY_HZ, busSignals);
    }
  }

  /** Reads every device and hands its values to the log. Call once, at the top of the loop. */
  public static void refreshAll() {
    if (!RunMode.isReplay()) {
      // One round trip per bus, instead of one per getter.
      for (List<BaseStatusSignal> busSignals : SIGNALS_BY_BUS.values()) {
        BaseStatusSignal.refreshAll(busSignals);
      }
      for (Device device : ALL.values()) {
        device.updateInputs();
      }
    }
    for (Device device : ALL.values()) {
      device.logInputs();
    }
    // After logInputs, so bring-up reads the same values replay would feed it.
    BringUp.log();
  }
}
