// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.hardware;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.CANBus;
import frc.robot.Robot;
import frc.robot.utils.RunMode;
import java.util.ArrayList;
import java.util.HashSet;
import java.util.LinkedHashMap;
import java.util.List;
import java.util.Map;
import java.util.Set;

/**
 * Reads every logged device once per loop. {@link frc.robot.Robot} calls {@link #refreshAll} at the
 * top of the loop; individual mechanisms never call it.
 *
 * <p>On a robot this asks the CAN bus for every signal in a single round trip, then writes the
 * answers to the log. During replay it skips the bus and fills the same fields from the log, so
 * mechanism code cannot tell which run it is in.
 */
public final class LoggedHardware {
  private LoggedHardware() {}

  /** One device that can be read and logged. Implemented by the Logged* wrappers. */
  interface Device {
    BaseStatusSignal[] signals();

    void updateInputs();

    void logInputs();
  }

  // Registration order = construction order, so replay reads devices in the same sequence.
  private static final List<Device> ALL = new ArrayList<>();
  private static final Set<String> KEYS = new HashSet<>();

  // ONE BATCH PER CAN BUS. Phoenix's refreshAll takes the bus of the first signal and, if any
  // other signal is on a different one, marks them all InvalidNetwork and refreshes NOTHING -
  // silently freezing every sensor on the robot. Mixing buses in one call is not an option.
  private static final Map<String, List<BaseStatusSignal>> SIGNALS_BY_BUS = new LinkedHashMap<>();

  /** Registers a device that has no signals of its own to batch, such as the swerve drivetrain. */
  static void register(Device device, String logKey) {
    if (!KEYS.add(logKey)) {
      throw new IllegalArgumentException("Two logged devices named \"" + logKey + "\"");
    }
    ALL.add(device);
  }

  static void register(Device device, String logKey, CANBus bus) {
    if (!KEYS.add(logKey)) {
      throw new IllegalArgumentException("Two logged devices named \"" + logKey + "\"");
    }
    ALL.add(device);
    SIGNALS_BY_BUS
        .computeIfAbsent(bus.getName(), name -> new ArrayList<>())
        .addAll(List.of(device.signals()));
  }

  // Devices default to 100 Hz on CAN FD, so a faster loop would just re-read the same value. Ask
  // for one sample per loop instead. Phoenix promotes unsupported rates to the next one up, and
  // caps at 1000 Hz. This costs CAN bandwidth - check CANBus.getStatus().BusUtilization after
  // changing it, and keep the swerve bus to itself if it climbs.
  private static final double SIGNAL_FREQUENCY_HZ = 1.0 / Robot.PERIOD_SECONDS;

  private static boolean initialized = false;

  /**
   * Asks every device for one sample per robot loop. Best called once from {@link
   * frc.robot.Robot}'s constructor - by then every device has registered, and these are blocking
   * calls that have no business in the periodic path. Forgetting it is not fatal: the first {@link
   * #refreshAll} does it instead, costing one slow loop.
   */
  public static void initialize() {
    initialized = true;
    if (RunMode.current() == RunMode.REPLAY) {
      return; // no devices to configure; the log already holds what they said
    }
    for (List<BaseStatusSignal> busSignals : SIGNALS_BY_BUS.values()) {
      BaseStatusSignal.setUpdateFrequencyForAll(SIGNAL_FREQUENCY_HZ, busSignals);
    }
  }

  /** Reads every device and hands its values to the log. Call once, at the top of the loop. */
  public static void refreshAll() {
    if (!initialized) {
      initialize();
    }
    if (RunMode.current() != RunMode.REPLAY) {
      // One round trip per bus, instead of one per getter.
      for (List<BaseStatusSignal> busSignals : SIGNALS_BY_BUS.values()) {
        BaseStatusSignal.refreshAll(busSignals);
      }
      for (Device device : ALL) {
        device.updateInputs();
      }
    }
    for (Device device : ALL) {
      device.logInputs();
    }
    // After logInputs, so bring-up reads the same values replay would feed it.
    BringUp.log();
  }
}
