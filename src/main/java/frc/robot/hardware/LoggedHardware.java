// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.hardware;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.CANBus;
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

  static void register(Device device, String logKey, CANBus bus) {
    if (!KEYS.add(logKey)) {
      throw new IllegalArgumentException("Two logged devices named \"" + logKey + "\"");
    }
    ALL.add(device);
    SIGNALS_BY_BUS
        .computeIfAbsent(bus.getName(), name -> new ArrayList<>())
        .addAll(List.of(device.signals()));
  }

  /** Reads every device and hands its values to the log. Call once, at the top of the loop. */
  public static void refreshAll() {
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
  }
}
