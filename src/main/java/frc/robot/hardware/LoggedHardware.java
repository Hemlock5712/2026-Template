// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.hardware;

import com.ctre.phoenix6.BaseStatusSignal;
import frc.robot.utils.RunMode;
import java.util.ArrayList;
import java.util.HashSet;
import java.util.List;
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
  private static BaseStatusSignal[] allSignals = new BaseStatusSignal[0];

  static void register(Device device, String logKey) {
    if (!KEYS.add(logKey)) {
      throw new IllegalArgumentException("Two logged devices named \"" + logKey + "\"");
    }
    ALL.add(device);

    List<BaseStatusSignal> collected = new ArrayList<>();
    for (Device each : ALL) {
      collected.addAll(List.of(each.signals()));
    }
    allSignals = collected.toArray(new BaseStatusSignal[0]);
  }

  /** Reads every device and hands its values to the log. Call once, at the top of the loop. */
  public static void refreshAll() {
    if (RunMode.current() != RunMode.REPLAY) {
      // One bus round trip for the whole robot, instead of one per getter.
      BaseStatusSignal.refreshAll(allSignals);
      for (Device device : ALL) {
        device.updateInputs();
      }
    }
    for (Device device : ALL) {
      device.logInputs();
    }
  }
}
