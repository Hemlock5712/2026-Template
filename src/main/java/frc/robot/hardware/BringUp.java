// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.hardware;

import java.util.LinkedHashMap;
import java.util.Map;
import org.littletonrobotics.junction.Logger;

/**
 * Measures what a mechanism actually does, so you can check it against what the code claims. Move
 * the mechanism - by hand while disabled, or with any OpMode - and read {@code
 * BringUp/&lt;name&gt;/MeasuredRatio}: that is the real rotor-to-mechanism gear ratio, sign
 * included. See the device-bringup skill.
 *
 * <p>Everything is matched by log name. A {@link LoggedTalonFX} and a {@link LoggedCANcoder} both
 * called "Arm" measure each other, and the ratio is the gear ratio. A motor named "Arm/2" is taken
 * to be a <b>follower</b> of "Arm" and is measured against the leader's rotor instead, so its ratio
 * should read +1 (same direction) or -1 (opposed). A motor with neither is skipped - there is
 * nothing to compare its rotor against.
 *
 * <p>Everything here is computed from logged inputs, so it replays.
 */
public final class BringUp {
  private BringUp() {}

  // ponytail: always on rather than a bring-up OpMode. Five doubles per pair, and it means every
  // log - sim, match, hand-moved on the bench - already carries the measurement.

  // Everything here is rotations over rotations, so the ratio is unitless - no conversions.
  //
  // A CANcoder position signal steps in 1/4096 rot (measured off a log, not a datasheet), so 0.01
  // rot of travel makes the ratio good to about 2%, sharpening the further the mechanism moves.
  // Sized off the signal, not off degrees, because a short-throw wrist never gets many degrees.
  private static final double MIN_TRAVEL_ROT = 0.01;

  private static final Map<String, LoggedTalonFX> MOTORS = new LinkedHashMap<>();
  private static final Map<String, LoggedCANcoder> ENCODERS = new LinkedHashMap<>();
  private static final Map<String, Baseline> BASELINES = new LinkedHashMap<>();

  /** Where a pair started this run, plus the best-travelled sample seen so far. */
  private static final class Baseline {
    boolean captured;
    double rotorRot;
    double sensorRot;
    double bestTravelRot;
    double bestRatio = Double.NaN;
  }

  static void add(String name, LoggedTalonFX motor) {
    MOTORS.put(name, motor);
  }

  static void add(String name, LoggedCANcoder encoder) {
    ENCODERS.put(name, encoder);
  }

  /** "Arm/2" follows "Arm". Returns null for a name with no slash. */
  private static String leaderOf(String name) {
    int slash = name.lastIndexOf('/');
    return slash < 0 ? null : name.substring(0, slash);
  }

  /** Logs one sample per measurable motor. Called from {@link LoggedHardware#refreshAll}. */
  static void log() {
    for (Map.Entry<String, LoggedTalonFX> entry : MOTORS.entrySet()) {
      String name = entry.getKey();
      LoggedTalonFX motor = entry.getValue();

      // What this rotor gets compared against: its own CANcoder, or a leader's rotor.
      LoggedCANcoder encoder = ENCODERS.get(name);
      String leaderName = leaderOf(name);
      LoggedTalonFX leader = encoder != null || leaderName == null ? null : MOTORS.get(leaderName);
      if (encoder == null && leader == null) {
        continue;
      }

      Baseline baseline = BASELINES.computeIfAbsent(name, key -> new Baseline());
      if (!baseline.captured) {
        // Wait for both devices to actually answer - signals read 0 until they do, and a bogus
        // zero baseline poisons every later measurement.
        boolean referenceReady = encoder != null ? encoder.isConnected() : leader.isConnected();
        if (!motor.isConnected() || !referenceReady) {
          continue;
        }
        baseline.rotorRot = motor.getRotorPositionRot();
        baseline.sensorRot =
            encoder != null ? encoder.getPositionRot() : leader.getRotorPositionRot();
        baseline.captured = true;
      }

      double rotorTravel = motor.getRotorPositionRot() - baseline.rotorRot;
      double sensorTravel =
          (encoder != null ? encoder.getPositionRot() : leader.getRotorPositionRot())
              - baseline.sensorRot;
      double travel = Math.abs(sensorTravel);
      double ratio = travel >= MIN_TRAVEL_ROT ? rotorTravel / sensorTravel : Double.NaN;

      if (travel > baseline.bestTravelRot) {
        baseline.bestTravelRot = travel;
        baseline.bestRatio = ratio;
      }

      // Rotor travel isn't logged: it's just ratio * sensor travel, and it's already in the log as
      // Hardware/TalonFX/<name>/RotorPositionRot.
      String key = "BringUp/" + name;
      Logger.recordOutput(key + "/SensorTravelRot", sensorTravel);
      Logger.recordOutput(key + "/MeasuredRatio", baseline.bestRatio);
    }
  }
}
