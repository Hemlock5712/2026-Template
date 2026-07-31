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
 * <p>Pairs a motor with an encoder by log name: a {@link LoggedTalonFX} and a {@link
 * LoggedCANcoder} both called "Arm" measure each other. A motor with no matching encoder is skipped
 * - there is nothing to compare its rotor against.
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

  /** Logs one sample per motor/encoder pair. Called from {@link LoggedHardware#refreshAll}. */
  static void log() {
    for (Map.Entry<String, LoggedTalonFX> entry : MOTORS.entrySet()) {
      String name = entry.getKey();
      LoggedTalonFX motor = entry.getValue();
      LoggedCANcoder encoder = ENCODERS.get(name);
      if (encoder == null) {
        continue;
      }

      Baseline baseline = BASELINES.computeIfAbsent(name, key -> new Baseline());
      if (!baseline.captured) {
        // Wait for both devices to actually answer - signals read 0 until they do, and a bogus
        // zero baseline poisons every later measurement.
        if (!motor.isConnected() || !encoder.isConnected()) {
          continue;
        }
        baseline.rotorRot = motor.getRotorPositionRot();
        baseline.sensorRot = encoder.getPositionRot();
        baseline.captured = true;
      }

      double rotorTravel = motor.getRotorPositionRot() - baseline.rotorRot;
      double sensorTravel = encoder.getPositionRot() - baseline.sensorRot;
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
