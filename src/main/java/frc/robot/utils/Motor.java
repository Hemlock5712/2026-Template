// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.utils;

/**
 * How much torque a motor can actually make at a given speed.
 *
 * <p>A DC motor fights its own back-EMF, so the faster it spins the less torque it has left. The
 * relationship is a straight line from stall torque at 0 rpm down to zero at free speed. That is
 * why a robot accelerates hard off the line and barely at all near top speed.
 *
 * <p>Numbers are from CTRE's published dyno data.
 */
public enum Motor {
  /** Kraken X60 with FOC: more torque, slightly lower free speed. */
  KRAKEN_X60_FOC(9.3615, 5784.65, 476.1),
  /** Kraken X60 without FOC. */
  KRAKEN_X60(7.1573, 6065.33, 374.4);

  private static final double RAD_PER_SEC_TO_RPM = 60.0 / (2.0 * Math.PI);

  private final double stallTorqueNm;
  private final double freeSpeedRpm;
  private final double stallCurrentAmps;

  /** Torque per amp. Torque and current are the same thing in different units. */
  private final double kt;

  Motor(double stallTorqueNm, double freeSpeedRpm, double stallCurrentAmps) {
    this.stallTorqueNm = stallTorqueNm;
    this.freeSpeedRpm = freeSpeedRpm;
    this.stallCurrentAmps = stallCurrentAmps;
    this.kt = stallTorqueNm / stallCurrentAmps;
  }

  /** Torque available at this speed, before any current limit. Zero at or above free speed. */
  public double torqueAtRpm(double rpm) {
    double speed = Math.abs(rpm);
    return speed >= freeSpeedRpm ? 0.0 : stallTorqueNm * (1.0 - speed / freeSpeedRpm);
  }

  /**
   * Torque available at this speed, capped by a current limit. The motor cannot beat {@code kt *
   * amps} however much the speed curve allows.
   */
  public double torqueAtRpm(double rpm, double currentLimitAmps) {
    return Math.min(torqueAtRpm(rpm), kt * currentLimitAmps);
  }

  /** Torque constant, in newton-metres per amp. */
  public double kt() {
    return kt;
  }

  /**
   * The most force one wheel can put into the carpet at this speed - motor curve and current limit
   * together.
   *
   * @param wheelSpeedMps how fast this wheel is rolling
   * @param gearRatio motor rotations per wheel rotation
   * @param wheelRadiusMeters wheel radius
   * @param currentLimitAmps stator current limit for one motor
   * @return force at the contact patch, in newtons
   */
  public double maxForceNewtons(
      double wheelSpeedMps, double gearRatio, double wheelRadiusMeters, double currentLimitAmps) {
    double motorRpm = wheelSpeedMps / wheelRadiusMeters * gearRatio * RAD_PER_SEC_TO_RPM;
    // The gearbox multiplies torque by the ratio; the wheel radius turns torque into force.
    return torqueAtRpm(motorRpm, currentLimitAmps) * gearRatio / wheelRadiusMeters;
  }
}
