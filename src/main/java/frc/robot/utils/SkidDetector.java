// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.utils;

import org.wpilib.math.geometry.Translation2d;
import org.wpilib.math.kinematics.SwerveModuleVelocity;

/**
 * Spots a wheel that is lying about how fast the robot is moving.
 *
 * <p>A robot cannot stretch, so once you subtract the part of each wheel's motion that comes from
 * spinning, all four must agree about how fast the body is travelling. The one that disagrees is
 * slipping. The answer is a ratio: 1.0 means perfect agreement, higher means someone is wrong.
 *
 * <p>READ THIS BEFORE TRUSTING IT. It is instrumentation, not truth, and it has two blind spots:
 *
 * <ul>
 *   <li>It is a RELATIVE measure. Floor it from a standstill and break all four wheels loose
 *       together and they still agree with each other - ratio 1.0, no skid reported.
 *   <li>A module only reports how fast it is rolling, never how fast it is sliding sideways. A
 *       uniform sideways slide is invisible.
 * </ul>
 */
public final class SkidDetector {
  /**
   * Below this the ratio is meaningless: the smallest wheel vector goes to zero when the robot is
   * spinning on the spot or barely moving, and dividing by it produces nonsense. Measured on a
   * slip-free simulation, an ungated ratio still spikes to 1.31 at 0.55 m/s from steer lag alone.
   */
  private static final double MIN_TRANSLATION_MPS = 0.3;

  private SkidDetector() {}

  /**
   * How much the wheels disagree, as max/min of what each thinks the robot's translation is.
   *
   * @param measured each module's measured speed and angle
   * @param locations module positions relative to robot centre, same order
   * @param omegaRadPerSec measured rotation rate, from the gyro
   * @return the ratio, or 1.0 (meaning "nothing to report") when moving too slowly to tell
   */
  public static double ratio(
      SwerveModuleVelocity[] measured, Translation2d[] locations, double omegaRadPerSec) {
    if (measured.length == 0 || measured.length != locations.length) {
      return 1.0;
    }

    double smallest = Double.MAX_VALUE;
    double largest = 0.0;
    for (int i = 0; i < measured.length; i++) {
      // Subtract the rotation as a VECTOR, not a speed. Doing it with scalars looks fine driving in
      // a straight line and falls apart the moment you rotate and translate at once.
      double wheelX = measured[i].velocity * measured[i].angle.getCos();
      double wheelY = measured[i].velocity * measured[i].angle.getSin();
      double spinX = -omegaRadPerSec * locations[i].getY();
      double spinY = omegaRadPerSec * locations[i].getX();

      double translation = Math.hypot(wheelX - spinX, wheelY - spinY);
      smallest = Math.min(smallest, translation);
      largest = Math.max(largest, translation);
    }

    // Gate on the SMALLEST, not the largest. A slipping wheel makes the largest big, so gating on
    // that would hide the very thing we are looking for - and any module mid-turn drives its own
    // estimate toward zero, which turns the ratio into noise. Only judge when every wheel agrees
    // the robot is moving.
    return smallest < MIN_TRANSLATION_MPS ? 1.0 : largest / smallest;
  }
}
