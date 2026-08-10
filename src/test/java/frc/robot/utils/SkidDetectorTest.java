// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.utils;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertTrue;

import org.junit.jupiter.api.Test;
import org.wpilib.math.geometry.Rotation2d;
import org.wpilib.math.geometry.Translation2d;
import org.wpilib.math.kinematics.SwerveModuleVelocity;

class SkidDetectorTest {
  private static final double HALF = 0.254; // 10 in, matching TunerConstants

  private static final Translation2d[] LOCATIONS = {
    new Translation2d(HALF, HALF),
    new Translation2d(HALF, -HALF),
    new Translation2d(-HALF, HALF),
    new Translation2d(-HALF, -HALF),
  };

  private static SwerveModuleVelocity[] straightAhead(double... speeds) {
    var states = new SwerveModuleVelocity[speeds.length];
    for (int i = 0; i < speeds.length; i++) {
      states[i] = new SwerveModuleVelocity(speeds[i], Rotation2d.kZero);
    }
    return states;
  }

  @Test
  void agreeingWheelsReportNoSkid() {
    assertEquals(1.0, SkidDetector.ratio(straightAhead(3, 3, 3, 3), LOCATIONS, 0.0), 1e-9);
  }

  @Test
  void oneFastWheelIsCaught() {
    // Front-left spinning 30% fast.
    double ratio = SkidDetector.ratio(straightAhead(3.9, 3, 3, 3), LOCATIONS, 0.0);
    assertEquals(1.3, ratio, 1e-9);
  }

  @Test
  void spinningInPlaceDoesNotDivideByZero() {
    // Pure rotation: every wheel is moving, but the robot's translation is zero. Without the speed
    // gate the ratio would be 0/0.
    double omega = 4.0;
    var states = new SwerveModuleVelocity[4];
    for (int i = 0; i < 4; i++) {
      double vx = -omega * LOCATIONS[i].getY();
      double vy = omega * LOCATIONS[i].getX();
      states[i] = new SwerveModuleVelocity(Math.hypot(vx, vy), new Rotation2d(Math.atan2(vy, vx)));
    }
    double ratio = SkidDetector.ratio(states, LOCATIONS, omega);
    assertTrue(Double.isFinite(ratio), "ratio must stay finite while spinning on the spot");
    assertEquals(1.0, ratio, 1e-6);
  }

  @Test
  void aModuleMidTurnDoesNotFakeASkid() {
    // One module pointing sideways contributes almost nothing to forward translation, which would
    // drive the ratio sky-high if the gate used the largest value instead of the smallest.
    var states =
        new SwerveModuleVelocity[] {
          new SwerveModuleVelocity(3.0, Rotation2d.kZero),
          new SwerveModuleVelocity(3.0, Rotation2d.kZero),
          new SwerveModuleVelocity(3.0, Rotation2d.kZero),
          new SwerveModuleVelocity(0.05, Rotation2d.kCCW_90deg),
        };
    assertEquals(1.0, SkidDetector.ratio(states, LOCATIONS, 0.0), 1e-9);
  }

  @Test
  void allFourSlippingTogetherIsInvisible() {
    // The documented blind spot: flooring it from a stop breaks all four loose equally, they still
    // agree with each other, and the ratio reports nothing. This test exists so nobody "fixes" it.
    assertEquals(1.0, SkidDetector.ratio(straightAhead(9, 9, 9, 9), LOCATIONS, 0.0), 1e-9);
  }

  @Test
  void rotatingWhileTranslatingStillAgrees() {
    // Subtracting the spin as a vector must hold up when both are happening at once. A scalar
    // version of this maths passes the straight-line tests above and fails this one.
    double omega = 2.0;
    double vx = 2.5;
    double vy = -1.0;
    var states = new SwerveModuleVelocity[4];
    for (int i = 0; i < 4; i++) {
      double wheelX = vx - omega * LOCATIONS[i].getY();
      double wheelY = vy + omega * LOCATIONS[i].getX();
      states[i] =
          new SwerveModuleVelocity(
              Math.hypot(wheelX, wheelY), new Rotation2d(Math.atan2(wheelY, wheelX)));
    }
    assertEquals(1.0, SkidDetector.ratio(states, LOCATIONS, omega), 1e-9);
  }
}
