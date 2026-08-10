// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.utils;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertTrue;

import org.junit.jupiter.api.Test;
import org.wpilib.math.geometry.Rotation2d;
import org.wpilib.math.geometry.Translation2d;
import org.wpilib.math.kinematics.ChassisVelocities;

class AccelerationLimiterTest {
  private static final double HALF = 0.254;
  private static final double DT = 0.005; // Robot.PERIOD_SECONDS

  private static final AccelerationLimiter.Config CONFIG =
      new AccelerationLimiter.Config(
          new Translation2d[] {
            new Translation2d(HALF, HALF),
            new Translation2d(HALF, -HALF),
            new Translation2d(-HALF, HALF),
            new Translation2d(-HALF, -HALF),
          },
          60.0,
          1.1,
          0.2,
          Motor.KRAKEN_X60_FOC,
          7.3636,
          0.05504,
          120.0);

  private static ChassisVelocities stopped() {
    return new ChassisVelocities();
  }

  @Test
  void smallRequestsPassThroughUntouched() {
    // Creeping from a stop needs almost no force, so the ramp reaches the target within one
    // interval. This is the ramp's END state; the caller commands the midpoint of it.
    var out =
        AccelerationLimiter.limit(
            stopped(), new ChassisVelocities(0.01, 0, 0), Rotation2d.kZero, DT, CONFIG);
    assertEquals(0.01, out.vx, 1e-9);
  }

  @Test
  void aFloorItRequestIsClipped() {
    // Ask for 5 m/s instantly - that is 1000 m/s^2, far past what carpet can hold.
    var out =
        AccelerationLimiter.limit(
            stopped(), new ChassisVelocities(5.0, 0, 0), Rotation2d.kZero, DT, CONFIG);
    double impliedAccel = out.vx / DT;
    assertTrue(
        impliedAccel < 1.1 * 9.81 + 1e-6,
        "commanded acceleration " + impliedAccel + " exceeded the friction limit");
    assertTrue(impliedAccel > 0.0, "should still move");
  }

  @Test
  void spinningFastEatsTheTractionBudget() {
    // Centripetal force to hold the modules on their circle is real and is spent first. At 6 rad/s
    // it alone exceeds the friction budget, so almost nothing is left for driving.
    var spinning = new ChassisVelocities(0, 0, 6.0);
    var driving =
        AccelerationLimiter.limit(
            spinning, new ChassisVelocities(5.0, 0, 6.0), Rotation2d.kZero, DT, CONFIG);
    var still =
        AccelerationLimiter.limit(
            stopped(), new ChassisVelocities(5.0, 0, 0), Rotation2d.kZero, DT, CONFIG);
    assertTrue(
        driving.vx - spinning.vx < still.vx,
        "a fast spin must leave less room to accelerate than standing still");
  }

  @Test
  void theMotorLimitBitesAtSpeed() {
    // A Kraken near free speed has little torque left, so the same request that flies from a stop
    // barely moves near the top. Friction binds below ~4.2 m/s and the motor curve above it, so
    // this has to be sampled clear of that crossover to mean anything.
    var fromStop =
        AccelerationLimiter.limit(
            stopped(), new ChassisVelocities(9.0, 0, 0), Rotation2d.kZero, DT, CONFIG);
    var fast = new ChassisVelocities(4.4, 0, 0);
    var atSpeed =
        AccelerationLimiter.limit(
            fast, new ChassisVelocities(9.0, 0, 0), Rotation2d.kZero, DT, CONFIG);
    assertTrue(
        atSpeed.vx - fast.vx < fromStop.vx,
        "acceleration available at 4.2 m/s must be less than from a standstill");
  }

  @Test
  void theLimitDoesNotDependOnWhichWayTheRobotFaces() {
    // Quarter turns map the square wheelbase onto itself, so the same field-relative request must
    // be clipped identically. This fails if the field-frame acceleration reaches the robot-frame
    // module geometry unrotated: the load then shifts onto the wrong wheels and a sideways-facing
    // robot is allowed to accelerate harder than a forwards-facing one.
    //
    // Only quarter turns. At 45 degrees the answer legitimately differs - a diagonal push unloads
    // one corner by BOTH transfer components, where an axis-aligned push spreads it over an axle.
    // That is the cost of splitting force equally between modules, not a frame error.
    var request = new ChassisVelocities(9.0, 0, 0);
    double straight =
        magnitude(AccelerationLimiter.limit(stopped(), request, Rotation2d.kZero, DT, CONFIG));
    for (double degrees : new double[] {90, 180, 270, -90}) {
      double turned =
          magnitude(
              AccelerationLimiter.limit(
                  stopped(), request, Rotation2d.fromDegrees(degrees), DT, CONFIG));
      assertEquals(straight, turned, 1e-9, "heading " + degrees + " deg changed the limit");
    }
  }

  private static double magnitude(ChassisVelocities velocity) {
    return Math.hypot(velocity.vx, velocity.vy);
  }

  @Test
  void brakingIsLimitedJustLikeAccelerating() {
    // Taken from a real DriveToPose log: the profile ended, the target jumped from -0.618 to +0.010
    // while the robot was still commanded at 0.469 m/s, and the limiter let through far more
    // deceleration than the carpet can supply. Braking shifts load onto the FRONT wheels instead of
    // the rear, but the magnitude of the limit is the same either way.
    var current = new ChassisVelocities(0.469, 0, 0);
    var target = new ChassisVelocities(0.010, -0.070, 0);
    var out = AccelerationLimiter.limit(current, target, Rotation2d.kZero, DT, CONFIG);

    double accel = Math.hypot(out.vx - current.vx, out.vy - current.vy) / DT;
    assertTrue(accel <= 5.79, "braking at " + accel + " m/s^2 exceeds the traction limit");
  }

  @Test
  void alreadyAtTargetHoldsSteady() {
    var cruising = new ChassisVelocities(2.0, 0, 0);
    var out = AccelerationLimiter.limit(cruising, cruising, Rotation2d.kZero, DT, CONFIG);
    assertEquals(2.0, out.vx, 1e-9);
  }

  @Test
  void zeroDtIsNotADivideByZero() {
    var cruising = new ChassisVelocities(2.0, 0, 0);
    var out =
        AccelerationLimiter.limit(
            cruising, new ChassisVelocities(3.0, 0, 0), Rotation2d.kZero, 0.0, CONFIG);
    assertEquals(2.0, out.vx, 1e-9);
  }
}
