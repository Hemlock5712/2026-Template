// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.utils;

import org.wpilib.math.geometry.Rotation2d;
import org.wpilib.math.geometry.Translation2d;
import org.wpilib.math.kinematics.ChassisVelocities;

/**
 * Slows down how fast the drive command is allowed to change, so the wheels never get asked for
 * more force than the carpet or the motors can give.
 *
 * <p>A wheel that breaks loose transmits LESS force than one that grips, so a robot that never
 * slips is a robot that accelerates faster. This is feedforward - it stops us asking for slip in
 * the first place. The stator current limit is still there underneath as the hard stop, because by
 * the time current is high it is already too late.
 *
 * <p>Pure function: same inputs, same answer, no stored state. That makes it testable without a
 * robot and identical during replay.
 */
public final class AccelerationLimiter {
  /** Everything about the robot the limiter needs to know. Measure these, do not guess. */
  public record Config(
      /** Where each module sits relative to robot centre, metres. */
      Translation2d[] moduleLocations,
      /** Robot mass with bumpers and battery, kg. WEIGH IT. */
      double massKg,
      /** Grip in g before the wheels slide. MEASURE IT - see cgHeightMeters for what it means. */
      double frictionCoefficient,
      /**
       * Height of the centre of gravity, metres - decides how much load shifts under acceleration.
       * Set it to 0 to skip load transfer entirely, and then the grip number above is the WHOLE
       * ROBOT's limit in g, not the carpet's: one measured number rather than two guessed ones.
       */
      double cgHeightMeters,
      Motor motor,
      double driveGearRatio,
      double wheelRadiusMeters,
      /** Stator current limit per drive motor, amps. */
      double currentLimitAmps) {}

  private static final double GRAVITY = 9.81;

  /** Friction is solved outright; these only settle the motor curve against the wheel speed. */
  private static final int SOLVER_PASSES = 3;

  private AccelerationLimiter() {}

  /**
   * The velocity to command this loop, given where we are and where we want to be.
   *
   * <p>BOTH velocities must be FIELD-relative, because acceleration only means anything in a frame
   * that is not itself turning. The module geometry is robot-relative, so the heading is what ties
   * the two together - get it wrong and the load shifts onto the wrong wheels.
   *
   * @param current what we commanded last loop, field-relative (not what was measured)
   * @param target what the driver or the path is asking for, field-relative
   * @param heading the robot's heading, to line the field frame up with the module positions
   * @param dt seconds since the last call
   * @return the velocity to command now, field-relative
   */
  public static ChassisVelocities limit(
      ChassisVelocities current,
      ChassisVelocities target,
      Rotation2d heading,
      double dt,
      Config config) {
    if (dt <= 0) {
      return current;
    }

    // Acceleration needed to hit the target in one loop, then cut down to what is possible.
    double ax = (target.vx - current.vx) / dt;
    double ay = (target.vy - current.vy) / dt;
    double alpha = (target.omega - current.omega) / dt;

    // Turn the field-frame acceleration into the robot's own frame, which is where the wheels are.
    double cos = heading.getCos();
    double sin = heading.getSin();
    double axRobot = ax * cos + ay * sin;
    double ayRobot = -ax * sin + ay * cos;
    ChassisVelocities currentRobot = current.toRobotRelative(heading);

    // Start at zero: it predicts the wheel speeds where they are now, which is right to within one
    // loop's acceleration. Starting at 1.0 predicts a speed the robot never reaches.
    double scale = 0.0;
    for (int i = 0; i < SOLVER_PASSES; i++) {
      scale = allowedFraction(axRobot, ayRobot, alpha, currentRobot, dt, config, scale);
    }

    // The END of this interval. This is the ramp's state, which the caller carries into the next
    // call; what it actually COMMANDS is the midpoint between the two (see midpoint()), because
    // holding the end value for the whole interval asks for more speed than the ramp reaches and
    // the motor makes up the difference in torque.
    return new ChassisVelocities(
        current.vx + ax * scale * dt,
        current.vy + ay * scale * dt,
        current.omega + alpha * scale * dt);
  }

  /**
   * How much of the wanted acceleration we can actually have, as a fraction from 0 to 1.
   *
   * <p>Checked per module, because the modules do not share a limit: under acceleration the load
   * shifts backwards, so the rear wheels can push harder than the front ones. The total grip is the
   * same either way, but a limit that ignores this either slips the light wheels or wastes the
   * loaded ones.
   */
  private static double allowedFraction(
      double ax,
      double ay,
      double alpha,
      ChassisVelocities current,
      double dt,
      Config config,
      double previousScale) {
    Translation2d[] locations = config.moduleLocations();
    int count = locations.length;
    double massPerModule = config.massKg() / count;

    // Where the previous pass says we will be by the end of this loop.
    double predictedVx = current.vx + ax * previousScale * dt;
    double predictedVy = current.vy + ay * previousScale * dt;
    double predictedOmega = current.omega + alpha * previousScale * dt;

    // Newtons moved PER UNIT OF SCALE - linear in scale, which is what makes the solve below exact.
    double transferX =
        shift(config.massKg(), ax, config.cgHeightMeters(), 2.0 * maxAbs(locations, true));
    double transferY =
        shift(config.massKg(), ay, config.cgHeightMeters(), 2.0 * maxAbs(locations, false));

    double staticNormal = config.massKg() * GRAVITY / count;
    double fraction = 1.0;

    for (Translation2d location : locations) {
      // Translation plus the tangential push of angular acceleration. A steady spin is NOT in here:
      // the force holding a module on its circle comes through the frame, not the carpet.
      double forceX = ax - alpha * location.getY();
      double forceY = ay + alpha * location.getX();
      double commandedForce = massPerModule * Math.hypot(forceX, forceY);
      if (commandedForce <= 1e-9) {
        continue;
      }

      // scale * commanded <= mu * (static - scale * transfer), solved for scale. A wheel the load
      // moves ONTO gains grip faster than demand, goes negative here, and can never be the slipper.
      double transfer =
          Math.signum(location.getX()) * transferX + Math.signum(location.getY()) * transferY;
      double denominator = commandedForce + config.frictionCoefficient() * transfer;
      if (denominator > 0.0) {
        fraction = Math.min(fraction, config.frictionCoefficient() * staticNormal / denominator);
      }

      double moduleVx = predictedVx - predictedOmega * location.getY();
      double moduleVy = predictedVy + predictedOmega * location.getX();

      // A motor only loses torque to speed when it is DRIVING. Back-driven it fights nothing -
      // back-EMF helps push current through - so braking gets the full current-limited force at any
      // speed. Passing 0 asks the curve for exactly that. Without this a robot at 4.5 m/s is
      // modelled as unable to stop, because the motoring curve is zero at free speed.
      boolean braking = forceX * moduleVx + forceY * moduleVy < 0.0;
      double motorForce =
          config
              .motor()
              .maxForceNewtons(
                  braking ? 0.0 : Math.hypot(moduleVx, moduleVy),
                  config.driveGearRatio(),
                  config.wheelRadiusMeters(),
                  config.currentLimitAmps());
      fraction = Math.min(fraction, motorForce / commandedForce);
    }
    return Math.max(0.0, Math.min(1.0, fraction));
  }

  /** The average of two velocities - what to command over an interval that ends at {@code next}. */
  public static ChassisVelocities midpoint(ChassisVelocities current, ChassisVelocities next) {
    return new ChassisVelocities(
        (current.vx + next.vx) * 0.5,
        (current.vy + next.vy) * 0.5,
        (current.omega + next.omega) * 0.5);
  }

  /**
   * Newtons shifted off one axle onto the other, per wheel. The trailing /2 assumes TWO WHEELS PER
   * AXLE: right for a rectangle, wrong for anything else - a diamond gets half the transfer it
   * owes.
   */
  private static double shift(double massKg, double accel, double cgHeight, double wheelbase) {
    return wheelbase <= 0 ? 0.0 : massKg * accel * cgHeight / wheelbase / 2.0;
  }

  private static double maxAbs(Translation2d[] locations, boolean useX) {
    double most = 0.0;
    for (Translation2d location : locations) {
      most = Math.max(most, Math.abs(useX ? location.getX() : location.getY()));
    }
    return most;
  }
}
