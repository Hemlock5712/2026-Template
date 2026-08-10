// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.utils;

import org.wpilib.driverstation.Alliance;
import org.wpilib.driverstation.DriverStationErrors;
import org.wpilib.driverstation.MatchState;
import org.wpilib.math.geometry.Pose2d;
import org.wpilib.math.geometry.Rotation2d;
import org.wpilib.math.geometry.Translation2d;
import org.wpilib.math.kinematics.ChassisAccelerations;
import org.wpilib.math.kinematics.ChassisVelocities;
import org.wpilib.vision.apriltag.AprilTagFieldLayout;
import org.wpilib.vision.apriltag.AprilTagFields;

/**
 * Turns a blue-alliance field position into the matching red one, so autos only get written once.
 *
 * <p>The field origin sits in the blue corner and never moves, so the same coordinates mean
 * different physical spots per side. Spinning the field 180° about its center lands red exactly
 * where blue was - that's the flip.
 *
 * <p>Author every pose blue-origin and let this handle red. See the {@code game-info} skill.
 */
public final class AllianceFlip {
  private AllianceFlip() {}

  // Field size comes from the AprilTag layout, so it's never a hardcoded season number.
  // TODO: swap to the 2027 field once WPILib ships it - kDefaultField is still 2026.
  // If loading fails we keep going without flipping: wrong side beats a dead robot.
  private static final AprilTagFieldLayout FIELD = loadFieldOrNull();

  private static AprilTagFieldLayout loadFieldOrNull() {
    try {
      return AprilTagFieldLayout.loadField(AprilTagFields.kDefaultField);
    } catch (RuntimeException e) {
      DriverStationErrors.reportError(
          "AllianceFlip: no field layout (" + e + ") - red autos will NOT flip", false);
      return null;
    }
  }

  /** True when the driver station says we are on red. Unknown alliance counts as blue. */
  public static boolean isRed() {
    return MatchState.getAlliance().orElse(Alliance.BLUE) == Alliance.RED;
  }

  /**
   * True when a blue-authored position needs flipping. Check this once instead of calling {@link
   * #apply} in a loop - it reads the driver station every time.
   */
  public static boolean shouldFlip() {
    return isRed() && FIELD != null;
  }

  /**
   * The given blue-origin pose, flipped to the red side if we are on red. On blue it is returned
   * unchanged.
   */
  public static Pose2d apply(Pose2d bluePose) {
    return shouldFlip() ? flip(bluePose) : bluePose;
  }

  /** Flips a pose regardless of alliance. Only call this behind {@link #shouldFlip()}. */
  public static Pose2d flip(Pose2d pose) {
    return new Pose2d(
        flip(pose.getTranslation()), new Rotation2d(flipHeading(pose.getRotation().getRadians())));
  }

  /** Field length in m, or 0 if the layout failed to load. */
  public static double fieldLength() {
    return FIELD == null ? 0 : FIELD.getFieldLength();
  }

  /** Field width in m, or 0 if the layout failed to load. */
  public static double fieldWidth() {
    return FIELD == null ? 0 : FIELD.getFieldWidth();
  }

  /** Flips a field position regardless of alliance. */
  public static Translation2d flip(Translation2d position) {
    if (FIELD == null) {
      return position;
    }
    return new Translation2d(
        FIELD.getFieldLength() - position.getX(), FIELD.getFieldWidth() - position.getY());
  }

  /**
   * Flips a heading regardless of alliance. Does NOT wrap to +/-pi, so a continuously-unwrapped
   * heading (like a planned path's) stays continuous - wrapping it would make an interpolation
   * across the seam sweep almost a full turn the wrong way.
   */
  public static double flipHeading(double headingRadians) {
    return headingRadians + Math.PI;
  }

  /**
   * Flips a field-relative velocity regardless of alliance. Both linear components negate; omega
   * keeps its sign, because rotating the whole field preserves handedness.
   */
  public static ChassisVelocities flip(ChassisVelocities velocity) {
    return new ChassisVelocities(-velocity.vx, -velocity.vy, velocity.omega);
  }

  /** Flips a field-relative acceleration regardless of alliance. Same signs as {@link #flip}. */
  public static ChassisAccelerations flip(ChassisAccelerations acceleration) {
    return new ChassisAccelerations(-acceleration.ax, -acceleration.ay, acceleration.alpha);
  }
}
