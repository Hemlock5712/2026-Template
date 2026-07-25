// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.utils;

import org.wpilib.driverstation.Alliance;
import org.wpilib.driverstation.DriverStationErrors;
import org.wpilib.driverstation.MatchState;
import org.wpilib.math.geometry.Pose2d;
import org.wpilib.math.geometry.Rotation2d;
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
   * The given blue-origin pose, flipped to the red side if we are on red. On blue it is returned
   * unchanged.
   */
  public static Pose2d apply(Pose2d bluePose) {
    if (!isRed() || FIELD == null) {
      return bluePose;
    }
    return new Pose2d(
        FIELD.getFieldLength() - bluePose.getX(),
        FIELD.getFieldWidth() - bluePose.getY(),
        bluePose.getRotation().plus(Rotation2d.k180deg));
  }
}
