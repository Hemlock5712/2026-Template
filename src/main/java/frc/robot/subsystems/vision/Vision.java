// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.vision;

import com.limelightvision.Limelight;
import com.limelightvision.Limelight.PoseEstimateConfig;
import frc.robot.hardware.LoggedLimelight;
import frc.robot.hardware.LoggedLimelight.Estimate;
import frc.robot.subsystems.DriveMechanism;
import frc.robot.utils.RunMode;
import org.littletonrobotics.junction.Logger;
import org.wpilib.command3.Scheduler;
import org.wpilib.math.geometry.Pose2d;
import org.wpilib.math.linalg.VecBuilder;

/**
 * Feeds each Limelight's AprilTag pose estimates into the drivetrain's pose estimator. Call {@link
 * #registerAll} once from {@link frc.robot.Robot}.
 *
 * <p>MegaTag1 solves position from the tags alone, so it needs 2+ tags. MegaTag2 leans on the gyro
 * heading we send it, so one tag is enough (seed the gyro, or single-tag vision will be off).
 * Vision only corrects x/y; the gyro owns heading.
 *
 * <p>EVERY TRUST DECISION LIVES HERE, not in the library config, so an old log can be replayed with
 * different numbers to find the right ones. The library is left to reject only frames that are
 * structurally broken - a missing pose, a non-finite number - which is never a tuning question.
 *
 * <p>Does nothing in sim (no camera). Accepted and rejected estimates are logged under {@code
 * Vision/*}.
 */
public class Vision {
  // How far away tags are still trusted; past this they are too noisy to help.
  private static final double MAX_TAG_DISTANCE_METERS = 4.0;

  // Trust numbers (smaller = trust vision more): xy = 0.15 * d^2 / sqrt(n). Error grows with
  // distance squared (farther tags look smaller); more tags average the noise down.
  private static final double XY_STD_DEV = 0.15;
  private static final double DISTANCE_EXPONENT = 2.0;

  // Heading never comes from vision, so make the estimator ignore it entirely.
  private static final double HEADING_STD_DEV = Double.MAX_VALUE;

  private static final int MT1_MIN_TAGS = 2;
  private static final int MT2_MIN_TAGS = 1;

  // Spin faster than this (1 full turn per second) and we stop trusting MegaTag2. See accept().
  private static final double MAX_SPIN_FOR_MT2_RAD_PER_SEC = 2 * Math.PI;

  // Let the library through on anything structurally sound; the gates below are ours.
  private static final PoseEstimateConfig PERMISSIVE_MT1 =
      PoseEstimateConfig.defaultMT1().withMinTagCount(1).withMaxAvgTagDistance(Double.MAX_VALUE);
  private static final PoseEstimateConfig PERMISSIVE_MT2 =
      PoseEstimateConfig.defaultMT2().withMinTagCount(1).withMaxAvgTagDistance(Double.MAX_VALUE);

  private final LoggedLimelight camera;
  private final DriveMechanism drivetrain;

  private Vision(LoggedLimelight camera, DriveMechanism drivetrain) {
    this.camera = camera;
    this.drivetrain = drivetrain;
  }

  /** Wires every camera: relax the library's gates and run each camera's update every loop. */
  public static void registerAll(DriveMechanism drivetrain, LoggedLimelight... cameras) {
    // MegaTag2 needs to know which way we're facing, so send every camera our heading (degrees,
    // CCW+). One call covers all of them - see setUseSharedOrientation below.
    //
    // ORDER MATTERS: these run in the order registered, so send the heading BEFORE reading the
    // cameras. And keep it at 50 Hz - this call ends in a full NetworkTables flush, which is far
    // too expensive for the fast odometry thread.
    Scheduler.getDefault()
        .addPeriodic(
            () -> {
              if (RunMode.current() == RunMode.REPLAY) {
                return; // nothing to talk to; the log already holds what the cameras answered
              }
              Limelight.setSharedRobotOrientation(
                  drivetrain.getPose().getRotation().getDegrees(),
                  Math.toDegrees(drivetrain.getFieldVelocity().omega),
                  0,
                  0,
                  0,
                  0);
            });

    for (LoggedLimelight camera : cameras) {
      camera
          .camera()
          .withPoseEstimateConfig_MT1(PERMISSIVE_MT1)
          .withPoseEstimateConfig_MT2(PERMISSIVE_MT2);
      camera.camera().setUseSharedOrientation(true); // heading comes from the shared feed above
      Vision vision = new Vision(camera, drivetrain);
      Scheduler.getDefault().addPeriodic(vision::update);
    }
  }

  /** Runs one vision update: every new camera frame becomes at most one pose measurement. */
  private void update() {
    // Always, even if nothing is used from it - the log has one entry per loop either way.
    camera.refresh();

    // Spinning fast means the heading we sent is stale by the time the camera solves the frame,
    // so MegaTag2 answers get smeared. MegaTag1 ignores our heading, so it stays trustworthy.
    boolean spinningTooFast =
        Math.abs(drivetrain.getFieldVelocity().omega) > MAX_SPIN_FOR_MT2_RAD_PER_SEC;

    Estimate best = null;
    for (Estimate estimate : camera.estimates()) {
      if (!accept(estimate, spinningTooFast)) {
        continue;
      }
      // Prefer MegaTag1: it does not depend on a heading that may be stale.
      if (best == null || (best.megaTag2() && !estimate.megaTag2())) {
        best = estimate;
      }
    }

    Logger.recordOutput("Vision/Offered", camera.estimates().size());
    Logger.recordOutput("Vision/Accepted", best != null);
    if (best == null) {
      return;
    }

    double xy = standardDeviation(best);
    Logger.recordOutput("Vision/AcceptedPose", best.pose());
    Logger.recordOutput("Vision/AcceptedStdDevXY", xy);
    Logger.recordOutput("Vision/AcceptedIsMegaTag2", best.megaTag2());
    drivetrain.addVisionMeasurement(
        best.pose(), best.timestampSeconds(), VecBuilder.fill(xy, xy, HEADING_STD_DEV));
  }

  /** Our trust rules. Change these, replay an old log, and the answers change with them. */
  private boolean accept(Estimate estimate, boolean spinningTooFast) {
    if (!estimate.soundEnough() || estimate.pose().equals(new Pose2d())) {
      return false;
    }
    if (estimate.avgTagDistanceMeters() > MAX_TAG_DISTANCE_METERS) {
      return false;
    }
    if (estimate.megaTag2()) {
      return !spinningTooFast && estimate.tagCount() >= MT2_MIN_TAGS;
    }
    return estimate.tagCount() >= MT1_MIN_TAGS;
  }

  /** How much to distrust this estimate, in meters. Bigger = the estimator leans on it less. */
  private double standardDeviation(Estimate estimate) {
    double distance = Math.max(estimate.avgTagDistanceMeters(), 0.1);
    return XY_STD_DEV
        * Math.pow(distance, DISTANCE_EXPONENT)
        / Math.sqrt(Math.max(estimate.tagCount(), 1));
  }
}
