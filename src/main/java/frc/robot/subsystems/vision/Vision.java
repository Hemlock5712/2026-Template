// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.vision;

import com.limelightvision.Limelight;
import com.limelightvision.Limelight.LimelightResults;
import com.limelightvision.Limelight.PoseEstimate;
import com.limelightvision.Limelight.PoseEstimateConfig;
import com.limelightvision.Limelight.PoseEstimateType;
import frc.robot.subsystems.DriveMechanism;
import org.wpilib.command3.Scheduler;

/**
 * Feeds each Limelight's AprilTag pose estimates into the drivetrain's pose estimator. Call {@link
 * #registerAll} once from {@link frc.robot.Robot}.
 *
 * <p>The library filters estimates and computes their trust numbers; this class only picks which
 * estimate type to use per frame - MegaTag1 needs 2+ tags, MegaTag2 leans on the gyro heading so
 * one tag is enough (seed the gyro, or single-tag vision will be off). Vision only corrects x/y;
 * the gyro owns heading.
 *
 * <p>Does nothing in sim (no camera). Accepted/rejected poses show up in AdvantageScope under
 * {@code limelight_telemetry}.
 */
public class Vision {
  // How far away tags are still trusted; past this they are too noisy to help.
  private static final double MAX_TAG_DISTANCE_METERS = 4.0;

  // Trust numbers (smaller = trust vision more): xy = 0.15 * d^2 / sqrt(n). Error grows with
  // distance squared (farther tags look smaller); more tags average the noise down.
  private static final double XY_STD_DEV = 0.15;

  // Spin faster than this (1 full turn per second) and we stop trusting MegaTag2. See update().
  private static final double MAX_SPIN_FOR_MT2_RAD_PER_SEC = 2 * Math.PI;

  // MegaTag1 solves position from the tags alone - trustworthy with 2+ tags.
  private static final PoseEstimateConfig MT1_CONFIG =
      PoseEstimateConfig.defaultMT1()
          .withMinTagCount(2)
          .withMaxAvgTagDistance(MAX_TAG_DISTANCE_METERS)
          .withStdDevXY(XY_STD_DEV)
          .withStdDevDistanceScaling(2.0);

  // MegaTag2 leans on the gyro heading we feed it, so one tag is enough.
  private static final PoseEstimateConfig MT2_CONFIG =
      PoseEstimateConfig.defaultMT2()
          .withMaxAvgTagDistance(MAX_TAG_DISTANCE_METERS)
          .withStdDevXY(XY_STD_DEV)
          .withStdDevDistanceScaling(2.0);

  private final Limelight camera;
  private final DriveMechanism drivetrain;

  private Vision(Limelight camera, DriveMechanism drivetrain) {
    this.camera = camera;
    this.drivetrain = drivetrain;
  }

  /** Wires every camera: apply the trust configs and run each camera's update every loop. */
  public static void registerAll(DriveMechanism drivetrain, Limelight... cameras) {
    // MegaTag2 needs to know which way we're facing, so send every camera our heading (degrees,
    // CCW+). One call covers all of them - see setUseSharedOrientation below.
    //
    // ORDER MATTERS: these run in the order registered, so send the heading BEFORE reading the
    // cameras. And keep it at 50 Hz - this call ends in a full NetworkTables flush, which is far
    // too expensive for the fast odometry thread.
    Scheduler.getDefault()
        .addPeriodic(
            () ->
                Limelight.setSharedRobotOrientation(
                    drivetrain.getPose().getRotation().getDegrees(),
                    Math.toDegrees(drivetrain.getFieldVelocity().omega),
                    0,
                    0,
                    0,
                    0));

    for (Limelight camera : cameras) {
      camera.withPoseEstimateConfig_MT1(MT1_CONFIG).withPoseEstimateConfig_MT2(MT2_CONFIG);
      camera.setUseSharedOrientation(true); // heading comes from the shared feed above
      Vision vision = new Vision(camera, drivetrain);
      Scheduler.getDefault().addPeriodic(vision::update);
    }
  }

  /** Runs one vision update: every new camera frame becomes one pose-estimator measurement. */
  private void update() {
    // Spinning fast means the heading we sent is stale by the time the camera solves the frame,
    // so MegaTag2 answers get smeared. MegaTag1 ignores our heading, so it stays trustworthy.
    boolean spinningTooFast =
        Math.abs(drivetrain.getFieldVelocity().omega) > MAX_SPIN_FOR_MT2_RAD_PER_SEC;

    for (LimelightResults frame : camera.readResultsQueue()) {
      // Try MegaTag1 first (2+ tags); fall back to MegaTag2 for a lone tag.
      PoseEstimate estimate = camera.getPoseEstimate(frame, PoseEstimateType.MT1_WPIBLUE);
      if (!estimate.isValid() && !spinningTooFast) {
        estimate = camera.getPoseEstimate(frame, PoseEstimateType.MT2_WPIBLUE);
      }
      if (estimate.isValid()) {
        drivetrain.addVisionMeasurement(estimate.pose, estimate.timestampSeconds, estimate.stdDevs);
      }
    }
  }
}
