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
 * #registerAll} once from {@link frc.robot.Robot} with the camera objects it owns.
 *
 * <p>LimelightLib does the heavy lifting: per {@link PoseEstimateConfig} it rejects estimates that
 * fail the filters and computes distance/tag-count-scaled standard deviations, so an accepted
 * estimate drops straight into {@code addVisionMeasurement}. This class only picks which estimate
 * type to trust per frame - MegaTag1 (solves from the tags alone) needs 2+ tags, MegaTag2 (leans on
 * the gyro heading) works with a lone tag - so seed the gyro or single-tag vision will be off.
 * Vision only ever corrects x/y; the gyro owns heading. The robot heading MegaTag2 needs is fed to
 * every camera at odometry rate by {@link DriveMechanism}, via the shared {@code limelightshared}
 * table.
 *
 * <p>Does nothing in sim (no camera). The library publishes accepted/rejected pose telemetry to the
 * {@code limelight_telemetry} NT table automatically - view it in AdvantageScope.
 */
public class Vision {
  // How far away tags are still trusted; past this they are too noisy to help.
  private static final double MAX_TAG_DISTANCE_METERS = 4.0;

  /*
   * The library computes each estimate's std devs - the "trust numbers" the pose estimator wants
   * (smaller = trust vision more) - for both estimate types as:
   *
   *   xy    = 0.15 * d^2 / sqrt(n)     [d = average tag distance (m), n = tag count]
   *   theta = untrusted (library default) - the gyro owns heading, so seed it correctly.
   *
   * Distance is SQUARED because the camera ranges off the tag's apparent size: corner noise is
   * constant in pixels, so the range error a pixel causes grows with distance squared. More tags
   * average that noise down by sqrt(n). The base 0.15 is simply the std dev at 1 m with 1 tag.
   */
  private static final double XY_STD_DEV = 0.15;

  // MegaTag1 solves position from the tags alone (no heading needed) - trustworthy with 2+ tags.
  private static final PoseEstimateConfig MT1_CONFIG =
      PoseEstimateConfig.defaultMT1()
          .withMinTagCount(2)
          .withMaxAvgTagDistance(MAX_TAG_DISTANCE_METERS)
          .withStdDevXY(XY_STD_DEV)
          .withStdDevDistanceScaling(2.0);

  // MegaTag2 leans on the gyro heading we feed it, so a single tag is enough, and its heading
  // output stays untrusted - the gyro owns heading.
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
    for (Limelight camera : cameras) {
      camera.withPoseEstimateConfig_MT1(MT1_CONFIG).withPoseEstimateConfig_MT2(MT2_CONFIG);
      camera.setUseSharedOrientation(true); // heading comes from DriveMechanism's shared feed
      Vision vision = new Vision(camera, drivetrain);
      Scheduler.getDefault().addPeriodic(vision::update);
    }
  }

  /** Runs one vision update: every new camera frame becomes one pose-estimator measurement. */
  private void update() {
    for (LimelightResults frame : camera.readResultsQueue()) {
      // MegaTag1 when the frame saw 2+ tags; otherwise fall back to MegaTag2 for the lone tag.
      PoseEstimate estimate = camera.getPoseEstimate(frame, PoseEstimateType.MT1_WPIBLUE);
      if (!estimate.isValid()) {
        estimate = camera.getPoseEstimate(frame, PoseEstimateType.MT2_WPIBLUE);
      }
      if (estimate.isValid()) {
        drivetrain.addVisionMeasurement(estimate.pose, estimate.timestampSeconds, estimate.stdDevs);
      }
    }
  }
}
