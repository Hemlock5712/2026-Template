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
 * type to trust per frame - MegaTag1 (vision heading) needs 2+ tags, MegaTag2 (gyro heading) works
 * with a lone tag - so seed the gyro or single-tag vision will be off. The robot heading MegaTag2
 * needs is fed to every camera at odometry rate by {@link DriveMechanism}, via the shared {@code
 * limelightshared} table.
 *
 * <p>Does nothing in sim (no camera). The library publishes accepted/rejected pose telemetry to the
 * {@code limelight_telemetry} NT table automatically - view it in AdvantageScope.
 */
public class Vision {
  // How far away tags are still trusted; past this they are too noisy to help.
  private static final double MAX_TAG_DISTANCE_METERS = 4.0;

  /*
   * How the library turns a config into an estimate's std devs (the "trust numbers" the pose
   * estimator wants - SMALLER = trust vision MORE). Per accepted estimate it computes:
   *
   *   scale = d ^ distanceExponent / n ^ tagCountExponent
   *
   *   xy    = clamp(baseXY    * scale, 0.0001 m, max)    [meters]
   *   theta = clamp(baseTheta * scale, 0.01 rad, max)    [radians]
   *
   * where d = average distance to the tags (meters) and n = how many field-mapped tags it saw.
   * The default exponents are 1 and 0.5, so trust falls off linearly with distance and improves
   * with sqrt(tag count). With the two configs below that works out to:
   *
   *   MT1 (2+ tags):  xy = 0.5 * d / sqrt(n)     theta = 1.5 * d / sqrt(n)
   *   MT2 (any tag):  xy = 0.3 * d / sqrt(n)     theta = 9999999 (gyro owns heading)
   *
   * The knobs: withStdDevXY / withStdDevTheta set the base (and optionally the clamps),
   * withStdDevDistanceScaling sets the distance exponent, withStdDevTagCountDivision the tag-count
   * exponent. An estimate with no distance data comes back untrusted (9999999) on all three axes.
   */

  // MegaTag1 solves position AND heading from the tags alone - only trustworthy with 2+ tags.
  // withStdDevTheta sets how much to trust that heading (the library default is "not at all").
  private static final PoseEstimateConfig MT1_CONFIG =
      PoseEstimateConfig.defaultMT1()
          .withMinTagCount(2)
          .withMaxAvgTagDistance(MAX_TAG_DISTANCE_METERS)
          .withStdDevTheta(1.5);

  // MegaTag2 leans on the gyro heading we feed it, so a single tag is enough, and its heading
  // output stays untrusted - the gyro owns heading.
  private static final PoseEstimateConfig MT2_CONFIG =
      PoseEstimateConfig.defaultMT2().withMaxAvgTagDistance(MAX_TAG_DISTANCE_METERS);

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
