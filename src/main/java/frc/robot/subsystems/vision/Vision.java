// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.vision;

import com.limelightvision.Limelight.PoseEstimateConfig;
import frc.robot.Robot;
import frc.robot.hardware.LoggedLimelight;
import frc.robot.hardware.LoggedLimelight.Estimate;
import frc.robot.subsystems.DriveMechanism;
import org.littletonrobotics.junction.Logger;
import org.wpilib.command3.Scheduler;
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
  // Two or more tags: MegaTag1 solves from the tag corners alone, so its heading owes nothing to
  // the gyro and we let it correct ours. One tag: MegaTag1 is ambiguous, so use MegaTag2 - but its
  // heading IS the gyro heading we sent the camera, so feeding it back would count it twice.
  private static final int MIN_TAGS_FOR_MEGATAG1 = 2;

  // Trust numbers (smaller = trust vision more): base * d^2 / sqrt(n). Error grows with distance
  // squared (farther tags look smaller); more tags average the noise down. TODO: tune both bases.
  private static final double XY_STD_DEV = 0.15; // meters, at 1 m off one tag
  private static final double OMEGA_STD_DEV =
      0.5; // radians - heading is trusted less than position
  private static final double DISTANCE_EXPONENT = 2.0;

  // Hand this to the estimator and it ignores that axis completely.
  private static final double UNTRUSTED = Double.MAX_VALUE;

  // Past this a tag is too small to measure well; spin faster than this and the frame is smeared.
  private static final double MAX_TAG_DISTANCE_METERS = 4.0;
  private static final double MAX_SPIN_RAD_PER_SEC = 2 * Math.PI;

  // Send the heading at 50 Hz however fast the robot loop runs. Rounds to 1 on a 50 Hz loop.
  private static final int HEADING_BROADCAST_DIVIDER =
      Math.max(1, (int) Math.round(0.02 / Robot.PERIOD_SECONDS));

  private static final PoseEstimateConfig PERMISSIVE_MT1 =
      PoseEstimateConfig.defaultMT1().withMinTagCount(1).withMaxAvgTagDistance(Double.MAX_VALUE);
  private static final PoseEstimateConfig PERMISSIVE_MT2 =
      PoseEstimateConfig.defaultMT2().withMinTagCount(1).withMaxAvgTagDistance(Double.MAX_VALUE);

  private final LoggedLimelight camera;
  private final DriveMechanism drivetrain;
  // Per camera, or two cameras would overwrite each other's numbers every loop.
  private final String logKey;

  private Vision(LoggedLimelight camera, DriveMechanism drivetrain) {
    this.camera = camera;
    this.drivetrain = drivetrain;
    this.logKey = "Vision/" + camera.name();
  }

  /** Wires every camera: relax the library's gates and run each camera's update every loop. */
  public static void registerAll(DriveMechanism drivetrain, LoggedLimelight... cameras) {
    // MegaTag2 needs to know which way we're facing, so send every camera our heading (degrees,
    // CCW+). One call covers all of them - see setUseSharedOrientation below.
    //
    // ORDER MATTERS: these run in the order registered, so send the heading BEFORE reading the
    // cameras. Held to 50 Hz by the divider: this call ends in a full NetworkTables flush, which is
    // far too expensive to do every loop, and a camera solving at 30 fps cannot use it any faster.
    Scheduler.getDefault()
        .addPeriodic(
            new Runnable() {
              private int loop = 0;

              @Override
              public void run() {
                if (loop++ % HEADING_BROADCAST_DIVIDER != 0) {
                  return;
                }
                LoggedLimelight.setSharedRobotOrientation(
                    drivetrain.getPose().getRotation().getDegrees(),
                    Math.toDegrees(drivetrain.getFieldVelocity().omega));
              }
            });

    for (LoggedLimelight camera : cameras) {
      // heading comes from the shared feed above
      camera.configure(PERMISSIVE_MT1, PERMISSIVE_MT2);
      Vision vision = new Vision(camera, drivetrain);
      Scheduler.getDefault().addPeriodic(vision::update);
    }
  }

  /** Runs one vision update: every new camera frame becomes at most one pose measurement. */
  private void update() {
    // Always, even if nothing is used from it - the log has one entry per loop either way.
    camera.refresh();

    // A fast spin smears the image and staleness the heading we sent, so throw the frame away.
    // This drops MegaTag1 too, which costs us a gyro-free heading exactly when the gyro is working
    // hardest - move the check into accept()'s MegaTag2 branch if you would rather keep it.
    boolean spinningTooFast = Math.abs(drivetrain.getFieldVelocity().omega) > MAX_SPIN_RAD_PER_SEC;

    int accepted = 0;
    for (Estimate estimate : camera.estimates()) {
      if (!accept(estimate, spinningTooFast)) {
        continue;
      }
      accepted++;
      double xy = deviation(XY_STD_DEV, estimate);
      // MegaTag2's heading came from us, so only MegaTag1 is allowed to move ours.
      double omega = estimate.megaTag2() ? UNTRUSTED : deviation(OMEGA_STD_DEV, estimate);

      Logger.recordOutput(logKey + "/AcceptedPose", estimate.pose());
      Logger.recordOutput(logKey + "/AcceptedStdDevXY", xy);
      Logger.recordOutput(logKey + "/AcceptedStdDevOmega", omega);
      Logger.recordOutput(logKey + "/AcceptedIsMegaTag2", estimate.megaTag2());
      drivetrain.addVisionMeasurement(
          estimate.pose(), estimate.timestampSeconds(), VecBuilder.fill(xy, xy, omega));
    }
    Logger.recordOutput(logKey + "/Offered", camera.estimates().size());
    Logger.recordOutput(logKey + "/Accepted", accepted);
  }

  /**
   * Our trust rules. Change these, replay an old log, and the answers change with them.
   *
   * <p>{@code rejectionFlags} is the library's own verdict, but only on things we could never
   * re-derive from the log - a non-finite solve, a missing timestamp, a pose off the field. Its
   * tunable gates are left switched off in {@code PERMISSIVE_*} on purpose: they run at record
   * time, so leaning on them would bake this decision into the log instead of leaving it
   * re-runnable.
   */
  private boolean accept(Estimate estimate, boolean spinningTooFast) {
    boolean wantMegaTag2 = estimate.tagCount() < MIN_TAGS_FOR_MEGATAG1;
    return estimate.megaTag2() == wantMegaTag2
        && estimate.rejectionFlags() == 0
        && estimate.tagCount() > 0
        && estimate.avgTagDistanceMeters() <= MAX_TAG_DISTANCE_METERS
        && !spinningTooFast;
  }

  /** One trust number, from a base value scaled by how far the tags are and how many there are. */
  private static double deviation(double base, Estimate estimate) {
    double distance = Math.max(estimate.avgTagDistanceMeters(), 0.1);
    return base * Math.pow(distance, DISTANCE_EXPONENT) / Math.sqrt(estimate.tagCount());
  }
}
