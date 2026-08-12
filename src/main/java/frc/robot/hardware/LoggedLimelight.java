// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.hardware;

import com.limelightvision.Limelight;
import com.limelightvision.Limelight.FiducialTarget;
import com.limelightvision.Limelight.IMUMode;
import com.limelightvision.Limelight.LimelightResults;
import com.limelightvision.Limelight.PoseEstimate;
import com.limelightvision.Limelight.PoseEstimateConfig;
import com.limelightvision.Limelight.PoseEstimateType;
import frc.robot.utils.RunMode;
import java.util.ArrayList;
import java.util.List;
import java.util.Optional;
import org.littletonrobotics.junction.AutoLog;
import org.littletonrobotics.junction.Logger;
import org.wpilib.math.geometry.Pose2d;
import org.wpilib.math.geometry.Pose3d;

/**
 * A Limelight whose whole frame goes through the log, so any vision decision can be replayed.
 *
 * <p>This is the Limelight class made loggable: everything the camera reports about AprilTags is an
 * input here, not just the pose estimate we happen to use today. That is deliberate - a formula you
 * write next season can reach for {@code robotPoseTargetSpace} or a tag's ambiguity on a log
 * recorded this season, without having had to predict that you would want it.
 *
 * <p>Three levels, each a set of parallel arrays. A frame produces one entry at frame level, two at
 * estimate level (MegaTag1 and MegaTag2 both solve the same frame), and one per tag it saw. {@code
 * frameIndex} ties them together.
 *
 * <p>What replay cannot redo is the MegaTag solve itself: that needs the raw image, which never
 * reaches the robot. Everything computed *from* the solve is ours to change and re-run.
 *
 * <p>There is no vision simulation: in sim these keys are logged empty, so a sim log has the vision
 * plumbing but no sightings. Replay a real-robot log to exercise the vision code.
 */
public class LoggedLimelight {
  /** One camera's frames for a single loop. */
  @AutoLog
  public static class CameraInputs {
    public boolean connected;

    // ---- frame level: one entry per frame received this loop ----
    public long[] frameIndices = new long[0];
    public double[] frameTimestampsSeconds = new double[0];
    public double[] txDegrees = new double[0];
    public double[] tyDegrees = new double[0];
    public double[] txDegreesNoCrosshair = new double[0];
    public double[] tyDegreesNoCrosshair = new double[0];
    public double[] targetAreaPercent = new double[0];
    public double[] targetDistanceMeters = new double[0];
    public int[] pipelineIndices = new int[0];
    public double[] captureLatencyMs = new double[0];
    public double[] targetingLatencyMs = new double[0];
    // The camera's own IMU. Its accelerations are a second opinion on the gyro's.
    public double[] imuYawDegrees = new double[0];
    // What we SENT the camera, and the offset it applied. If imuYaw tracks these exactly, the
    // camera's IMU is echoing our gyro rather than measuring independently - see setIMUMode.
    public double[] imuRobotYawDegrees = new double[0];
    public double[] imuYawOffsetDegrees = new double[0];
    public double[] imuPitchDegrees = new double[0];
    public double[] imuRollDegrees = new double[0];
    public double[] imuGyroZDegreesPerSecond = new double[0];
    public double[] imuAccelXG = new double[0];
    public double[] imuAccelYG = new double[0];
    public double[] imuAccelZG = new double[0];

    // ---- estimate level: two entries per frame (MegaTag1, then MegaTag2) ----
    public long[] estimateFrameIndices = new long[0];
    public Pose2d[] poses = new Pose2d[0];
    public double[] timestampsSeconds = new double[0];
    public double[] latencyMs = new double[0];
    public String[] estimateTypes = new String[0];
    public int[] tagCounts = new int[0]; // tags actually on the field map
    public int[] reportedTagCounts = new int[0]; // tags the camera claims to see
    public double[] tagSpanMeters = new double[0];
    public double[] avgTagDistancesMeters = new double[0];
    public double[] avgTagAreaPercent = new double[0];
    // The library's own trust numbers. Ours are computed in Vision from the values above.
    public double[] stdDevX = new double[0];
    public double[] stdDevY = new double[0];
    public double[] stdDevTheta = new double[0];
    public int[] rejectionFlags = new int[0];
    public boolean[] megaTag2 = new boolean[0];

    // ---- tag level: one entry per tag seen, per frame ----
    public long[] tagFrameIndices = new long[0];
    public int[] tagIds = new int[0];
    public double[] tagAmbiguity = new double[0];
    public boolean[] tagFielded = new boolean[0];
    public double[] tagTxDegrees = new double[0];
    public double[] tagTyDegrees = new double[0];
    public double[] tagAreaPercent = new double[0];
    public double[] tagDistanceToCameraMeters = new double[0];
    public double[] tagDistanceToRobotMeters = new double[0];
    public Pose3d[] robotPoseTargetSpace = new Pose3d[0];
    public Pose3d[] targetPoseRobotSpace = new Pose3d[0];
    public Pose3d[] targetPoseCameraSpace = new Pose3d[0];
    public Pose3d[] cameraPoseTargetSpace = new Pose3d[0];
    public Pose3d[] robotPoseFieldSpace = new Pose3d[0];
    public Pose3d[] robotPoseFieldSpaceMegaTag2 = new Pose3d[0];
  }

  /** One pose estimate, already pulled out of the log. */
  public record Estimate(
      Pose2d pose,
      double timestampSeconds,
      double latencyMs,
      String type,
      int tagCount,
      int reportedTagCount,
      double tagSpanMeters,
      double avgTagDistanceMeters,
      double avgTagAreaPercent,
      double stdDevX,
      double stdDevY,
      double stdDevTheta,
      int rejectionFlags,
      boolean megaTag2) {}

  /**
   * One AprilTag as the camera saw it, already pulled out of the log. The pose names match the
   * Limelight library's: {@code robotPoseTargetSpace} is the robot in the tag's frame (+X out of
   * the tag face), {@code targetPoseRobotSpace} is the tag in the robot's frame.
   */
  public record Tag(
      int id,
      double ambiguity,
      boolean fielded,
      double txDegrees,
      double tyDegrees,
      double areaPercent,
      double distanceToCameraMeters,
      double distanceToRobotMeters,
      Pose3d robotPoseTargetSpace,
      Pose3d targetPoseRobotSpace,
      Pose3d targetPoseCameraSpace,
      Pose3d cameraPoseTargetSpace,
      Pose3d robotPoseFieldSpace,
      Pose3d robotPoseFieldSpaceMegaTag2) {}

  /** The camera's onboard IMU for the newest frame. */
  public record Imu(
      double yawDegrees,
      double robotYawDegrees,
      double yawOffsetDegrees,
      double pitchDegrees,
      double rollDegrees,
      double gyroZDegreesPerSecond,
      double accelXG,
      double accelYG,
      double accelZG) {}

  private final Limelight camera;
  private final String name;
  private final String logKey;
  private final CameraInputsAutoLogged inputs = new CameraInputsAutoLogged();
  private final List<Estimate> estimates = new ArrayList<>();
  private final List<Tag> tags = new ArrayList<>();

  /**
   * @param name the camera's NetworkTables name, e.g. "limelight-br".
   */
  public LoggedLimelight(String name) {
    this.camera = new Limelight(name);
    this.name = name;
    this.logKey = "Hardware/Limelight/" + name;
  }

  /** This camera's name, e.g. "limelight-br". */
  public String name() {
    return name;
  }

  /**
   * Drains this loop's frames and hands them to the log. Must be called every loop, in the same
   * order every loop - skipping a call would put replay out of step with the recording.
   */
  public void refresh() {
    switch (RunMode.current()) {
      case REAL -> readFromCamera();
      // SIM reports nothing: no camera, no fake tag. The keys are still logged, just empty.
      case REPLAY, SIM -> {} // in replay the log already holds what the camera said
      default -> {}
    }
    Logger.processInputs(logKey, inputs);
    unpack();
  }

  // ---------------------------------------------------------------------------
  // Reads. Every one of these comes out of the log, so they all replay.
  // ---------------------------------------------------------------------------

  /** This loop's pose estimates, oldest first. Empty on loops with no new frame. */
  public List<Estimate> estimates() {
    return estimates;
  }

  /** Every tag seen this loop, oldest frame first. */
  public List<Tag> tags() {
    return tags;
  }

  /** The newest sighting of one tag ID, or empty if the camera did not see it this loop. */
  public Optional<Tag> tag(int id) {
    for (int i = tags.size() - 1; i >= 0; i--) {
      if (tags.get(i).id() == id) {
        return Optional.of(tags.get(i));
      }
    }
    return Optional.empty();
  }

  /** True if any tag was seen this loop. */
  public boolean hasTarget() {
    return !tags.isEmpty();
  }

  /** True if the camera is publishing. */
  public boolean isConnected() {
    return inputs.connected;
  }

  /** Crosshair-relative horizontal offset to the best target, newest frame. */
  public double getTXDegrees() {
    return newestFrame(inputs.txDegrees);
  }

  /** Crosshair-relative vertical offset to the best target, newest frame. */
  public double getTYDegrees() {
    return newestFrame(inputs.tyDegrees);
  }

  /** Horizontal offset ignoring the crosshair calibration, newest frame. */
  public double getTXDegreesNoCrosshair() {
    return newestFrame(inputs.txDegreesNoCrosshair);
  }

  /** Vertical offset ignoring the crosshair calibration, newest frame. */
  public double getTYDegreesNoCrosshair() {
    return newestFrame(inputs.tyDegreesNoCrosshair);
  }

  /** Fraction of the image the best target fills, newest frame. */
  public double getTargetAreaPercent() {
    return newestFrame(inputs.targetAreaPercent);
  }

  /** Distance to the best target, newest frame. */
  public double getTargetDistanceMeters() {
    return newestFrame(inputs.targetDistanceMeters);
  }

  /** Total latency - capture plus targeting - of the newest frame. */
  public double getLatencyMs() {
    return newestFrame(inputs.captureLatencyMs) + newestFrame(inputs.targetingLatencyMs);
  }

  /** The pipeline that produced the newest frame. */
  public int getCurrentPipelineIndex() {
    return inputs.pipelineIndices.length == 0
        ? -1
        : inputs.pipelineIndices[inputs.pipelineIndices.length - 1];
  }

  /** The camera's onboard IMU at the newest frame. */
  public Imu imu() {
    return new Imu(
        newestFrame(inputs.imuYawDegrees),
        newestFrame(inputs.imuRobotYawDegrees),
        newestFrame(inputs.imuYawOffsetDegrees),
        newestFrame(inputs.imuPitchDegrees),
        newestFrame(inputs.imuRollDegrees),
        newestFrame(inputs.imuGyroZDegreesPerSecond),
        newestFrame(inputs.imuAccelXG),
        newestFrame(inputs.imuAccelYG),
        newestFrame(inputs.imuAccelZG));
  }

  private static double newestFrame(double[] values) {
    return values.length == 0 ? 0.0 : values[values.length - 1];
  }

  // ---------------------------------------------------------------------------
  // Writes. No-ops during replay: there is no camera to tell.
  // ---------------------------------------------------------------------------

  /** Applies the pose-estimate gates and points the camera at our shared heading feed. */
  public void configure(PoseEstimateConfig megaTag1, PoseEstimateConfig megaTag2) {
    if (RunMode.current() != RunMode.REPLAY) {
      camera.withPoseEstimateConfig_MT1(megaTag1).withPoseEstimateConfig_MT2(megaTag2);
      camera.setUseSharedOrientation(true);
    }
  }

  /**
   * Picks where the camera's reported yaw comes from. Pin this in code: the default lives in the LL
   * web UI, and {@code EXTERNAL} / {@code INTERNAL_EXTERNAL_ASSIST} make the camera echo the
   * heading we send it - which silently turns any gyro cross-check into a comparison of our gyro
   * with itself. {@code INTERNAL} is the independent one.
   */
  public void setIMUMode(IMUMode mode) {
    if (RunMode.current() != RunMode.REPLAY) {
      camera.setIMUMode(mode);
    }
  }

  /** Tells the camera which tag to favour when several are in view. -1 clears it. */
  public void setPriorityTagID(int id) {
    if (RunMode.current() != RunMode.REPLAY) {
      camera.setPriorityTagID(id);
    }
  }

  /**
   * Sends our heading to every camera at once, which is what MegaTag2 needs to solve. Degrees,
   * CCW+.
   *
   * <p>Ends in a full NetworkTables flush, so call it once per loop and never from the fast
   * odometry thread.
   */
  public static void setSharedRobotOrientation(double yawDegrees, double yawRateDegreesPerSecond) {
    if (RunMode.current() != RunMode.REPLAY) {
      Limelight.setSharedRobotOrientation(yawDegrees, yawRateDegreesPerSecond, 0, 0, 0, 0);
    }
  }

  // ---------------------------------------------------------------------------
  // Input plumbing
  // ---------------------------------------------------------------------------

  /** Turns the logged arrays back into records, so callers never do index arithmetic. */
  private void unpack() {
    estimates.clear();
    for (int i = 0; i < inputs.poses.length; i++) {
      estimates.add(
          new Estimate(
              inputs.poses[i],
              inputs.timestampsSeconds[i],
              inputs.latencyMs[i],
              inputs.estimateTypes[i],
              inputs.tagCounts[i],
              inputs.reportedTagCounts[i],
              inputs.tagSpanMeters[i],
              inputs.avgTagDistancesMeters[i],
              inputs.avgTagAreaPercent[i],
              inputs.stdDevX[i],
              inputs.stdDevY[i],
              inputs.stdDevTheta[i],
              inputs.rejectionFlags[i],
              inputs.megaTag2[i]));
    }

    tags.clear();
    for (int i = 0; i < inputs.tagIds.length; i++) {
      tags.add(
          new Tag(
              inputs.tagIds[i],
              inputs.tagAmbiguity[i],
              inputs.tagFielded[i],
              inputs.tagTxDegrees[i],
              inputs.tagTyDegrees[i],
              inputs.tagAreaPercent[i],
              inputs.tagDistanceToCameraMeters[i],
              inputs.tagDistanceToRobotMeters[i],
              inputs.robotPoseTargetSpace[i],
              inputs.targetPoseRobotSpace[i],
              inputs.targetPoseCameraSpace[i],
              inputs.cameraPoseTargetSpace[i],
              inputs.robotPoseFieldSpace[i],
              inputs.robotPoseFieldSpaceMegaTag2[i]));
    }
  }

  private void readFromCamera() {
    LimelightResults[] frames = camera.readResultsQueue();
    int tagTotal = 0;
    for (LimelightResults frame : frames) {
      tagTotal += frame.fiducialTargets == null ? 0 : frame.fiducialTargets.length;
    }

    allocate(frames.length, frames.length * 2, tagTotal);
    inputs.connected = camera.isConnected();

    int tag = 0;
    for (int f = 0; f < frames.length; f++) {
      LimelightResults frame = frames[f];
      storeFrame(f, frame);

      // MegaTag1 and MegaTag2 solve the same frame; log both so replay can re-pick.
      storeEstimate(
          f * 2, frame.frameIndex, camera.getPoseEstimate(frame, PoseEstimateType.MT1_WPIBLUE));
      storeEstimate(
          f * 2 + 1, frame.frameIndex, camera.getPoseEstimate(frame, PoseEstimateType.MT2_WPIBLUE));

      if (frame.fiducialTargets != null) {
        for (FiducialTarget target : frame.fiducialTargets) {
          storeTag(tag++, frame.frameIndex, target);
        }
      }
    }
  }

  private void storeFrame(int i, LimelightResults frame) {
    inputs.frameIndices[i] = frame.frameIndex;
    inputs.frameTimestampsSeconds[i] = frame.timestamp;
    inputs.txDegrees[i] = frame.txDegrees;
    inputs.tyDegrees[i] = frame.tyDegrees;
    inputs.txDegreesNoCrosshair[i] = frame.txDegreesNoCrosshair;
    inputs.tyDegreesNoCrosshair[i] = frame.tyDegreesNoCrosshair;
    inputs.targetAreaPercent[i] = frame.targetAreaPercent;
    inputs.targetDistanceMeters[i] = frame.targetDistanceMeters;
    inputs.pipelineIndices[i] = (int) frame.pipelineIndex;
    inputs.captureLatencyMs[i] = frame.captureLatencyMs;
    inputs.targetingLatencyMs[i] = frame.targetingLatencyMs;
    if (frame.imu != null) {
      inputs.imuYawDegrees[i] = frame.imu.yaw;
      inputs.imuRobotYawDegrees[i] = frame.imu.robotYaw;
      inputs.imuYawOffsetDegrees[i] = frame.imu.yawOffset;
      inputs.imuPitchDegrees[i] = frame.imu.pitch;
      inputs.imuRollDegrees[i] = frame.imu.roll;
      inputs.imuGyroZDegreesPerSecond[i] = frame.imu.gyroZ;
      inputs.imuAccelXG[i] = frame.imu.accelX;
      inputs.imuAccelYG[i] = frame.imu.accelY;
      inputs.imuAccelZG[i] = frame.imu.accelZ;
    }
  }

  private void storeEstimate(int i, long frameIndex, PoseEstimate estimate) {
    inputs.estimateFrameIndices[i] = frameIndex;
    inputs.poses[i] = estimate.pose == null ? new Pose2d() : estimate.pose;
    inputs.timestampsSeconds[i] = estimate.timestampSeconds;
    inputs.latencyMs[i] = estimate.latencyMs;
    inputs.estimateTypes[i] = estimate.type == null ? "" : estimate.type.name();
    inputs.tagCounts[i] = estimate.fieldedTagCount;
    inputs.reportedTagCounts[i] = estimate.reportedTagCount;
    inputs.tagSpanMeters[i] = estimate.tagSpanMeters;
    inputs.avgTagDistancesMeters[i] = estimate.avgTagDistanceMeters;
    inputs.avgTagAreaPercent[i] = estimate.avgTagAreaPercent;
    if (estimate.stdDevs != null) {
      inputs.stdDevX[i] = estimate.stdDevs.get(0);
      inputs.stdDevY[i] = estimate.stdDevs.get(1);
      inputs.stdDevTheta[i] = estimate.stdDevs.get(2);
    }
    inputs.rejectionFlags[i] = estimate.rejectionFlags;
    inputs.megaTag2[i] = estimate.isMegaTag2();
  }

  private void storeTag(int i, long frameIndex, FiducialTarget target) {
    inputs.tagFrameIndices[i] = frameIndex;
    inputs.tagIds[i] = target.fiducialId;
    inputs.tagAmbiguity[i] = target.ambiguity;
    inputs.tagFielded[i] = target.fielded;
    inputs.tagTxDegrees[i] = target.txDegrees;
    inputs.tagTyDegrees[i] = target.tyDegrees;
    inputs.tagAreaPercent[i] = target.targetAreaPercent;
    inputs.tagDistanceToCameraMeters[i] = target.getDistanceToCamera();
    inputs.tagDistanceToRobotMeters[i] = target.getDistanceToRobot();
    // The library's own converters, so the degree/radian and axis conventions stay theirs.
    inputs.robotPoseTargetSpace[i] = target.getRobotPose_TargetSpace();
    inputs.targetPoseRobotSpace[i] = target.getTargetPose_RobotSpace();
    inputs.targetPoseCameraSpace[i] = target.getTargetPose_CameraSpace();
    inputs.cameraPoseTargetSpace[i] = target.getCameraPose_TargetSpace();
    inputs.robotPoseFieldSpace[i] = target.getRobotPose_FieldSpace();
    inputs.robotPoseFieldSpaceMegaTag2[i] = target.getRobotPose_FieldSpace_MegaTag2();
  }

  private void allocate(int frames, int estimateCount, int tagCount) {
    inputs.connected = false;

    inputs.frameIndices = new long[frames];
    inputs.frameTimestampsSeconds = new double[frames];
    inputs.txDegrees = new double[frames];
    inputs.tyDegrees = new double[frames];
    inputs.txDegreesNoCrosshair = new double[frames];
    inputs.tyDegreesNoCrosshair = new double[frames];
    inputs.targetAreaPercent = new double[frames];
    inputs.targetDistanceMeters = new double[frames];
    inputs.pipelineIndices = new int[frames];
    inputs.captureLatencyMs = new double[frames];
    inputs.targetingLatencyMs = new double[frames];
    inputs.imuYawDegrees = new double[frames];
    inputs.imuRobotYawDegrees = new double[frames];
    inputs.imuYawOffsetDegrees = new double[frames];
    inputs.imuPitchDegrees = new double[frames];
    inputs.imuRollDegrees = new double[frames];
    inputs.imuGyroZDegreesPerSecond = new double[frames];
    inputs.imuAccelXG = new double[frames];
    inputs.imuAccelYG = new double[frames];
    inputs.imuAccelZG = new double[frames];

    inputs.estimateFrameIndices = new long[estimateCount];
    inputs.poses = new Pose2d[estimateCount];
    inputs.timestampsSeconds = new double[estimateCount];
    inputs.latencyMs = new double[estimateCount];
    inputs.estimateTypes = new String[estimateCount];
    inputs.tagCounts = new int[estimateCount];
    inputs.reportedTagCounts = new int[estimateCount];
    inputs.tagSpanMeters = new double[estimateCount];
    inputs.avgTagDistancesMeters = new double[estimateCount];
    inputs.avgTagAreaPercent = new double[estimateCount];
    inputs.stdDevX = new double[estimateCount];
    inputs.stdDevY = new double[estimateCount];
    inputs.stdDevTheta = new double[estimateCount];
    inputs.rejectionFlags = new int[estimateCount];
    inputs.megaTag2 = new boolean[estimateCount];

    inputs.tagFrameIndices = new long[tagCount];
    inputs.tagIds = new int[tagCount];
    inputs.tagAmbiguity = new double[tagCount];
    inputs.tagFielded = new boolean[tagCount];
    inputs.tagTxDegrees = new double[tagCount];
    inputs.tagTyDegrees = new double[tagCount];
    inputs.tagAreaPercent = new double[tagCount];
    inputs.tagDistanceToCameraMeters = new double[tagCount];
    inputs.tagDistanceToRobotMeters = new double[tagCount];
    inputs.robotPoseTargetSpace = filled(tagCount);
    inputs.targetPoseRobotSpace = filled(tagCount);
    inputs.targetPoseCameraSpace = filled(tagCount);
    inputs.cameraPoseTargetSpace = filled(tagCount);
    inputs.robotPoseFieldSpace = filled(tagCount);
    inputs.robotPoseFieldSpaceMegaTag2 = filled(tagCount);
  }

  /** Struct logging cannot serialise a null pose, so unseen slots hold the origin. */
  private static Pose3d[] filled(int n) {
    var poses = new Pose3d[n];
    java.util.Arrays.fill(poses, Pose3d.kZero);
    return poses;
  }
}
