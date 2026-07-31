// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.hardware;

import com.limelightvision.Limelight;
import com.limelightvision.Limelight.LimelightResults;
import com.limelightvision.Limelight.PoseEstimate;
import com.limelightvision.Limelight.PoseEstimateType;
import frc.robot.utils.RunMode;
import java.util.ArrayList;
import java.util.List;
import java.util.Random;
import java.util.function.Supplier;
import org.littletonrobotics.junction.AutoLog;
import org.littletonrobotics.junction.Logger;
import org.wpilib.math.geometry.Pose2d;
import org.wpilib.math.geometry.Rotation2d;
import org.wpilib.math.geometry.Translation2d;

/**
 * A Limelight whose pose estimates go through the log, so vision decisions can be replayed.
 *
 * <p>Every camera frame produces two estimates - MegaTag1 (tags only) and MegaTag2 (leans on our
 * gyro heading) - and both are logged, so replay can re-decide which one to believe. What replay
 * cannot redo is the MegaTag solve itself: that needs the raw image, which never reaches the robot.
 *
 * <p>The library is configured to reject only structurally broken frames (see {@link
 * frc.robot.subsystems.vision.Vision}); every tuning decision lives in our code so it can be
 * changed and re-run against an old log.
 */
public class LoggedLimelight {
  /** One camera's estimates for a single loop. Parallel arrays - entry i is one estimate. */
  @AutoLog
  public static class CameraInputs {
    public boolean connected;
    public Pose2d[] poses = new Pose2d[0];
    public double[] timestampsSeconds = new double[0];
    public int[] tagCounts = new int[0];
    public double[] avgTagDistancesMeters = new double[0];
    public boolean[] megaTag2 = new boolean[0];
    public boolean[] soundEnough = new boolean[0];
  }

  /** One pose estimate, already pulled out of the log. */
  public record Estimate(
      Pose2d pose,
      double timestampSeconds,
      int tagCount,
      double avgTagDistanceMeters,
      boolean megaTag2,
      boolean soundEnough) {}

  private final Limelight camera;
  private final String name;
  private final String logKey;
  private final CameraInputsAutoLogged inputs = new CameraInputsAutoLogged();
  private final List<Estimate> estimates = new ArrayList<>();

  // Fixed seed: a sim run repeats exactly, which makes a replay comparison meaningful.
  private final Random noise;

  /**
   * @param name the camera's NetworkTables name, e.g. "limelight-br".
   */
  public LoggedLimelight(String name) {
    this.camera = new Limelight(name);
    this.name = name;
    this.logKey = "Hardware/Limelight/" + name;
    this.noise = new Random(name.hashCode());
  }

  /**
   * The wrapped camera, for configuration and target-space reads. Anything read here comes from
   * NetworkTables, not the log, so it does NOT replay - see {@link frc.robot.commands.DriveToTag}.
   */
  /** This camera's name, e.g. "limelight-br". */
  public String name() {
    return name;
  }

  public Limelight camera() {
    return camera;
  }

  /**
   * Drains this loop's frames and hands them to the log. Must be called every loop, in the same
   * order every loop - skipping a call would put replay out of step with the recording.
   */
  public void refresh() {
    switch (RunMode.current()) {
      case REAL -> readFromCamera();
      case SIM -> simulateFrame();
      case REPLAY -> {} // the log already holds what the camera said
      default -> {}
    }
    Logger.processInputs(logKey, inputs);

    estimates.clear();
    for (int i = 0; i < inputs.poses.length; i++) {
      estimates.add(
          new Estimate(
              inputs.poses[i],
              inputs.timestampsSeconds[i],
              inputs.tagCounts[i],
              inputs.avgTagDistancesMeters[i],
              inputs.megaTag2[i],
              inputs.soundEnough[i]));
    }
  }

  /** This loop's estimates, oldest first. Empty on loops with no new frame. */
  public List<Estimate> estimates() {
    return estimates;
  }

  /** True if the camera is publishing. */
  public boolean isConnected() {
    return inputs.connected;
  }

  // ---------------------------------------------------------------------------
  // Simulation only: a pretend AprilTag, so vision code has something to chew on
  // with no camera plugged in.
  // ---------------------------------------------------------------------------

  // One imaginary tag on the field, and how far away it can still be seen.
  private static final Translation2d SIM_TAG = new Translation2d(3.0, 0.0);
  private static final double SIM_MAX_SIGHT_METERS = 6.0;

  // Closer than this and the camera makes out a second tag, so MegaTag1 becomes usable.
  private static final double SIM_TWO_TAG_METERS = 2.5;

  // Error grows with distance: 2 cm per metre of range.
  private static final double SIM_NOISE_PER_METER = 0.02;

  // How stale a solved frame is by the time we read it.
  private static final double SIM_LATENCY_SECONDS = 0.03;

  private static Supplier<Pose2d> simPose = null;

  /** Tells the sim cameras where the robot really is. Called by Vision in simulation. */
  public static void setSimPoseSource(Supplier<Pose2d> trueRobotPose) {
    simPose = trueRobotPose;
  }

  private void simulateFrame() {
    if (simPose == null) {
      clearInputs(0);
      return;
    }
    Pose2d truth = simPose.get();
    double distance = truth.getTranslation().getDistance(SIM_TAG);
    if (distance > SIM_MAX_SIGHT_METERS) {
      clearInputs(0);
      inputs.connected = true;
      return;
    }

    int tagCount = distance < SIM_TWO_TAG_METERS ? 2 : 1;
    double sigma = SIM_NOISE_PER_METER * distance;

    // MegaTag1 and MegaTag2 both see the same frame, so they share its noise draw.
    Pose2d seen =
        new Pose2d(
            truth.getX() + noise.nextGaussian() * sigma,
            truth.getY() + noise.nextGaussian() * sigma,
            Rotation2d.fromRadians(
                truth.getRotation().getRadians() + noise.nextGaussian() * sigma * 0.1));

    clearInputs(2);
    inputs.connected = true;
    for (int i = 0; i < 2; i++) {
      inputs.poses[i] = seen;
      inputs.timestampsSeconds[i] = Logger.getTimestamp() / 1.0e6 - SIM_LATENCY_SECONDS;
      inputs.tagCounts[i] = tagCount;
      inputs.avgTagDistancesMeters[i] = distance;
      inputs.megaTag2[i] = i == 1;
      inputs.soundEnough[i] = true;
    }
  }

  private void clearInputs(int n) {
    inputs.connected = false;
    inputs.poses = new Pose2d[n];
    inputs.timestampsSeconds = new double[n];
    inputs.tagCounts = new int[n];
    inputs.avgTagDistancesMeters = new double[n];
    inputs.megaTag2 = new boolean[n];
    inputs.soundEnough = new boolean[n];
  }

  private void readFromCamera() {
    List<PoseEstimate> found = new ArrayList<>();
    for (LimelightResults frame : camera.readResultsQueue()) {
      found.add(camera.getPoseEstimate(frame, PoseEstimateType.MT1_WPIBLUE));
      found.add(camera.getPoseEstimate(frame, PoseEstimateType.MT2_WPIBLUE));
    }

    clearInputs(found.size());
    inputs.connected = camera.isConnected();
    int n = found.size();
    for (int i = 0; i < n; i++) {
      PoseEstimate estimate = found.get(i);
      inputs.poses[i] = estimate.pose == null ? new Pose2d() : estimate.pose;
      inputs.timestampsSeconds[i] = estimate.timestampSeconds;
      inputs.tagCounts[i] = estimate.fieldedTagCount;
      inputs.avgTagDistancesMeters[i] = estimate.avgTagDistanceMeters;
      inputs.megaTag2[i] = estimate.isMegaTag2();
      // The library only rejects broken frames now, so this means "the maths came out finite",
      // not "worth trusting" - that call is ours to make.
      inputs.soundEnough[i] = estimate.isValid();
    }
  }
}
