// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.hardware;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.swerve.SwerveRequest;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.utils.AccelerationLimiter;
import frc.robot.utils.Motor;
import frc.robot.utils.RunMode;
import frc.robot.utils.SkidDetector;
import java.util.ArrayList;
import java.util.concurrent.ArrayBlockingQueue;
import org.littletonrobotics.junction.AutoLog;
import org.littletonrobotics.junction.Logger;
import org.wpilib.math.estimator.SwerveDrivePoseEstimator;
import org.wpilib.math.geometry.Pose2d;
import org.wpilib.math.geometry.Rotation2d;
import org.wpilib.math.kinematics.ChassisVelocities;
import org.wpilib.math.kinematics.SwerveModulePosition;
import org.wpilib.math.kinematics.SwerveModuleVelocity;
import org.wpilib.math.linalg.Matrix;
import org.wpilib.math.numbers.N1;
import org.wpilib.math.numbers.N3;
import org.wpilib.system.RobotController;

/**
 * The swerve drivetrain as a single logged device: its whole state goes through the log, so
 * everything downstream - autos, drive-to-pose, alignment - replays.
 *
 * <p>The twelve swerve devices are not wrapped individually. CTRE runs them on its own 250 Hz
 * odometry thread inside the drivetrain, which this code cannot get between, so CTRE's odometry
 * *answer* ({@link #getPose}) is an input here rather than the sensors behind it. Every wheel
 * position and heading it integrated is logged too, which is what lets {@link #getEstimatedPose}
 * recompute a pose that does replay.
 *
 * <p>This exposes the same methods the drivetrain does and handles replay internally, so nothing
 * downstream branches on {@code RunMode} - swapping the type in is the whole change.
 */
public class LoggedSwerveDrivetrain implements LoggedHardware.Device {
  /** The drivetrain's state for a single loop. */
  @AutoLog
  public static class SwerveInputs {
    public Pose2d pose = new Pose2d();
    public ChassisVelocities velocity = new ChassisVelocities();
    public Rotation2d rawHeading = new Rotation2d();
    public SwerveModuleVelocity[] moduleVelocities = new SwerveModuleVelocity[0];
    public SwerveModuleVelocity[] moduleTargets = new SwerveModuleVelocity[0];
    public SwerveModulePosition[] modulePositions = new SwerveModulePosition[0];
    public double odometryPeriodSeconds;
    // When the modules above were last sampled, in the WPILib timebase.
    public double timestampSeconds;

    // Every odometry sample since the last loop - about five at 250 Hz - so code in the main loop
    // sees all of them instead of only the newest. Positions are flattened: four per timestamp.
    public double[] sampleTimestamps = new double[0];
    public Rotation2d[] sampleHeadings = new Rotation2d[0];
    public SwerveModulePosition[] samplePositions = new SwerveModulePosition[0];
  }

  /** One odometry sample, copied off the 250 Hz thread. */
  private record Sample(double timestamp, Rotation2d heading, SwerveModulePosition[] positions) {}

  // 200 ms of buffer. offer() drops when full rather than growing without bound if the loop stalls.
  private final ArrayBlockingQueue<Sample> samples = new ArrayBlockingQueue<>(50);

  private final CommandSwerveDrivetrain drivetrain;
  private final SwerveInputsAutoLogged inputs = new SwerveInputsAutoLogged();

  // Built on the first loop, not in the constructor: no module data exists until the first refresh.
  private SwerveDrivePoseEstimator estimator;

  // What we asked for last loop, FIELD-relative. The limiter ramps from this, not from the measured
  // velocity, so the answer is the same on the robot and in replay. Field-relative because holding
  // a
  // steady field velocity while the robot spins is not acceleration, and must not be limited as if
  // it were.
  private ChassisVelocities lastCommanded = new ChassisVelocities();

  private AccelerationLimiter.Config limiterConfig;

  // When the limiter last advanced, so it moves with the clock rather than once per call.
  private double lastLimitSeconds;

  public LoggedSwerveDrivetrain(CommandSwerveDrivetrain drivetrain) {
    this.drivetrain = drivetrain;
    // No signals to batch: CTRE reads the modules itself on its odometry thread.
    LoggedHardware.register(this, "Drivetrain");

    // Not in replay: the samples come from the log, and nothing would ever drain this queue.
    if (RunMode.current() != RunMode.REPLAY) {
      // Runs on CTRE's odometry thread, holding its state lock. Copy and leave - CTRE warns that
      // slow work here degrades odometry, and AdvantageKit's logger is not thread-safe.
      drivetrain.registerTelemetry(
          state ->
              samples.offer(
                  new Sample(state.Timestamp, state.RawHeading, state.ModulePositions.clone())));
    }
  }

  /** Field pose from odometry, blue-origin (the origin never flips with alliance). */
  public Pose2d getPose() {
    return inputs.pose;
  }

  /** Velocity in the robot frame. */
  public ChassisVelocities getVelocity() {
    return inputs.velocity;
  }

  /** Velocity rotated into the field frame. */
  public ChassisVelocities getFieldVelocity() {
    return inputs.velocity.toFieldRelative(inputs.pose.getRotation());
  }

  /** Gyro heading before any pose reset or vision correction. */
  public Rotation2d getRawHeading() {
    return inputs.rawHeading;
  }

  /** How far each wheel has driven and where it points - what odometry integrates. */
  public SwerveModulePosition[] getModulePositions() {
    return inputs.modulePositions;
  }

  /** Measured speed and angle of each module. */
  public SwerveModuleVelocity[] getModuleVelocities() {
    return inputs.moduleVelocities;
  }

  /** When the module data above was sampled, in the WPILib timebase. */
  public double getTimestampSeconds() {
    return inputs.timestampSeconds;
  }

  @Override
  public BaseStatusSignal[] signals() {
    return new BaseStatusSignal[0];
  }

  @Override
  public void updateInputs() {
    // getStateCopy, not getState: getState hands back CTRE's one shared state object, whose module
    // arrays are refilled by the next caller. We hold these until the end of the loop.
    var state = drivetrain.getStateCopy();
    inputs.pose = state.Pose;
    inputs.velocity = state.Velocity;
    inputs.rawHeading = state.RawHeading;
    inputs.moduleVelocities = state.ModuleVelocities;
    inputs.moduleTargets = state.ModuleTargets;
    inputs.modulePositions = state.ModulePositions;
    inputs.odometryPeriodSeconds = state.OdometryPeriod;
    inputs.timestampSeconds = state.Timestamp;

    drainSamples(state.ModulePositions.length);
  }

  /** Empties the 250 Hz queue into the flat input arrays. */
  private void drainSamples(int moduleCount) {
    var drained = new ArrayList<Sample>(samples.size());
    samples.drainTo(drained);

    inputs.sampleTimestamps = new double[drained.size()];
    inputs.sampleHeadings = new Rotation2d[drained.size()];
    inputs.samplePositions = new SwerveModulePosition[drained.size() * moduleCount];
    for (int i = 0; i < drained.size(); i++) {
      Sample sample = drained.get(i);
      inputs.sampleTimestamps[i] = sample.timestamp();
      inputs.sampleHeadings[i] = sample.heading();
      System.arraycopy(sample.positions(), 0, inputs.samplePositions, i * moduleCount, moduleCount);
    }
  }

  /** The wheel positions of one drained sample. */
  private SwerveModulePosition[] samplePositions(int index, int moduleCount) {
    var positions = new SwerveModulePosition[moduleCount];
    System.arraycopy(inputs.samplePositions, index * moduleCount, positions, 0, moduleCount);
    return positions;
  }

  /**
   * Re-integrates the odometry samples through a WPILib estimator. Called from {@link #logInputs},
   * so it runs off the log during replay and off CAN otherwise - identical either way.
   */
  private void updateEstimate() {
    int moduleCount = drivetrain.getModuleLocations().length;
    if (inputs.modulePositions.length != moduleCount) {
      return; // before the first refresh, or a partial odometry sample
    }
    if (estimator == null) {
      estimator =
          new SwerveDrivePoseEstimator(
              drivetrain.getKinematics(), inputs.rawHeading, inputs.modulePositions, inputs.pose);
    }

    // Every sample, not just the newest, so this sees what CTRE's own odometry saw. A log recorded
    // before sample capture existed has none - fall back to the single reading.
    if (inputs.sampleTimestamps.length == 0) {
      estimator.updateWithTime(inputs.timestampSeconds, inputs.rawHeading, inputs.modulePositions);
    } else {
      for (int i = 0; i < inputs.sampleTimestamps.length; i++) {
        estimator.updateWithTime(
            inputs.sampleTimestamps[i], inputs.sampleHeadings[i], samplePositions(i, moduleCount));
      }
    }
  }

  /**
   * Our own pose estimate, re-integrated from the logged wheel positions. Unlike {@link #getPose}
   * it recomputes during replay. Falls back to CTRE's until the first loop has run.
   */
  public Pose2d getEstimatedPose() {
    return estimator == null ? inputs.pose : estimator.getEstimatedPosition();
  }

  /**
   * Corrects the pose with a vision measurement. Our estimator is corrected even during replay - it
   * is ours, so a changed trust number moves {@code Drivetrain/EstimatedPose}. CTRE's native
   * estimator is not running then, so it is skipped.
   */
  public void addVisionMeasurement(
      Pose2d visionRobotPose, double timestampSeconds, Matrix<N3, N1> stdDevs) {
    if (estimator != null) {
      estimator.addVisionMeasurement(visionRobotPose, timestampSeconds, stdDevs);
    }
    if (RunMode.current() != RunMode.REPLAY) {
      drivetrain.addVisionMeasurement(visionRobotPose, timestampSeconds, stdDevs);
    }
  }

  /**
   * Sends a control request to the drivetrain. Logged either way, so a replay shows what the code
   * decided even though there is no drivetrain to send it to.
   */
  public void setControl(SwerveRequest request) {
    // Everything is limited in the field frame, because that is the one that is not turning.
    // Requests that speak robot-relative are rotated in and back out again.
    Rotation2d heading = inputs.pose.getRotation();
    boolean robotRelative = isRobotRelative(request);
    ChassisVelocities wanted = commandedVelocity(request);
    ChassisVelocities wantedField = robotRelative ? wanted.toFieldRelative(heading) : wanted;

    // Real elapsed time, not the nominal period. Two things need this: the loop jitters a little,
    // and a command handover calls setControl twice in one cycle (the old command's Idle, then the
    // new command's request). Charging both calls a full period would let the ramp advance twice.
    double now = RobotController.getTime() / 1.0e6;
    double dt = now - lastLimitSeconds;
    lastLimitSeconds = now;

    ChassisVelocities nextField =
        AccelerationLimiter.limit(lastCommanded, wantedField, heading, dt, limiterConfig());
    // Command the average across the interval; carry its END as the ramp's state.
    ChassisVelocities allowedField = AccelerationLimiter.midpoint(lastCommanded, nextField);
    lastCommanded = nextField;

    ChassisVelocities allowed =
        robotRelative ? allowedField.toRobotRelative(heading) : allowedField;
    writeVelocity(request, allowed);

    Logger.recordOutput("Drivetrain/Request", request.getClass().getSimpleName());
    Logger.recordOutput("Drivetrain/WantedVelocity", wanted);
    Logger.recordOutput("Drivetrain/CommandedVelocity", allowed);
    if (RunMode.current() != RunMode.REPLAY) {
      drivetrain.setControl(request);
    }
  }

  /**
   * True when the request's velocity is expressed in the robot's own frame rather than the field's.
   */
  private static boolean isRobotRelative(SwerveRequest request) {
    return request instanceof SwerveRequest.ApplyRobotVelocity
        || request instanceof SwerveRequest.RobotCentric;
  }

  /**
   * Puts the limited velocity back into the request. Commands hold one request and re-fill it each
   * loop - CTRE's own idiom - so mutating it here is safe.
   */
  private static void writeVelocity(SwerveRequest request, ChassisVelocities velocity) {
    switch (request) {
      case SwerveRequest.ApplyFieldVelocity r -> r.withVelocity(velocity);
      case SwerveRequest.ApplyRobotVelocity r -> r.withVelocity(velocity);
      case SwerveRequest.FieldCentric r ->
          r.withVelocityX(velocity.vx)
              .withVelocityY(velocity.vy)
              .withRotationalRate(velocity.omega);
      case SwerveRequest.RobotCentric r ->
          r.withVelocityX(velocity.vx)
              .withVelocityY(velocity.vy)
              .withRotationalRate(velocity.omega);
      default -> {} // Idle and friends command no velocity; nothing to limit
    }
  }

  /** TODO: weigh the robot, measure the carpet, and measure the CG height. These are estimates. */
  private AccelerationLimiter.Config limiterConfig() {
    if (limiterConfig == null) {
      limiterConfig =
          new AccelerationLimiter.Config(
              drivetrain.getModuleLocations(),
              60.0,
              1.1,
              0.2,
              Motor.KRAKEN_X60_FOC,
              TunerConstants.FrontLeft.DriveMotorGearRatio,
              TunerConstants.FrontLeft.WheelRadius,
              120.0);
    }
    return limiterConfig;
  }

  /** Keeps the driver's "forward" matched to the alliance colour. */
  public void applyOperatorPerspective() {
    if (RunMode.current() != RunMode.REPLAY) {
      drivetrain.applyOperatorPerspective();
    }
  }

  /**
   * The velocity a request asks for. Read it next to {@code Drivetrain/Request}: the field-centric
   * requests are field-relative and the robot-centric ones are not. Requests that command no
   * velocity, such as {@code Idle}, read zero.
   */
  private static ChassisVelocities commandedVelocity(SwerveRequest request) {
    return switch (request) {
      case SwerveRequest.ApplyFieldVelocity r -> r.Velocity;
      case SwerveRequest.ApplyRobotVelocity r -> r.Velocity;
      case SwerveRequest.FieldCentric r ->
          new ChassisVelocities(r.VelocityX, r.VelocityY, r.RotationalRate);
      case SwerveRequest.RobotCentric r ->
          new ChassisVelocities(r.VelocityX, r.VelocityY, r.RotationalRate);
      default -> new ChassisVelocities();
    };
  }

  @Override
  public void logInputs() {
    Logger.processInputs("Drivetrain", inputs);
    updateEstimate();

    // Derived from the inputs above, so these are outputs - they recompute during replay, and
    // changing the maths here changes what a replay reports.
    Logger.recordOutput(
        "Drivetrain/TranslationSpeedMps", Math.hypot(inputs.velocity.vx, inputs.velocity.vy));
    Logger.recordOutput("Drivetrain/RotationSpeedRadPerSec", inputs.velocity.omega);
    Logger.recordOutput("Drivetrain/OdometrySamplesPerLoop", inputs.sampleTimestamps.length);
    // Instrumentation only - nothing acts on it yet. Read SkidDetector's blind spots first.
    Logger.recordOutput(
        "Drivetrain/SkidRatio",
        SkidDetector.ratio(
            inputs.moduleVelocities, drivetrain.getModuleLocations(), inputs.velocity.omega));
    Logger.recordOutput("Drivetrain/EstimatedPose", getEstimatedPose());
    // Near zero means the re-integration matches CTRE's - i.e. replay is seeing the real thing.
    Logger.recordOutput(
        "Drivetrain/EstimatedPoseErrorMeters",
        getEstimatedPose().getTranslation().getDistance(inputs.pose.getTranslation()));
    Logger.recordOutput(
        "Drivetrain/OdometryFrequencyHz",
        inputs.odometryPeriodSeconds > 0 ? 1.0 / inputs.odometryPeriodSeconds : 0.0);
  }
}
