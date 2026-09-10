// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.hardware;

import static org.wpilib.units.Units.Hertz;

import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.swerve.SwerveRequest;
import com.ctre.phoenix6.swerve.utility.WheelForceCalculator;
import frc.robot.Robot;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.utils.RunMode;
import frc.robot.utils.SkidDetector;
import java.util.ArrayList;
import java.util.Optional;
import java.util.concurrent.ArrayBlockingQueue;
import org.littletonrobotics.junction.AutoLog;
import org.littletonrobotics.junction.Logger;
import org.wpilib.math.estimator.SwerveDrivePoseEstimator;
import org.wpilib.math.geometry.Pose2d;
import org.wpilib.math.geometry.Rotation2d;
import org.wpilib.math.geometry.Rotation3d;
import org.wpilib.math.geometry.Translation2d;
import org.wpilib.math.kinematics.ChassisVelocities;
import org.wpilib.math.kinematics.SwerveDriveKinematics;
import org.wpilib.math.kinematics.SwerveModulePosition;
import org.wpilib.math.kinematics.SwerveModuleVelocity;
import org.wpilib.math.linalg.Matrix;
import org.wpilib.math.linalg.VecBuilder;
import org.wpilib.math.numbers.N1;
import org.wpilib.math.numbers.N3;
import org.wpilib.units.measure.Frequency;

/**
 * The swerve drivetrain as a single logged device: its whole state goes through the log, so
 * everything downstream - autos, drive-to-pose, alignment - replays.
 *
 * <p>The twelve swerve devices are not wrapped individually. CTRE runs them on its own 250 Hz
 * odometry thread inside the drivetrain, which this code cannot get between, so CTRE's odometry
 * *answer* is an input here rather than the sensors behind it. Every wheel position and heading it
 * integrated is logged too, which is what lets {@link #getPose} re-integrate a pose that replays
 * instead of handing back CTRE's frozen one.
 *
 * <p>This exposes the same methods the drivetrain does and handles replay internally, so nothing
 * downstream branches on {@code RunMode} - swapping the type in is the whole change. Reads come
 * from the log, writes are skipped in replay and mirrored onto our estimator where they move it.
 *
 * <p>Five of CTRE's methods are deliberately NOT forwarded, and adding them would undo the wrapper:
 *
 * <ul>
 *   <li>{@code getState} / {@code getStateCopy} - live CTRE state, frozen in replay. Every field is
 *       already a getter here, fed from the log.
 *   <li>{@code getModule} / {@code getModules} / {@code getPigeon2} - hand out raw Phoenix devices,
 *       which read the bus directly in replay. {@code checkReplaySafety} fails the build for this.
 *   <li>{@code registerTelemetry} - we own the one callback; a second one would steal the samples.
 *   <li>{@code updateSimState} - driven by {@link CommandSwerveDrivetrain}'s 4 ms sim notifier.
 *   <li>{@code optimizeBusUtilization} - it drops unused signals to 0 Hz, undoing the 250 Hz rates
 *       {@link LoggedHardware#initialize} just set. Bus config belongs there, not here.
 * </ul>
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
    // False when CTRE's odometry thread is not keeping up - a dropped or faulted sample.
    public boolean odometryValid;
    public boolean onCanFd;
    // Full gyro orientation: pitch and roll as well as yaw, for tip detection.
    public Rotation3d rotation3d = new Rotation3d();
    public Rotation2d operatorForwardDirection = new Rotation2d();

    // Every odometry sample since the last loop - one or two at 250 Hz - so code in the main loop
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

  // Also first-loop, and the velocity it differences against. Copied, not aliased - CTRE reuses
  // the state object, so holding its reference would difference a value against itself.
  private WheelForceCalculator wheelForces;
  private ChassisVelocities previousVelocity = new ChassisVelocities();

  // WPILib takes both in the constructor, so changing either rebuilds the estimator. These are its
  // defaults - state 0.1 m / 0.1 rad, vision 0.9 m / 0.9 rad.
  private Matrix<N3, N1> stateStdDevs = VecBuilder.fill(0.1, 0.1, 0.1);
  private Matrix<N3, N1> visionStdDevs = VecBuilder.fill(0.9, 0.9, 0.9);

  // Set when the wheel encoders themselves are about to jump (tareEverything), so the estimator is
  // rebuilt on NEXT loop's positions instead of integrating the discontinuity as real motion.
  private Pose2d pendingResync;

  public LoggedSwerveDrivetrain(CommandSwerveDrivetrain drivetrain) {
    this.drivetrain = drivetrain;
    // No signals to batch: CTRE reads the modules itself on its odometry thread.
    LoggedHardware.register(this, "Drivetrain");

    // Not in replay: the samples come from the log, and nothing would ever drain this queue.
    if (!RunMode.isReplay()) {
      // Runs on CTRE's odometry thread, holding its state lock. Copy and leave - CTRE warns that
      // slow work here degrades odometry, and AdvantageKit's logger is not thread-safe.
      drivetrain.registerTelemetry(
          state ->
              samples.offer(
                  new Sample(state.Timestamp, state.RawHeading, state.ModulePositions.clone())));
    }
  }

  /**
   * Field pose from odometry, blue-origin (the origin never flips with alliance). This is OUR
   * estimator, re-integrated from the logged samples, not CTRE's - so it recomputes during replay
   * and everything driving off it replays too. CTRE's own answer stays in the log as {@code
   * Drivetrain/Pose}; falls back to it until the first loop has run.
   */
  public Pose2d getPose() {
    return estimator == null ? inputs.pose : estimator.getEstimatedPosition();
  }

  /** Velocity in the robot frame. */
  public ChassisVelocities getVelocity() {
    return inputs.velocity;
  }

  /** Velocity rotated into the field frame. */
  public ChassisVelocities getFieldVelocity() {
    return inputs.velocity.toFieldRelative(getPose().getRotation());
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

  /** Our estimate at an earlier timestamp, for latency compensation. Empty outside the buffer. */
  public Optional<Pose2d> samplePoseAt(double timestampSeconds) {
    return estimator == null ? Optional.empty() : estimator.sampleAt(timestampSeconds);
  }

  /** Full gyro orientation - pitch and roll as well as yaw. */
  public Rotation3d getRotation3d() {
    return inputs.rotation3d;
  }

  /** Which field direction the driver's "forward" points: blue 0 deg, red 180 deg. */
  public Rotation2d getOperatorForwardDirection() {
    return inputs.operatorForwardDirection;
  }

  /** False when CTRE's odometry thread stopped keeping up - the pose above is stale. */
  public boolean isOdometryValid() {
    return inputs.odometryValid;
  }

  /** True on a CANivore, which is what allows the 250 Hz odometry rate. */
  public boolean isOnCANFD() {
    return inputs.onCanFd;
  }

  /** Odometry samples per second: 250 on CAN FD, 100 on plain CAN. */
  public double getOdometryFrequency() {
    return inputs.odometryPeriodSeconds > 0 ? 1.0 / inputs.odometryPeriodSeconds : 0.0;
  }

  /** {@link #getOdometryFrequency} as a unit-typed measure. */
  public Frequency getOdometryFrequencyMeasure() {
    return Hertz.of(getOdometryFrequency());
  }

  /**
   * Wheel layout, for turning module velocities into a chassis velocity. Built from {@code
   * TunerConstants}, which is identical in replay, so it is safe to read off the drivetrain.
   */
  public SwerveDriveKinematics getKinematics() {
    return drivetrain.getKinematics();
  }

  /** Where each module sits relative to the robot center. Constant, as {@link #getKinematics}. */
  public Translation2d[] getModuleLocations() {
    return drivetrain.getModuleLocations();
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

    // Not carried in SwerveDriveState, so read straight off the drivetrain.
    inputs.odometryValid = drivetrain.isOdometryValid();
    inputs.onCanFd = drivetrain.isOnCANFD();
    inputs.rotation3d = drivetrain.getRotation3d();
    inputs.operatorForwardDirection = drivetrain.getOperatorForwardDirection();

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
      rebuildEstimator(inputs.pose);
    } else if (pendingResync != null) {
      rebuildEstimator(pendingResync);
      pendingResync = null;
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
   * Snaps the pose to a known starting point - an auto's first waypoint, or a bench test with no
   * tags in sight. Both estimators, so {@code Drivetrain/Pose} stays a fair comparison.
   *
   * <p>This re-aims the heading MegaTag2 solves against, so a reset the robot doesn't actually
   * match turns every later single-tag fix into that same error. Only call it when you know where
   * it is. Before the first loop the estimator does not exist yet; it seeds itself from the reset
   * pose.
   */
  public void resetPose(Pose2d pose) {
    if (estimator != null) {
      estimator.resetPose(pose);
    }
    if (!RunMode.isReplay()) {
      drivetrain.resetPose(pose);
    }
  }

  /** Moves the pose without touching the heading. Blue-origin, as {@link #resetPose}. */
  public void resetTranslation(Translation2d translation) {
    if (estimator != null) {
      estimator.resetTranslation(translation);
    }
    if (!RunMode.isReplay()) {
      drivetrain.resetTranslation(translation);
    }
  }

  /** Turns the pose without moving it. Read {@link #resetPose}'s warning about MegaTag2 first. */
  public void resetRotation(Rotation2d rotation) {
    if (estimator != null) {
      estimator.resetRotation(rotation);
    }
    if (!RunMode.isReplay()) {
      drivetrain.resetRotation(rotation);
    }
  }

  /**
   * Makes the robot's current heading the driver's "forward". Same warning as {@link #resetPose}.
   */
  public void seedFieldCentric() {
    seedFieldCentric(Rotation2d.kZero);
  }

  /** {@link #seedFieldCentric} with an offset: the robot ends up {@code rotation} off forward. */
  public void seedFieldCentric(Rotation2d rotation) {
    resetRotation(rotation.plus(getOperatorForwardDirection()));
  }

  /** Zeroes the wheel encoders and puts the robot at the origin. Bench use - never mid-match. */
  public void tareEverything() {
    if (!RunMode.isReplay()) {
      drivetrain.tareEverything();
    }
    // The wheel positions jump next loop, so rebuild rather than integrate the jump as motion.
    pendingResync = Pose2d.kZero;
  }

  /** Brake or coast on every drive motor. */
  public StatusCode configNeutralMode(NeutralModeValue neutralMode) {
    return RunMode.isReplay() ? StatusCode.OK : drivetrain.configNeutralMode(neutralMode);
  }

  /** {@link #configNeutralMode} with an explicit CAN timeout. */
  public StatusCode configNeutralMode(NeutralModeValue neutralMode, double timeoutSeconds) {
    return RunMode.isReplay()
        ? StatusCode.OK
        : drivetrain.configNeutralMode(neutralMode, timeoutSeconds);
  }

  /** Rotates the driver's "forward" for the alliance. Does not move the pose, which stays blue. */
  public void setOperatorPerspectiveForward(Rotation2d fieldDirection) {
    if (!RunMode.isReplay()) {
      drivetrain.setOperatorPerspectiveForward(fieldDirection);
    }
  }

  /** Default trust for vision fixes that don't carry their own. Applies to our estimator too. */
  public void setVisionMeasurementStdDevs(Matrix<N3, N1> stdDevs) {
    visionStdDevs = stdDevs;
    if (estimator != null) {
      estimator.setVisionMeasurementStdDevs(stdDevs);
    }
    if (!RunMode.isReplay()) {
      drivetrain.setVisionMeasurementStdDevs(stdDevs);
    }
  }

  /**
   * How far to trust wheel odometry against vision. WPILib takes this in the constructor, so
   * changing it after the first loop rebuilds our estimator and drops its sample history - call it
   * at startup.
   */
  public void setStateStdDevs(Matrix<N3, N1> stdDevs) {
    stateStdDevs = stdDevs;
    if (estimator != null) {
      rebuildEstimator(getPose());
    }
    if (!RunMode.isReplay()) {
      drivetrain.setStateStdDevs(stdDevs);
    }
  }

  /** A vision fix at the estimator's default trust - see {@link #setVisionMeasurementStdDevs}. */
  public void addVisionMeasurement(Pose2d visionRobotPose, double timestampSeconds) {
    addVisionMeasurement(visionRobotPose, timestampSeconds, visionStdDevs);
  }

  /** Seeds a fresh estimator at {@code pose} from the current wheel positions. */
  private void rebuildEstimator(Pose2d pose) {
    estimator =
        new SwerveDrivePoseEstimator(
            getKinematics(),
            inputs.rawHeading,
            inputs.modulePositions,
            pose,
            stateStdDevs,
            visionStdDevs);
  }

  /**
   * Corrects the pose with a vision measurement. Our estimator is corrected even during replay - it
   * is ours, so a changed trust number moves the pose the robot drives on. CTRE's native estimator
   * is not running then, so it is skipped.
   */
  public void addVisionMeasurement(
      Pose2d visionRobotPose, double timestampSeconds, Matrix<N3, N1> stdDevs) {
    if (estimator != null) {
      estimator.addVisionMeasurement(visionRobotPose, timestampSeconds, stdDevs);
    }
    if (!RunMode.isReplay()) {
      drivetrain.addVisionMeasurement(visionRobotPose, timestampSeconds, stdDevs);
    }
  }

  /**
   * Sends a control request to the drivetrain. Logged either way, so a replay shows what the code
   * decided even though there is no drivetrain to send it to.
   */
  public void setControl(SwerveRequest request) {
    Logger.recordOutput("Drivetrain/Request", request.getClass().getSimpleName());
    Logger.recordOutput("Drivetrain/CommandedVelocity", commandedVelocity(request));
    if (!RunMode.isReplay()) {
      drivetrain.setControl(request);
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

  /** Keeps the driver's "forward" matched to the alliance colour. */
  public void applyOperatorPerspective() {
    if (!RunMode.isReplay()) {
      drivetrain.applyOperatorPerspective();
    }
  }

  @Override
  public void logInputs() {
    Logger.processInputs("Drivetrain", inputs);
    updateEstimate();

    // Derived from the inputs above, so these are outputs - they recompute during replay, and
    // changing the maths here changes what a replay reports.
    Logger.recordOutput(
        "Drivetrain/TranslationSpeedMps", Math.hypot(inputs.velocity.vx, inputs.velocity.vy));
    Logger.recordOutput("Drivetrain/OdometrySamplesPerLoop", inputs.sampleTimestamps.length);
    // Instrumentation only - nothing acts on it yet. Read SkidDetector's blind spots first.
    Logger.recordOutput(
        "Drivetrain/SkidRatio",
        SkidDetector.ratio(
            inputs.moduleVelocities, drivetrain.getModuleLocations(), inputs.velocity.omega));
    // What the last cycle's velocity change ASKED of each module. Rigid-body allocation only: it
    // knows nothing about load transfer, so this is demand, not grip. Read it against each module's
    // StatorCurrentAmps to see how much of the traction budget a launch really used.
    // TODO measure the robot's mass and yaw MOI - both scale this output linearly.
    if (wheelForces == null) {
      wheelForces = new WheelForceCalculator(drivetrain.getModuleLocations(), 55.0, 6.0);
    }
    var demand = wheelForces.calculate(Robot.PERIOD_SECONDS, previousVelocity, inputs.velocity);
    double[] demandNewtons = new double[demand.x_newtons.length];
    for (int i = 0; i < demandNewtons.length; i++) {
      demandNewtons[i] = Math.hypot(demand.x_newtons[i], demand.y_newtons[i]);
    }
    Logger.recordOutput("Drivetrain/WheelForceDemandNewtons", demandNewtons);
    previousVelocity =
        new ChassisVelocities(inputs.velocity.vx, inputs.velocity.vy, inputs.velocity.omega);

    Logger.recordOutput("Drivetrain/EstimatedPose", getPose());
    // Near zero means the re-integration matches CTRE's - i.e. replay is seeing the real thing.
    Logger.recordOutput(
        "Drivetrain/EstimatedPoseErrorMeters",
        getPose().getTranslation().getDistance(inputs.pose.getTranslation()));
    Logger.recordOutput(
        "Drivetrain/OdometryFrequencyHz",
        inputs.odometryPeriodSeconds > 0 ? 1.0 / inputs.odometryPeriodSeconds : 0.0);
  }
}
