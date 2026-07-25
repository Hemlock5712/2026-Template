package frc.robot.subsystems;

import com.ctre.phoenix6.Utils;
import com.ctre.phoenix6.swerve.SwerveDrivetrainConstants;
import com.ctre.phoenix6.swerve.SwerveModuleConstants;
import frc.robot.generated.TunerConstants.TunerSwerveDrivetrain;
import org.wpilib.driverstation.Alliance;
import org.wpilib.driverstation.MatchState;
import org.wpilib.driverstation.RobotState;
import org.wpilib.math.geometry.Rotation2d;
import org.wpilib.system.Notifier;
import org.wpilib.system.RobotController;

/**
 * Extends the Phoenix 6 swerve drivetrain: owns the hardware and odometry. The command wrapping
 * lives in {@link DriveMechanism}. Like {@code TunerConstants}, this is an example - regenerate
 * both from Tuner X for your robot.
 */
public class CommandSwerveDrivetrain extends TunerSwerveDrivetrain {
  private static final double SIM_LOOP_PERIOD = 0.004; // 4 ms
  private Notifier simNotifier = null;
  private double lastSimTime;

  /* Blue alliance: forward = 0 deg (toward the red wall) */
  private static final Rotation2d kBlueAlliancePerspectiveRotation = Rotation2d.kZero;
  /* Red alliance: forward = 180 deg (toward the blue wall) */
  private static final Rotation2d kRedAlliancePerspectiveRotation = Rotation2d.k180deg;
  private boolean hasAppliedOperatorPerspective = false;

  /**
   * Constructs the drivetrain - and all its hardware devices - from the Tuner constants.
   *
   * @param drivetrainConstants drivetrain-wide constants
   * @param modules per-module constants
   */
  public CommandSwerveDrivetrain(
      SwerveDrivetrainConstants drivetrainConstants, SwerveModuleConstants<?, ?, ?>... modules) {
    super(drivetrainConstants, modules);
    if (Utils.isSimulation()) {
      startSimThread();
    }
  }

  /**
   * Keeps the driver's "forward" matched to the alliance color. Called every loop by {@link
   * DriveMechanism}.
   */
  public void applyOperatorPerspective() {
    // Apply at startup and whenever disabled - so a mid-match code restart still gets the right
    // perspective, but driving never changes while enabled.
    if (!hasAppliedOperatorPerspective || RobotState.isDisabled()) {
      MatchState.getAlliance()
          .ifPresent(
              allianceColor -> {
                setOperatorPerspectiveForward(
                    allianceColor == Alliance.RED
                        ? kRedAlliancePerspectiveRotation
                        : kBlueAlliancePerspectiveRotation);
                hasAppliedOperatorPerspective = true;
              });
    }
  }

  private void startSimThread() {
    lastSimTime = Utils.getCurrentTimeSeconds();

    /* Sim runs faster than the main loop so PID gains behave reasonably. */
    simNotifier =
        new Notifier(
            () -> {
              final double currentTime = Utils.getCurrentTimeSeconds();
              double deltaTime = currentTime - lastSimTime;
              lastSimTime = currentTime;

              updateSimState(deltaTime, RobotController.getBatteryVoltage());
            });
    simNotifier.startPeriodic(SIM_LOOP_PERIOD);
  }
}
