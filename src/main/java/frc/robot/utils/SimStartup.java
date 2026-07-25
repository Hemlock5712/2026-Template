// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.utils;

import org.wpilib.framework.RobotBase;
import org.wpilib.hardware.hal.OpModeOption;
import org.wpilib.hardware.hal.RobotMode;
import org.wpilib.simulation.DriverStationSim;

/**
 * Headless sim auto-enable: starts the robot without a human clicking Enable. See the run-sim
 * skill.
 *
 * <p>{@code frc.sim.startMode} values: {@code auto} / {@code teleop} / {@code utility} for that
 * kind's default OpMode, {@code <mode>:<OpMode name>} to pick one by name, or empty/{@code
 * disabled} to stay disabled.
 */
public final class SimStartup {
  private SimStartup() {}

  // Which OpMode a bare "-Pmode=auto" (or teleop, or utility) starts. Named on purpose: picking
  // "whichever OpMode is first" lets a newly added class silently take over the default run.
  private static final String DEFAULT_AUTONOMOUS = "3 - Drive Stow Drive";
  private static final String DEFAULT_TELEOP = "Teleop";
  private static final String DEFAULT_UTILITY = "Stow";

  /** Reads {@code frc.sim.startMode} and, in simulation, selects an OpMode and enables the DS. */
  public static void arm() {
    if (!RobotBase.isSimulation()) {
      return;
    }

    String spec = System.getProperty("frc.sim.startMode", "").trim();
    if (spec.isEmpty() || spec.equalsIgnoreCase("disabled")) {
      return; // Stay disabled - normal interactive sim behavior.
    }

    // Split "<mode>" or "<mode>:<name>".
    String modeStr = spec;
    String wantName = null;
    int colon = spec.indexOf(':');
    if (colon >= 0) {
      modeStr = spec.substring(0, colon).trim();
      wantName = spec.substring(colon + 1).trim();
    }

    RobotMode mode =
        switch (modeStr.toLowerCase()) {
          case "auto", "autonomous" -> RobotMode.AUTONOMOUS;
          case "teleop", "teleoperated" -> RobotMode.TELEOPERATED;
          case "utility", "test" -> RobotMode.UTILITY;
          default -> null;
        };
    if (mode == null) {
      System.err.println(
          "[SimStartup] Unknown mode '" + spec + "' (use auto|teleop|utility); staying disabled.");
      return;
    }

    boolean askedByName = wantName != null;
    if (!askedByName) {
      wantName =
          switch (mode) {
            case AUTONOMOUS -> DEFAULT_AUTONOMOUS;
            case TELEOPERATED -> DEFAULT_TELEOP;
            case UTILITY -> DEFAULT_UTILITY;
            default -> null;
          };
    }

    OpModeOption chosen = null;
    OpModeOption firstOfMode = null;
    StringBuilder available = new StringBuilder();
    for (OpModeOption option : DriverStationSim.getOpModeOptions()) {
      if (option.getMode() != mode) {
        continue;
      }
      if (firstOfMode == null) {
        firstOfMode = option;
      }
      if (available.length() > 0) {
        available.append(", ");
      }
      available.append('"').append(option.name).append('"');
      if (chosen == null && option.name.equalsIgnoreCase(wantName)) {
        chosen = option;
      }
    }

    // Default renamed or deleted: complain loudly but still run something, so a CI loop never
    // just sits there disabled.
    if (chosen == null && !askedByName && firstOfMode != null) {
      System.err.println(
          "[SimStartup] Default "
              + mode
              + " OpMode \""
              + wantName
              + "\" not found - was it renamed? Update SimStartup. Falling back to \""
              + firstOfMode.name
              + "\". Available: "
              + available);
      chosen = firstOfMode;
    }
    if (chosen == null) {
      System.err.println(
          "[SimStartup] No "
              + mode
              + " OpMode named \""
              + wantName
              + "\" found; staying disabled. Available: "
              + available);
      return;
    }

    // Both setRobotMode and setOpMode are required - the opmode id combines the two and won't
    // match without the mode bits.
    DriverStationSim.setDsAttached(true);
    DriverStationSim.setRobotMode(mode);
    DriverStationSim.setOpMode(chosen.id);
    DriverStationSim.setEnabled(true);
    DriverStationSim.notifyNewData();

    System.out.println(
        "[SimStartup] Headless start: enabled=true mode="
            + mode
            + " opmode=\""
            + chosen.name
            + "\"");
  }
}
