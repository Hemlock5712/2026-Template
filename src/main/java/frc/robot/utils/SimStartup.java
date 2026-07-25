// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.utils;

import org.wpilib.framework.RobotBase;
import org.wpilib.hardware.hal.OpModeOption;
import org.wpilib.hardware.hal.RobotMode;
import org.wpilib.simulation.DriverStationSim;

/**
 * Headless sim auto-enable: lets an agent/CI run start the robot without a human clicking Enable.
 * {@link frc.robot.Robot#simulationInit()} calls {@link #arm()}, which reads the {@code
 * frc.sim.startMode} system property (see the run-sim skill) and drives the sim driver station.
 *
 * <p>Values: {@code auto} / {@code teleop} / {@code utility} (first OpMode of that kind), {@code
 * <mode>:<OpMode name>} to pick one by name, or empty/{@code disabled} to stay disabled.
 */
public final class SimStartup {
  private SimStartup() {}

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

    OpModeOption chosen = null;
    for (OpModeOption option : DriverStationSim.getOpModeOptions()) {
      if (option.getMode() != mode) {
        continue;
      }
      if (wantName == null || option.name.equalsIgnoreCase(wantName)) {
        chosen = option;
        break;
      }
    }
    if (chosen == null) {
      System.err.println(
          "[SimStartup] No "
              + mode
              + " OpMode"
              + (wantName != null ? " named \"" + wantName + "\"" : "")
              + " found; staying disabled.");
      return;
    }

    // Both setRobotMode and setOpMode are required - the opmode id the framework reads back
    // combines the two, and it won't match without the mode bits.
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
