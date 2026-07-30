// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.utils;

import org.wpilib.framework.RobotBase;

/**
 * Which of the three ways this code is running: on the robot, in simulation, or replaying a log.
 * Replay is on when {@code -Dfrc.replay=<path to .wpilog>} is set.
 *
 * <p>REPLAY looks like simulation to WPILib, so {@code RobotBase.isSimulation()} is true in both -
 * use this instead when the answer matters (running sim physics, auto-enabling, touching hardware).
 */
public enum RunMode {
  REAL,
  SIM,
  REPLAY;

  private static final String REPLAY_LOG = System.getProperty("frc.replay", "").trim();

  private static final RunMode CURRENT =
      !REPLAY_LOG.isEmpty() ? REPLAY : RobotBase.isReal() ? REAL : SIM;

  /** The mode this process is running in. Fixed at startup. */
  public static RunMode current() {
    return CURRENT;
  }

  /** Path of the log being replayed, or "" when not replaying. */
  public static String replayLog() {
    return REPLAY_LOG;
  }
}
