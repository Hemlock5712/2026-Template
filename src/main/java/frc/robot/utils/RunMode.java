// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.utils;

import org.wpilib.framework.RobotBase;
import org.wpilib.simulation.SimHooks;

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

  // How much simulated time each step jumps. One robot period per step works but costs a real
  // notifier wake every loop (~5 ms on Windows); a big jump leaves most loops already expired when
  // they arm their alarm, so they never sleep at all.
  private static final double CLOCK_CHUNK_SECONDS = 0.5;

  /**
   * Frees the robot loop from wall-clock time so replay runs as fast as the CPU allows.
   *
   * <p>Each cycle the loop arms a HAL notifier alarm and sleeps until it fires
   * (PeriodicPriorityQueue.runCallbacks). Pausing simulated time and jumping it forward in chunks
   * from this thread means most alarms are already expired when armed. stepTiming blocks until the
   * notifiers it woke have run, so the robot loop still sets the pace and never runs ahead of us.
   */
  public static void startFastClock() {
    SimHooks.pauseTiming();
    Thread stepper =
        new Thread(
            () -> {
              while (true) {
                SimHooks.stepTiming(CLOCK_CHUNK_SECONDS);
              }
            },
            "ReplayClock");
    stepper.setDaemon(true);
    stepper.start();
  }
}
