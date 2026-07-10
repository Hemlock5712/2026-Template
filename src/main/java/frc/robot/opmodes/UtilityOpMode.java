// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.opmodes;

import frc.robot.Robot;
import org.wpilib.command3.Command;
import org.wpilib.command3.Scheduler;
import org.wpilib.opmode.PeriodicOpMode;
import org.wpilib.opmode.Utility;

/**
 * A utility opmode - the third mode kind alongside {@code @Teleop} and {@code @Autonomous}. Utility
 * mode (formerly "Test") is for safe off-field interaction: calibrating a mechanism, checking
 * sensors, or putting the robot in a known pose for maintenance/transport. Like the other opmodes
 * it's auto-discovered and shows up by name on the driver station.
 *
 * <p>This example simply stows the superstructure (arm vertical, flywheel stopped) - a safe pose to
 * leave the robot in on the cart. Replace the routine with whatever calibration you need; one
 * {@code @Utility} class per utility task, same as autonomous.
 *
 * <p>{@link #start()} fires once when the robot is enabled in this mode, which is where the routine
 * is scheduled.
 */
@Utility(name = "Stow")
public class UtilityOpMode extends PeriodicOpMode {
  private final Command routine;

  public UtilityOpMode(Robot robot) {
    routine = robot.stow();
  }

  @Override
  public void start() {
    Scheduler.getDefault().schedule(routine);
  }

  @Override
  public void end() {
    Scheduler.getDefault().cancel(routine);
  }
}
