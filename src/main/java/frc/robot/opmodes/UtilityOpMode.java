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
 * A utility opmode (formerly "Test"): safe off-field tasks like calibrating a mechanism or posing
 * the robot for the cart. This one just stows the superstructure - add one {@code @Utility} class
 * per task.
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
