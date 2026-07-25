// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.opmodes;

import static org.wpilib.units.Units.Seconds;

import frc.robot.Robot;
import frc.robot.commands.DriveDistance;
import org.wpilib.command3.Command;
import org.wpilib.command3.Scheduler;
import org.wpilib.opmode.Autonomous;
import org.wpilib.opmode.PeriodicOpMode;

/**
 * The simplest autonomous in this project: drive forward 2 meters and stop.
 *
 * <p>Start here. There are no field positions, no alliance colors, and no motion profiles - just
 * one {@link DriveDistance} command. Once this makes sense, "2 - Drive To Pose" is the next step
 * up.
 */
@Autonomous(name = "1 - Drive 2 Meters")
public class DriveDistanceOpMode extends PeriodicOpMode {
  private static final double DISTANCE_METERS = 2.0;

  private final Command routine;

  public DriveDistanceOpMode(Robot robot) {
    // The seatbelt: a blocked robot never reaches 2 meters, so give up instead of pushing for
    // the whole autonomous period.
    routine = new DriveDistance(robot.drivetrain, DISTANCE_METERS).withTimeout(Seconds.of(5.0));
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
