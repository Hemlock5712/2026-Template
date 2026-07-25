// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.opmodes;

import frc.robot.Robot;
import frc.robot.commands.DriveToPose;
import org.wpilib.command3.Command;
import org.wpilib.command3.Scheduler;
import org.wpilib.math.geometry.Pose2d;
import org.wpilib.math.geometry.Rotation2d;
import org.wpilib.opmode.Autonomous;
import org.wpilib.opmode.PeriodicOpMode;

/**
 * Drive, stow the arm, drive again - a multi-mechanism auto built by <b>chaining</b>. As far as
 * most autos ever need to go.
 *
 * <p>The one rule: <b>a hold never finishes, so never wait on a hold.</b> Two tools work around it:
 *
 * <ul>
 *   <li>{@code hold.until(condition)} - gives a hold a finish line.
 *   <li>{@code Command.race(step, hold)} - do a step WHILE holding. The step always decides when
 *       the race ends, because the hold never finishes.
 * </ul>
 */
@Autonomous(name = "3 - Drive Stow Drive")
public class DriveStowDriveOpMode extends PeriodicOpMode {
  private final Command routine;

  public DriveStowDriveOpMode(Robot robot) {
    // Blue-origin field poses (x forward from the blue wall, y left). TODO: real poses.
    final Pose2d pose1 = new Pose2d(2.0, 0.0, Rotation2d.kZero); // 2 m straight ahead
    final Pose2d pose2 = new Pose2d(2.0, 1.5, Rotation2d.fromDegrees(90)); // then 1.5 m left

    routine =
        Command.sequence(
                // Leg 1: DriveToPose finishes on its own, so it can sit in a sequence as-is.
                new DriveToPose(robot.drivetrain, pose1),

                // Stow is a hold - .until(...) gives it a finish line at the stow angle.
                robot.stow().until(robot.arm::isAtTarget).named("stow until stowed"),

                // Leg 2 WHILE holding stow: the drive finishes, the race cancels the hold.
                Command.race(new DriveToPose(robot.drivetrain, pose2), robot.stow())
                    .named("drive holding stow"))
            .named("Drive Stow Drive");
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
