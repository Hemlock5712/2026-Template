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
 * The same routine as {@link DriveStowDriveChainedOpMode}, written with coroutines ({@code fork} /
 * {@code await}) - the <b>advanced dialect</b>. Learn chaining first; reach for coroutines when a
 * hold must span many steps or the logic needs real loops/branches.
 *
 * <p>Why not {@code Command.sequence}? A sequence owns every mechanism for the whole routine, even
 * between the steps that command it. Coroutines keep closed-loop mechanisms (our arm and flywheel)
 * actively commanded the whole time.
 *
 * <p>The three verbs:
 *
 * <ul>
 *   <li>{@code coroutine.await(command)} - run a command and wait for it to finish.
 *   <li>{@code coroutine.fork(command)} - start a command and keep going; it runs in the background
 *       and is auto-canceled when this routine ends.
 *   <li>{@code coroutine.waitUntil(condition)} - pause until the condition is true.
 * </ul>
 */
@Autonomous(name = "Drive Stow Drive")
public class DriveStowDriveOpMode extends PeriodicOpMode {
  private final Command routine;

  public DriveStowDriveOpMode(Robot robot) {
    // Blue-origin field poses (x forward from the blue wall, y left). TODO: real poses.
    final Pose2d pose1 = new Pose2d(2.0, 0.0, Rotation2d.kZero); // 2 m straight ahead
    final Pose2d pose2 = new Pose2d(2.0, 1.5, Rotation2d.fromDegrees(90)); // then 1.5 m left

    routine =
        Command.noRequirements(
                coroutine -> {
                  // Drive to the first pose and wait until we're there.
                  coroutine.await(new DriveToPose(robot.drivetrain, pose1));

                  // fork: start holding the stow pose and keep going. The hold stays commanded
                  // through the next drive and is auto-canceled when the routine ends.
                  coroutine.fork(robot.stow());
                  coroutine.waitUntil(robot.arm::isAtTarget); // move on once actually stowed

                  // Drive to the second pose while the stow pose is still held.
                  coroutine.await(new DriveToPose(robot.drivetrain, pose2));
                })
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
