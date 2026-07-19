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
 * The same routine as {@link DriveStowDriveOpMode}, written by <b>chaining</b> instead of
 * coroutines. This is the recommended style for multi-mechanism autos on this team - chaining is as
 * far as most routines ever need to go.
 *
 * <p>Remember the one rule: <b>a hold never finishes, so nothing may ever wait on a hold.</b>
 * Chaining needs just two tools to work around that:
 *
 * <ul>
 *   <li>{@code hold.until(condition)} - gives a hold a finish line, right where you need one.
 *   <li>{@code Command.race(step, hold)} - "do this step WHILE holding that pose." A race ends when
 *       its first member finishes and cancels the rest - and since a hold never finishes, the step
 *       is always what decides.
 * </ul>
 *
 * <p>One trade-off to know: the sequence owns <i>every</i> mechanism it touches for the whole
 * routine, so between its steps a mechanism can show up in telemetry as owned-but-uncommanded (the
 * motor still holds its last setpoint in firmware, and nothing else can steal it). The coroutine
 * version in {@link DriveStowDriveOpMode} keeps everything actively commanded instead - that's the
 * advanced dialect, for when a hold must span many steps or logic needs loops/branches.
 */
@Autonomous(name = "Drive Stow Drive (Chained)")
public class DriveStowDriveChainedOpMode extends PeriodicOpMode {
  private final Command routine;

  public DriveStowDriveChainedOpMode(Robot robot) {
    // Blue-origin field poses (x forward from the blue wall, y left). TODO: real poses.
    final Pose2d pose1 = new Pose2d(2.0, 0.0, Rotation2d.kZero); // 2 m straight ahead
    final Pose2d pose2 = new Pose2d(2.0, 1.5, Rotation2d.fromDegrees(90)); // then 1.5 m left

    routine =
        Command.sequence(
                // Leg 1: DriveToPose finishes on its own, so it can sit in a sequence as-is.
                new DriveToPose(robot.drivetrain, pose1),

                // Stow is a hold - it would stick here forever. .until(...) gives it a finish
                // line: this step ends the moment the arm actually reaches the stow angle.
                robot.stow().until(robot.arm::isAtTarget).named("stow until stowed"),

                // Leg 2 WHILE holding the stow pose: the race ends when DriveToPose finishes
                // (the hold never finishes, so the drive always decides) and cancels the hold.
                Command.race(new DriveToPose(robot.drivetrain, pose2), robot.stow())
                    .named("drive holding stow"))
            .named("Drive Stow Drive (Chained)");
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
