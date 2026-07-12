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
 * A closed-loop autonomous routine written with coroutines ({@code fork} / {@code await}).
 *
 * <p><b>Why coroutines here instead of {@code Command.sequence}?</b> The {@code sequence} and {@code
 * parallel} builders use the older ownership rule: the group owns <i>every</i> mechanism for the
 * whole routine, so a mechanism that isn't being actively driven right now still shows up as "owned"
 * with nothing actually commanding it. That's fine for trivial logic or plain onboard (open-loop)
 * motors, but for closed-loop mechanisms - our Motion Magic arm and flywheel - the rule is: <i>the
 * command that issued a control request should keep running as long as that request is active.</i>
 * You should never fall back to idle while a motor is still holding a setpoint. Coroutines give that
 * finer-grained control. (Compare with {@link AutonomousOpMode}, which uses {@code Command.sequence}
 * for a plain drivetrain-only routine - exactly the trivial case where the builder is fine.)
 *
 * <p><b>The only three verbs you need:</b>
 *
 * <ul>
 *   <li>{@code coroutine.await(command)} - run a command and wait here until it finishes.
 *   <li>{@code coroutine.fork(command)} - start a command and keep going; it runs in the background
 *       (holding a setpoint, say), stays visible in telemetry, and is auto-canceled when this
 *       routine ends.
 *   <li>{@code coroutine.waitUntil(condition)} - pause here until the condition becomes true.
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

                  // Start holding the stow pose (arm vertical + flywheel stopped). fork keeps it
                  // running - so it stays commanded and visible in telemetry - through the next
                  // drive, and it is canceled automatically when this routine ends.
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
