// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.opmodes;

import static org.wpilib.units.Units.Seconds;

import frc.robot.Robot;
import org.wpilib.command3.Command;
import org.wpilib.command3.Scheduler;
import org.wpilib.opmode.PeriodicOpMode;
import org.wpilib.opmode.Utility;
import org.wpilib.units.measure.Time;

/**
 * Sweeps the arm across its presets while the flywheel holds speed, so one run produces every
 * number the bring-up needs: a measured gear ratio, a magnet offset, and a steady-state flywheel
 * voltage. Read them out of the log with the device-bringup skill.
 *
 * <p>This one MOVES the mechanisms. Selecting it and enabling is the confirmation - clear the arm's
 * path first.
 */
@Utility(name = "Bring-Up")
public class BringUpOpMode extends PeriodicOpMode {
  // Long enough to arrive and then sit still, so the holding voltage settles before the next pose.
  private static final Time DWELL = Seconds.of(2.5);

  private final Command routine;

  public BringUpOpMode(Robot robot) {
    // Three presets, so the arm travels far enough that the ratio isn't sensor noise.
    //
    // Timed rather than .until(arm::atVertical): every pose has to sit STILL long enough for its
    // holding voltage to settle, or there is no kG to measure.
    Command sweep =
        Command.sequence(
                dwell(robot.arm.vertical(), "at vertical"),
                dwell(robot.arm.horizontal(), "at horizontal"),
                dwell(robot.arm.scoring(), "at scoring"))
            .named("arm sweep");

    // The sweep decides when this ends; the flywheel hold never would.
    routine = Command.race(sweep, robot.flywheel.spinUp()).named("Bring-Up");
  }

  /** Runs a hold for a fixed time - the timer is what ends it. */
  private static Command dwell(Command hold, String name) {
    return Command.race(hold, Command.waitFor(DWELL).named("dwell")).named(name);
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
