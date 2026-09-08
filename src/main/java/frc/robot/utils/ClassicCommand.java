// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.utils;

import java.util.Arrays;
import java.util.Set;
import org.wpilib.command3.Command;
import org.wpilib.command3.Coroutine;
import org.wpilib.command3.Mechanism;

/**
 * The classic v2 command style ({@code initialize/execute/isFinished/end}) on top of Commands v3.
 * Extend it and override what you need; the instance is a {@link Command}.
 *
 * <p>{@link frc.robot.commands.DriveDistance} is a worked example.
 */
public abstract class ClassicCommand implements Command {
  private final String name;
  private final Set<Mechanism> requirements;

  /** The running coroutine. Only valid inside initialize/execute/isFinished, null otherwise. */
  protected Coroutine coroutine;

  /**
   * Creates a classic-style command.
   *
   * @param name The command name (shows up in telemetry).
   * @param requirements The mechanisms this command owns while it runs.
   */
  protected ClassicCommand(String name, Mechanism... requirements) {
    this.name = name;
    this.requirements = Set.copyOf(Arrays.asList(requirements));
  }

  /** Runs once when the command starts. Override to set up state. */
  protected void initialize() {}

  /** Runs every loop while the command is active. Override to do the work. */
  protected void execute() {}

  /**
   * Checked every loop, right after {@link #execute()}.
   *
   * @return true to finish the command, false to keep running.
   */
  protected boolean isFinished() {
    return false;
  }

  /**
   * Runs once when the command ends. Single-shot cleanup only (e.g. stop a motor).
   *
   * @param interrupted true if another command interrupted this one, false if {@link #isFinished()}
   *     ended it.
   */
  protected void end(boolean interrupted) {}

  @Override
  public final void run(Coroutine coroutine) {
    this.coroutine = coroutine;
    initialize();
    while (true) {
      execute();
      if (isFinished()) {
        break;
      }
      coroutine.yield();
    }
    end(false); // natural finish
  }

  @Override
  public final void onCancel() {
    // Interruption drops the coroutine, so this is the only cleanup hook that runs.
    end(true);
  }

  @Override
  public final String name() {
    return name;
  }

  @Override
  public final Set<Mechanism> requirements() {
    return requirements;
  }
}
