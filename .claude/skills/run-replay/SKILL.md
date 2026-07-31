---
name: run-replay
description: Replay a recorded .wpilog through the current code with AdvantageKit — how to record a log, run it back at ~33x speed, change logic and see what the robot would have done instead, and the replayCheck task that proves replay still works. Use when asked to replay a log, re-tune a threshold or vision trust number against a real match, debug something that only happened once on the field, or when replay stops reproducing a recording.
---

# Replaying a log

> File links below are relative to the **repo root**, not to this skill's directory.
> This is the **advanced-replay** branch. Plain `main` has none of this.

Replay re-runs your code against a recorded log. Sensor values come from the log instead of
hardware, so the robot "sees" exactly what it saw on the field, and anything you change in your
own code produces a new answer on that old data.

## The two commands

```bash
./gradlew simulateJavaAgent -PstopAfter=20        # record; ends on its own after 20 s
./gradlew replayJava                              # replay the newest log in ./logs
./gradlew replayJava -Preplay=logs/akit_....wpilog   # or pick one
```

Replay writes `<log>_replay.wpilog` next to the input. Open it in AdvantageScope and you get
**both**:

- `RealOutputs/…` — what the robot actually did, copied from the recording.
- `ReplayOutputs/…` — what **this build of the code** computes from the same inputs.

Graph the two against each other. They share one time axis: every replayed sample lands on a
timestamp the original run sampled.

## What you can and cannot re-tune

Replay only re-runs decisions that live in **our** code.

| Re-runs | Does not |
| --- | --- |
| `isAtTarget`, jam/stall rules, any threshold | TalonFX closed-loop gains — those run *on the motor* |
| Command and OpMode logic, autos, state machines | CTRE swerve odometry (its own 250 Hz thread) |
| Vision gates and trust numbers ([Vision.java](../../src/main/java/frc/robot/subsystems/vision/Vision.java)) | The MegaTag solve itself (needs the raw image) |
| Anything derived from a sensor reading | The pose estimator's output — the pose is an input |

Changing `kP` and replaying will do nothing. That is not a bug; the gain lives on the Talon.

## Worked example: re-tuning vision trust

1. Record a log (or use a real match log off the robot).
2. Change `XY_STD_DEV` in [Vision.java](../../src/main/java/frc/robot/subsystems/vision/Vision.java).
3. `./gradlew replayJava -Preplay=<that log>`
4. Compare `ReplayOutputs/Vision/<camera>/AcceptedStdDevXY` against `RealOutputs/…`.

The camera data is untouched — same frames, same tags, same distances — and only the trust number
moves. Same trick for `MAX_TAG_DISTANCE_METERS` or the tag-count gates.

## replayCheck — keep replay honest

```bash
./gradlew simulateJavaAgent -PstopAfter=20
./gradlew replayCheck
```

Replays and then compares every output key. With the code unchanged the two must agree; if they
do not, **some value is still coming from hardware instead of the log** and replay is quietly
lying to you. Run it after touching anything in `frc/robot/hardware`.

It ignores `Logger/*` and `LoggedRobot/*` — AdvantageKit's own timing telemetry measures elapsed
wall clock, which is *meant* to differ when replay runs 33x faster.

## Rules that keep replay working

1. **Every device goes through a wrapper** in [frc/robot/hardware](../../src/main/java/frc/robot/hardware).
   Never `new TalonFX(...)` in a subsystem. `checkReplaySafety` fails the build if you do.
2. **Never use `Utils.getCurrentTimeSeconds()`** (Phoenix's clock). AdvantageKit does not redirect
   it, so during replay it runs at wall-clock speed while the log runs 33x faster — anything timing
   itself with it never finishes. Use `RobotController.getTime() / 1.0e6`. Also build-checked.
3. **Guard sim physics on `RunMode.current() == SIM`, not `isSimulation()`.** Replay *is*
   simulation to WPILib, and re-running physics would fight the logged values.
4. **Read sensors through `LoggedHardware.refreshAll()`**, which [Robot.java](../../src/main/java/frc/robot/Robot.java)
   calls once at the top of the loop. Reading a motor in a constructor gives 0 — that runs before
   the first refresh.
5. **A log must contain a disabled → enabled transition.** OpModes schedule their commands on that
   edge; a log that is already enabled on its first entry replays with no commands running at all.
   `SimStartup` stays disabled briefly for exactly this reason.

## When replay stops matching

`replayCheck` names the key and the first bad timestamp. Work backwards:

- Key missing from `ReplayOutputs` entirely → the code path never ran. Usually a command that never
  started (rule 5) or one that never finished (rule 2).
- Values drift apart over time → something is still reading hardware, or timing itself off the
  wrong clock.
- Only `theta` differs, by ~1e-18 → harmless. `Rotation2d` round-trips radians → cos/sin → `atan2`
  through the log. `replayCheck` already tolerates it.

## How it is wired

- [RunMode.java](../../src/main/java/frc/robot/utils/RunMode.java) — REAL / SIM / REPLAY, from `-Dfrc.replay`.
- [Robot.java](../../src/main/java/frc/robot/Robot.java) — sets the replay source, and calls
  `setUseTiming(false)` so the loop free-runs instead of sleeping 20 ms per cycle (4x → 33x).
- `src/main/java/org/wpilib/**` — **local copies of two WPILib files**, patched to add that
  `setUseTiming` switch, because `OpModeRobot.startCompetition()` is `final`. Re-copy them from the
  wpilibj sources on every WPILib bump. Delete both once the change lands upstream.
- [ReplayCheck.java](../../src/test/java/frc/robot/ReplayCheck.java) — the comparison.
