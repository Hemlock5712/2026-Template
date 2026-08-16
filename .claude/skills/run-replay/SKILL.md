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
| Command and OpMode logic, autos, state machines | CTRE's `Drivetrain/Pose` — its own 250 Hz native thread |
| Vision gates and trust numbers ([Vision.java](../../src/main/java/frc/robot/subsystems/vision/Vision.java)) | The MegaTag solve itself (needs the raw image) |
| Tag-space alignment ([DriveToTag](../../src/main/java/frc/robot/commands/DriveToTag.java)) | |
| `Drivetrain/EstimatedPose` — wheel odometry + vision, recomputed in robot code | Physics. Replay re-runs decisions, not the robot's response to them |
| Anything derived from a sensor reading | |

Changing `kP` and replaying will do nothing. That is not a bug; the gain lives on the Talon.

## The drivetrain in replay

The swerve has two poses, and the difference is the whole point:

- **`Drivetrain/Pose`** — CTRE's, computed natively at 250 Hz. Logged as an **input**; replay copies
  it. Reference only — no command reads it.
- **`Drivetrain/EstimatedPose`** — a WPILib `SwerveDrivePoseEstimator` inside
  [LoggedSwerveDrivetrain](../../src/main/java/frc/robot/hardware/LoggedSwerveDrivetrain.java), fed
  from the logged wheel positions, heading and vision measurements. An **output**: it recomputes, so
  odometry and vision-trust changes move it. **This is what `getPose()` returns**, which is why a
  retuned trust number changes what the replayed robot does and not just a graph.

`Drivetrain/EstimatedPoseErrorMeters` is the gap between them — the self-check that the
recomputation is seeing real data (~2 mm over a 20 s sim run). Scale the logged wheel distances by
2% and replay the same log: `ReplayOutputs/…EstimatedPose` walks away from `RealOutputs/…` while
`Drivetrain/Pose` sits still.

### 250 Hz samples in a 200 Hz loop

CTRE's odometry runs far faster than the robot loop, so reading it once per loop throws most of the
data away. `registerTelemetry` hands us every sample on CTRE's own thread, where logging is illegal
(AdvantageKit's logger is not thread-safe, and CTRE warns that slow work there degrades odometry).
So the callback does one cheap thing — copy the sample into a queue — and
[LoggedSwerveDrivetrain](../../src/main/java/frc/robot/hardware/LoggedSwerveDrivetrain.java) drains
that queue in `updateInputs`, logging all of them as **inputs**:

- `Drivetrain/SampleTimestamps`, `SampleHeadings`, `SamplePositions` (flattened, four per timestamp)
- `Drivetrain/OdometrySamplesPerLoop` — how many arrived; ~1.25 on a 250 Hz CAN FD bus into the
  200 Hz loop (measured mean 1.26 in sim, min 0, max 6)

The estimator then integrates **every** sample in the main loop, which is what gets the error down
to millimetres, and it all still replays because the samples came from the log. Costs ~7 KB/s of log.

This is the pattern for any high-rate drive logic: **capture on the fast thread, decide in the main
loop.** Logic that runs *on* CTRE's thread cannot replay — during replay that thread has no logged
data feeding it. Logic that *consumes* the drained samples replays exactly.

`Drivetrain/CommandedVelocity` is what the code asked the drive for, next to `Drivetrain/Request`
saying which request carried it. Read the two together — the field-centric requests are
field-relative and the robot-centric ones are not.

## Worked example: re-tuning vision trust

1. Record a log (or use a real match log off the robot).
2. Change `XY_STD_DEV` in [Vision.java](../../src/main/java/frc/robot/subsystems/vision/Vision.java).
3. `./gradlew replayJava -Preplay=<that log>`
4. Compare `ReplayOutputs/Vision/<camera>/AcceptedStdDevXY` against `RealOutputs/…`.

The camera data is untouched — same frames, same tags, same distances — and only the trust number
moves. Same trick for `MAX_TAG_DISTANCE_METERS` or the tag-count gates.

And you are not limited to the values the current formula uses.
[LoggedLimelight](../../src/main/java/frc/robot/hardware/LoggedLimelight.java) logs the whole frame —
per-tag ambiguity, tag span, the library's own std devs, target-space poses, the camera's IMU — so a
formula written next season can reach for a number nobody thought to consume when the log was
recorded. That is the point of logging the device rather than the decision.

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
2. **Only the wrapper knows about replay.** `RunMode.REPLAY` or `RunMode.isReplay()` outside
   `frc/robot/hardware` fails the build (`Robot.java` excepted — it is what turns replay on). A wrapper exposes the same methods the
   real device does and decides internally what to skip, so swapping `SwerveDrivetrain` for
   `LoggedSwerveDrivetrain` is the *only* change replay costs you: no mode checks leak into
   mechanisms, commands or OpModes. Sim **physics** is different — a plant that only exists in sim
   still guards on `RunMode.current() == SIM` where it lives.
3. **Never use `Utils.getCurrentTimeSeconds()`** (Phoenix's clock). AdvantageKit does not redirect
   it, so during replay it runs at wall-clock speed while the log runs 33x faster — anything timing
   itself with it never finishes. Use `RobotController.getTime() / 1.0e6`. Also build-checked.
4. **Guard sim physics on `RunMode.current() == SIM`, not `isSimulation()`.** Replay *is*
   simulation to WPILib, and re-running physics would fight the logged values.
5. **Read sensors through `LoggedHardware.refreshAll()`**, which [Robot.java](../../src/main/java/frc/robot/Robot.java)
   calls once at the top of the loop. Reading a motor in a constructor gives 0 — that runs before
   the first refresh.
6. **A log must contain a disabled → enabled transition.** OpModes schedule their commands on that
   edge; a log that is already enabled on its first entry replays with no commands running at all.
   `SimStartup` stays disabled briefly for exactly this reason.

## When replay stops matching

`replayCheck` names the key and the first bad timestamp. Work backwards:

- Key missing from `ReplayOutputs` entirely → the code path never ran. Usually a command that never
  started (rule 6) or one that never finished (rule 3).
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
