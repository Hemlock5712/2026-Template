# Demo: replay + AI on one auto run

Two demos, one log. Record the default auto ("3 - Drive Stow Drive") once:

```bash
./gradlew simulateJavaAgent -PstopAfter=20     # writes logs/akit_<stamp>.wpilog
```

Both demos work on any log of that auto, including one already in `logs/`.

## Demo 1 — AI: "where is this auto wasting time?"

Ask the AI to read the log. The report tool is what it reaches for:

```bash
python tools/auto_report.py logs/akit_<stamp>.wpilog
```

```
  step       start     end   secs  arm at start  robot moving?
  drive      3.879   5.523  1.644   -10.0              yes
  arm        5.523   6.068  0.545   -10.0              no
  drive      6.068   7.484  1.415    89.0              yes

robot stationary while the arm moved: 0.60 s of a 3.61 s auto
arm motor peak: 6.7 V of 12 V, rotor 50 of 100 rps free speed
```

What the AI finds from that (measured on `akit_26-09-04_04-11-34`):

- **15 % of the auto is spent standing still** waiting for the arm. Leg 1 drives with the arm
  hanging at -10°, then the robot stops to stow. Fix: stow *during* leg 1 (`Command.race(leg1,
  stow)`), and the wait between legs disappears.
- **The arm was NOT stowed when leg 1's path ended.** Arm at -10°, the sim's low hard stop.
- **The arm motor has headroom.** Peak 6.7 V of 12, rotor at half its free speed. The move is
  limited by `MotionMagicAcceleration = 4`, not by the motor or the 50:1 gearing. Raising the
  acceleration is free speed; a different gear ratio is not needed for this move.
- Bonus catch: the arm reads -10°, not 0°, at enable. Brake mode does not exist in sim, so the arm
  falls to its stop while disabled. On a real robot that number tells you whether brake mode works.

None of this needed replay. It needed someone to read 80 keys and do arithmetic - that is the AI.

## Demo 2 — Replay: "what would the robot have decided?"

Replay re-runs *our code* on the recorded sensor values. Two things it can do that nothing else can:

### 2a. Add a measurement to a run that already happened

The auto now stamps `Auto/Step` and `Auto/ArmStowedAtStepStart` at the start of every step
(`mark(...)` in [DriveStowDriveOpMode](../src/main/java/frc/robot/opmodes/DriveStowDriveOpMode.java)).
Replay a log recorded **before** that code existed:

```bash
./gradlew replayJava -Preplay=logs/akit_<old>.wpilog
python tools/auto_report.py logs/akit_<old>_replay.wpilog
```

```
/RealOutputs/  steps:            <- the recording: no step names, inferred from the drive
  drive      3.879   5.523 ...
/ReplayOutputs/  steps:          <- same log, this build: the keys appear
  leg 1      3.879   5.523  1.644   -10.0  NOT stowed  yes
  stow       5.523   6.068  0.545   -10.0  NOT stowed  no
  leg 2      6.068   7.484  1.415    89.0  stowed      yes
```

The robot never ran the new code. The keys are computed from the logged arm encoder and the
logged clock. In AdvantageScope: `ReplayOutputs/Auto/Step` exists, `RealOutputs/Auto/Step` does not.

### 2b. Change a rule and see the different decision on the same data

"Stowed" means within 1° and Motion Magic finished. Suppose "above 75° is safe to drive". Change
one line and replay - no robot, no re-run:

```java
robot.stow().until(() -> robot.arm.getAngleDegrees() > 75).named("stow until stowed"),
```

```
replay vs recording (same log, this build of the code):
  stow     starts +0.000 s, lasts -0.110 s
  leg 2    starts -0.110 s
```

Leg 2 would have started 110 ms sooner on that exact run. Graph `RealOutputs/Drivetrain/Request`
against `ReplayOutputs/Drivetrain/Request` and the step is visible.

**Say the limit out loud:** replay re-runs decisions, not physics. It shows leg 2 *starting*
earlier; it cannot show the robot *arriving* earlier, because the wheels in the log did what they
did. Put the change on the robot, record again, and Demo 1 measures whether it paid off.

## Proving replay is honest

```bash
./gradlew replayCheck     # replays the newest log and diffs every output key
```

With the code unchanged, `RealOutputs` and `ReplayOutputs` must match exactly. That is the
guarantee behind 2a and 2b. See the `run-replay` skill.

## More demos, once there is a real-robot log

Sim has no camera and no wheel slip, so these need a match log. The keys are already logged
(`Hardware/Limelight/<cam>/*` holds every tag, ambiguity and distance per frame), so each one is a
code change plus `replayJava`:

| Question | Replay or AI | How |
| --- | --- | --- |
| Are our vision std devs right? | Replay | Change `XY_STD_DEV` / `DISTANCE_EXPONENT` in `Vision.java`, replay, compare `ReplayOutputs/Drivetrain/EstimatedPose` to the tag poses. Smallest residual wins. |
| Is the wheel diameter wrong? | AI, then replay | Odometry distance between two vision fixes vs. vision distance. A constant ratio is wheel radius; fix `kWheelRadius`, replay, error goes flat. |
| Should the std-dev formula use ambiguity or tag span? | Replay | The library logs both. Write the new `deviation()`, replay every match log, pick the one with the smallest pose jumps. |
| Are we skidding on that hard turn? | AI | `Drivetrain/SkidRatio` against `CommandedVelocity`. Sim has no slip, so this only means something on carpet. |
| Which step wastes the most time across a whole event? | AI | `tools/auto_report.py` on every match log; sort steps by `secs`. |
