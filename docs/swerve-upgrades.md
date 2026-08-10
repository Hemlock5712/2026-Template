# Plan: Orbit-style swerve upgrades

> **Status (2026-08-10):** research + ranking only; nothing here is implemented. The replay
> groundwork **is** shipped on branch `device-bringup` — see the `run-replay` skill.
>
> **This is the second version.** The first was written partly from a third-party reimplementation
> and misattributed several techniques to Orbit that are not theirs. Everything below is tagged by
> source tier.

## Sources

**T1 (Orbit's own words)** — two 2024 "Orbit Sessions" talks by 1690 students Itay Naumann, Yan
Vazan and Ido Zipori:

- **Part I — Swerve & System Control** (`vUtVXz7ebEE`, 58 min) — module control, the acceleration
  limiter, controller-choice rules. *This is where the swerve content actually is.*
- **Part II — PoseTracker & Shoot on the Move** (`N6ogT5DjGOk`, 75 min) — odometry, FOM fusion, skid
  and collision detection.
- Both slide decks (35 and 43 pages), downloaded from the Google Slides URLs 1690 members posted
  themselves on Chief Delphi, and both auto-caption transcripts.

**1690 have published no robot code.** Verified three ways: `github.com/orbit1690` 404s;
`github.com/Team1690` holds 16 repos of scouting/dashboard/strategy tooling and no drivetrain; their
CD releases are CAD-only. So nothing below can be checked against an implementation, and they
consistently withhold tuned constants — the skid threshold, FOM growth rates and tilt limits are all
"a logical value" with no figure.

Anything marked **T3** is inference or generic FRC practice, *not* Orbit.

---

## 1. Does Orbit move their modules in a circular path when rotating?

**No — and the premise is inverted: CTRE does more of this than Orbit does.**

Orbit's swerve is slides 20–32 of Part I, and it is textbook first-order kinematics:

- Slide 21: *"Module control: Steering – Position control / Drive – Velocity control"* — position-only
  steering, **no azimuth velocity feedforward**.
- Slide 22: *"Divide the movement to 2 vectors: Rotational vel = wanted Omega \* radius, perpendicular
  to radius; Translation vel; Module vel = Rotational vel + Translation vel."*
- Slide 24: uniform velocity desaturation.

The words *arc*, *circle*, *second order*, *discretize*, *skew* and *steering feedforward* appear
**nowhere** in either deck or transcript.

**Where the belief probably comes from (T3):** slide 22's "Omega × radius, perpendicular to radius"
describes a velocity tangent to a circle about the robot centre. "Modules move in a circular path
when rotating" is a fair paraphrase of that slide — but it is the standard decomposition every swerve
library performs, CTRE and WPILib included. It is not a differentiator.

### What CTRE actually does — verified in their published C++

Two separate effects get conflated here. CTRE handles both.

**Chassis discretization** — holding (vx, vy, ω) constant for `dt` traces an arc, so naive inverse
kinematics skews. `ChassisVelocities.discretize(dt)` is WPILib's fix. CTRE calls it internally, but
**not for every request**:

| Request | Discretizes | Used by |
| --- | --- | --- |
| `FieldCentric` (C++ `swerve_request.hpp:265`) | **yes** | `TeleopOpMode` |
| `ApplyFieldVelocity` (`:1399`) | **yes** | `DriveToPose`, `FollowTrajectory` |
| `ApplyRobotVelocity` (`:1041`) | **no** | `DriveDistance`, `DriveToTag` |

CTRE's own doc comment on `ApplyRobotVelocity`: *"Unlike the field-centric requests, this request does
not automatically discretize the provided ChassisVelocities. Users must manually discretize the
velocity if appropriate."*

**Module azimuth feedforward** — `ModuleRequest.UpdatePeriod`: *"Setting this to a non-zero value adds
a velocity feedforward to the steer motor."* Every built-in request passes
`.WithUpdatePeriod(parameters.updatePeriod)`, so it is on everywhere. Orbit have no equivalent.

### How much does it cost? (0.6 m wheelbase, v = 4 m/s, ω = 6 rad/s)

| | dt = 20 ms | dt = 4 ms |
| --- | --- | --- |
| Skew angle (ω·dt/2) | 3.44° | 0.69° |
| Cross-track drift rate | **0.24 m/s** | 0.048 m/s |
| Drift over a 2 s move | **48 cm** | 9.6 cm |
| Speed shortfall | 0.06% | 0.002% |

The error is a *direction* error, so it accumulates. **Which column applies depends on who does the
kinematics:** 4 ms if you hand a chassis velocity to a native CTRE request (re-linearised on the
250 Hz odometry thread), 20 ms if you run inverse kinematics yourself in a 50 Hz loop. That second
column is the classic "my swerve skews when I spin" complaint — and CTRE's architecture keeps us out
of it.

**One asymmetry worth knowing:** for a **robot-centric** constant command the azimuth rate is exactly
**zero** — the module vectors are fixed in the body frame, so the steer targets never move and
position-only steering is perfect. The azimuth-lag effect only exists for **field-centric** commands,
where the translation vector rotates at −ω in the body frame.

**Only real gap in this repo:** `DriveToTag` uses `ApplyRobotVelocity` *and* commands a real ω, so it
is the one place CTRE's "discretize it yourself" note applies. At alignment speeds (ω ≈ 1 rad/s,
v ≈ 0.5 m/s) that is ~1 mm/s — a correctness fix, not a match-loser. `DriveDistance` commands ω = 0,
where discretization is a no-op.

## 2. Orbit's real differentiator: the acceleration limiter

Slides 25–28 of Part I, and the thing worth actually taking:

```
wantedAcc     = (finalWantedVel - currentVel) / cycleTime
wantedAcc     = accLimits(wantedAcc)          // three limits, below
nextWantedVel = currentVel + wantedAcc * cycleTime
```

Three limits, applied to the **whole-robot chassis velocity vector, not per module.** Orbit say so
out loud, and volunteer that it is approximate (Part I Q&A, 54:25): *"are the swerve acceleration
limits applied per module basis or for the entire robot — so we are doing it for the entire robot,
which is not accurate, which is not accurate, but it's close enough to accurate and it works for us,
and gives us a smooth motion."*

| Limit | Formula (T1) | Their tuning procedure (T1) |
| --- | --- | --- |
| Forward | `maxForwardAcc = maxAcc * (1 - currentVel/maxVel)` | *"Set to a logical value (around 10 m/s²), increase maxAcc until robot can't follow wanted velocity"* — judged by plotting commanded vs actual |
| Tilt | per-axis cap | *"Set to logical value (depending on your CG), increase until robot starts to tip"* |
| Skid | cap on lateral accel | increase until the wheels break loose |

Note what is **not** there: no motor Kt, no stall torque, no robot mass, no µ, no friction circle.
The forward limit is one line with an empirically tuned `maxAcc`. The torque-speed *motivation* is
theirs (*"motors provide more torque the slower they spin"*); the motor model is not.

**The load-bearing principle, which the first version of this doc missed entirely** (Part I Q&A,
51:59–52:16). Asked whether the skid limit replaces the motor current limit: *"the answer is no, we do
both… if there is already big current then it's too late, so we want to avoid that by limiting the
requested velocity before the current is higher, so this is the reason that we do all the limits and
all the physical calculations as a feed forward mechanism and not as a feedback mechanism."*

So: **all limits are feedforward, and current limits stay alongside them, not replaced by them.** That
answers a question the first version left open — a measured `kSlipCurrent` does not substitute for the
acceleration limiter.

**It belongs in one place, not just teleop.** Auto and teleop enter the same algorithm — CD 369008 #1:
*"This same algorithm is used both in teleop driving and in autonomous"*, and #21 describes feeding a
path profile in as *"Vx, Vy, Omega (same as field centric joystick inputs)"*. Putting a limiter only
between the sticks and the request would be a half-port.

## 2b. Settled: do not ask CTRE for Motion Magic on the drive motor

Researched 2026-08-10 before filing a feature request. **Don't file it** — CTRE has answered this
publicly four times, and the most recent is 2026-03-23.

The exact request was already made (CD 483669, 2025-01-26) by a team with *our* motivation — their
robot lifted at the back under hard deceleration because of arm position. `bhall-ctre`:

> *"We do not support using Motion Magic® Velocity on the drive motors because it doesn't make much
> sense to control acceleration on a per-module basis. Doing so has the potential to introduce things
> like drift in robot heading. For things like preventing the robot from tipping, you instead want to
> limit whole-robot acceleration by doing something like apply a slew rate limiter to the inputs."*

Same answer for the obvious workaround (CD 484408): *"We recommend against using ramp rates on the
individual drive motors because you want to limit whole-robot acceleration, not per-module
acceleration."* So `ClosedLoopRampsConfigs` — which swerve does **not** overwrite, and which therefore
works today — is not a loophole; it's the same mistake by another name.

And the one argument that seemed to refute them, "but `SlipCurrent` already limits acceleration per
module", they addressed directly: *"the purpose of the constant is to avoid wheel slip, not limit
whole-robot acceleration."* Slip current is a per-module **slip guard**; acceleration policy belongs
to the chassis. Two different jobs.

Three more reasons it would not have worked anyway:

- **It bounds the reference, not the torque.** Delivered current is still `kP·(v_ref − v_meas)`,
  unbounded in the error — and our `driveGains` has **kA = 0**, so during a profiled ramp *all* the
  accelerating torque is manufactured by the P term. On our constants ~0.9 m/s of tracking error
  saturates the drive at slip current no matter what the profile is doing.
- **Module flip.** On a 180° optimization the drive target negates while the wheel's actual speed is
  continuous. A profile can't tell the difference and ramps across a `2v` step — ~0.4 s of pushing the
  wrong way at 2 m/s. The four modules don't flip together, so the residue is yaw drift. That *is* the
  heading drift CTRE names.
- **The trend runs the other way.** `DriveRequestType` has had exactly two values in every season. The
  only churn was on the **steer** side, which dropped trapezoidal Motion Magic in 2025, added
  unprofiled `Position`, and made `Position` the default.

Every team found running `MotionMagicVelocity` on a drive motor has abandoned
`com.ctre.phoenix6.swerve` entirely for their own `ModuleIO`.

**CTRE's sanctioned hook for external acceleration authority is wheel force feedforwards** —
`ModuleRequest.WheelForceFeedforwardX/Y`, added at PathPlanner's request (Phoenix-Releases #80), plus
the 2026 `WheelForceCalculator`. Profile the chassis, push per-module *force* down. That is §2's
architecture, and it is what to build.

One caution on doing that, from the same engineer (2026-03-23): a chassis slew limiter *"is not
strictly necessary or even desirable for many teams… It affects how the robot drives, which many
drivers do not like ('the robot feels like it's on ice')… the slip current limit is often sufficient
and less intrusive."* Orbit's version is friction- and motor-aware rather than a naive slew limiter,
so it earns its keep — but measure `kSlipCurrent` first and see how much is left to fix.

## 3. Ranked

Attribution is now explicit. **[O]** = Orbit's, **[G]** = generic FRC practice.

| # | Technique | Effort | Verdict |
| --- | --- | --- | --- |
| 1 | **[O]** Acceleration limiter: 3 feedforward limits on chassis velocity, one entry point for auto + teleop | M | **do now** |
| 2 | **[G]** Wheel-radius + free-speed calibration | S | **do now** |
| 3 | **[O]** Closed-loop **velocity** control on the drive motors | S | **do now** |
| 4 | **[O]** `V² = Vf² − 2a·Δx` for DriveTo / RotateTo closure | S | **do now** |
| 5 | **[O]** Skid detection by max/min module translational ratio, logged | S | **do now** |
| 6 | **[O]** Regime-based controller choice (profile vs position vs velocity) | S | do later |
| 7 | **[G]** Log drive stator current + Pigeon accel/yaw rate | S | do later |
| 8 | **[O]** One pose-confidence scalar that actions gate on | S | do later |
| 9 | **[O]** Collision detection that suspends the limits | M | do later |
| 10 | **[O]** Chassis-acceleration feedforward for the arm | S | do later |
| 11 | **[O]** Additive assist architecture | M | do later |
| 12 | **[O]** Manual discretize on `ApplyRobotVelocity` in `DriveToTag` | S | correctness fix |
| 13 | **[G]** Jerk limiting | S | skip |
| 14 | **[O]** Slip-rejecting odometry (drop skidding modules) | — | **skip — see §4** |
| 15 | **[O]** FOM fusion replacing the Kalman filter | L | skip |
| 16 | **[G]** Arc-based per-module odometry integration | M | skip |
| 17 | **[G]** Friction-circle path profiling | — | already done offline by the path generator |
| 18 | **[O]** Trig AprilTag pose estimation replacing MegaTag | M | skip |
| 19 | **[O]** Shoot-on-the-move | L–XL | skip |
| 20 | **[G]** Module desaturation + shortest-path azimuth | — | already have both |

### Three items the first version missed completely

**Rank 3 — velocity control on the drive motors.** Slide 21, and the reason (32:07): *"for the drive
itself we do velocity control because that's what we control at the end, it's the robot's velocity,
and it also helps us be able to tell how the robot drove."* Four of our five drive paths currently
request `DriveRequestType.OpenLoopVoltage` — `TeleopOpMode:39`, `DriveToPose:47`, `DriveDistance:47`,
`DriveToTag:46`. Cheapest Orbit item with the best fit here, and it is a precondition for trusting
`ModuleTargets` vs `ModuleVelocities` in a log.

**Rank 4 — `V² = Vf² − 2a·Δx`.** Slide 33, and the transcript's gloss: *"that's just kinematics"*, plus
*"it's similar to the Phoenix motion magic, but we're doing it on the robot's distance from the wanted
position."* One scalar relation used for both linear and angular closure — directly portable to
`DriveToPose`, `DriveDistance` and `DriveToTag`, all of which exist here already.

**Rank 6 — regime-based controller choice.** Slide 8: *"Motion Magic for big movements (between
constant setpoints) / Position for real-time updated setpoints / Velocity for constant speed."* They
apply it twice: to the azimuth (31:26 — they tried Motion Magic and measured it worse, *"because our
steering position just changes all the time"*) and to heading control (39:40 — profiled rotate-to in
auto, plain kP in teleop *"because we have the rapid changes"*). **Never profile a setpoint that is
recomputed every cycle** is a reusable rule this template can state once and follow.

## 4. Skid detection, and why rank 14 is still a skip

Detection is worth having, and it needs **no new logging** — `Drivetrain/ModuleVelocities` is already
an input at 50 Hz. Subtract the rotation-only component as **vectors**, take `max|t| / min|t|`.

Two guards, both confirmed against a real log: gate on a minimum translational speed (computing this
offline on `akit_26-08-09_23-13-53.wpilog` gave median 1.000 but spikes to **1.31 at 0.55 m/s** — steer
lag, not slip, and a permanent false positive if ungated), and subtract vectors rather than speeds.
Two blind spots to state out loud: it is a **relative** metric, so all four wheels breaking loose
together reads 1.0; and a uniform sideways slide is invisible, because a module reports only
along-wheel rolling speed.

**Acting on it by dropping modules from the odometry average is still a skip**, but the first version
was incomplete about why. The centroid objection stands: averaging three of four modules integrates a
centroid offset ~0.2 m from robot centre, fabricating ω×offset ≈ **1.2 m/s at 6 rad/s**. What was
missing is that **Orbit never let that average stand alone** — in the same cycle they inflate the
odometry FOM, which under their 1/FOM² weighting collapses odometry's weight and hands authority to
the cameras (*"this is how we actually say that we don't trust our odometry"*), and heading always
comes from the gyro, never from the wheels.

Our equivalent, and the reason this stays a skip for us specifically: WPILib's
`SwerveDrivePoseEstimator` cannot vary odometry trust at runtime — odometry is its process model. But
**CTRE's native estimator can**, via `setStateStdDevs`, whose own javadoc says *"This might be used to
change trust in odometry after an impact with the wall or traversing a bump."* So the Orbit-shaped
move available to us is: detect skid → inflate `setStateStdDevs` → let vision take over. That is rank
8's territory, and it needs no module dropping at all.

## 5. Testing slip without a slip-capable sim

CTRE's `SimSwerveDrivetrain` javadoc: *"assumes that the swerve drive is perfect, meaning that there
is no scrub and the wheels do not slip."* Odometry error in sim is structurally zero — the pose is
forward kinematics of the commanded module states. maple-sim would fix it but its newest release
(v0.4.0-beta, Jan 2026) targets WPILib 2026.2.1; there is no 2027 build.

**But Orbit's own tuning procedure needs no sim, no current logging and no µ** — it is three carpet
tests, each "increase the limit until the robot misbehaves", judged from plotted commanded-vs-actual
velocity. That is a stronger argument than the first version's "you need current logging first":

1. **A JUnit test on the pure function** — the skid ratio depends on four `SwerveModuleVelocity`
   values and the kinematics. Assert the ratio, the flagged module, and the two blind spots.
2. **Replay a log with one module's speed scaled** — reproduces a skid on real recorded data.
3. **Carpet, per Orbit's procedure** — the only place `maxAcc`, the tilt limit and the skid threshold
   can actually come from.

## 6. Other replay holes found on the way

- **`FollowTrajectory` has never been through a replay** — no `FollowTrajectory/*` key exists in any log
  on disk. Record one with `FollowPathOpMode` before trusting it. (Orbit's auto structure in CD 369008
  #21 — profile feedforward + P on position and heading, with vision gradually taking over — is the
  primary-source model for what this command should become.)
- **`LoggedTalonFX` logs only the request's name**, not its target value. You can see that a Motion Magic
  request happened, not where it was aiming.
- **Two `replayCheck` blind spots:** a key that stops being written only prints "(only in recording, not
  replayed)" and still **passes**; a newly added key is never compared at all.
- **`DriveToTag` now replays** (it read NetworkTables directly until the Limelight inputs were expanded).
