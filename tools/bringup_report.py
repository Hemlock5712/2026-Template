"""Reads a bring-up .wpilog and prints what every mechanism in it measured.

    python tools/bringup_report.py [log.wpilog]

Defaults to the newest log in ./logs. Needs `pip install robotpy-wpiutil`.
Record one first with:  ./gradlew simulateJavaAgent '-Pmode=utility:Bring-Up' -PstopAfter=14

Mechanisms are discovered from the log, not hardcoded - anything BringUp measured shows up, and
whether it is an arm, an elevator or a flywheel is worked out from what it did, not declared.
See the device-bringup skill for what the numbers mean.
"""

import glob
import math
import os
import struct
import sys

from wpiutil.log import DataLogReader

STILL_RPS = 0.02  # below this the mechanism counts as holding, not moving
MIN_DWELL_S = 0.5  # ignore momentary pauses
SETTLE_RPS = 0.2  # below this much change per loop, a wheel counts as up to speed
COARSE_TRAVEL_ROT = 0.1  # under this, say how rough the ratio is rather than implying precision

ENABLED = "/DriverStation/Enabled"
BRINGUP = "/RealOutputs/BringUp/"
RATIO = "/MeasuredRatio"
# Which control request was last sent - says whether a run was open loop or closed loop.
REQUEST = "/RealOutputs/Hardware/TalonFX/"


def entry_names(path):
    """Every key in the log, so mechanisms can be discovered rather than hardcoded."""
    names = {}
    for record in DataLogReader(path):
        if record.isStart():
            start = record.getStartData()
            names[start.name] = start.type
    return names


def read(path, keys):
    """Returns {key: [(t, value)]} for the double/boolean keys asked for."""
    entries = {}
    for record in DataLogReader(path):
        if record.isStart():
            start = record.getStartData()
            entries[start.entry] = (start.name, start.type)

    series = {key: [] for key in keys}
    for record in DataLogReader(path):
        if record.isStart() or record.isFinish() or record.isControl() or record.isSetMetadata():
            continue
        name, kind = entries.get(record.getEntry(), ("", ""))
        if name not in series:
            continue
        raw = bytes(record.getRaw())
        if kind == "double" and len(raw) == 8:
            series[name].append((record.getTimestamp() / 1e6, struct.unpack("<d", raw)[0]))
        elif kind == "boolean" and len(raw) == 1:
            series[name].append((record.getTimestamp() / 1e6, struct.unpack("<?", raw)[0]))
        elif kind == "string":
            series[name].append((record.getTimestamp() / 1e6, raw.decode("utf-8", "replace")))
    return series


def resample(series, keys, step=0.02):
    """Step-hold every key onto a common time grid - the log only records on change."""
    times = sorted({t for key in keys for t, _ in series.get(key, [])})
    if not times:
        return []
    cursors = {key: 0 for key in keys}
    held = {key: 0.0 for key in keys}
    grid = []
    t = times[0]
    while t <= times[-1]:
        for key in keys:
            samples = series.get(key, [])
            while cursors[key] < len(samples) and samples[cursors[key]][0] <= t:
                held[key] = samples[cursors[key]][1]
                cursors[key] += 1
        grid.append((t, dict(held)))
        t += step
    return grid


def spans(grid, predicate, min_seconds=MIN_DWELL_S):
    """Contiguous runs of grid rows where predicate holds, at least min_seconds long."""
    out, run = [], []
    for row in grid:
        if predicate(row[1]):
            run.append(row)
        else:
            if run and run[-1][0] - run[0][0] >= min_seconds:
                out.append(run)
            run = []
    if run and run[-1][0] - run[0][0] >= min_seconds:
        out.append(run)
    return out


def mean(rows, key):
    return sum(row[1][key] for row in rows) / len(rows)


def last(series, key, default=None):
    values = series.get(key) or []
    return values[-1][1] if values else default


def ratio_report(series, name):
    """Gear ratio, or - for a follower named "<Leader>/n" - direction against its leader."""
    ratio = last(series, f"{BRINGUP}{name}{RATIO}", float("nan"))
    travel = max(
        (abs(v) for _, v in series.get(f"{BRINGUP}{name}/SensorTravelRot", [])), default=0.0
    )

    if math.isnan(ratio):
        print(f"  no ratio yet: only {travel:.3f} rot of travel. Move it further.")
        return

    if "/" in name:
        leader = name.rsplit("/", 1)[0]
        print(
            f"  follows {leader} at {ratio:+.3f}"
            f"  ({'SAME direction as' if ratio > 0 else 'OPPOSED to'} the leader)"
        )
        if abs(abs(ratio) - 1.0) > 0.05:
            print("    NOT +-1 - a follower's rotor should track its leader's turn for turn")
        return

    print(f"  rotor:mechanism ratio = {ratio:.2f}   (measured over {travel:.2f} rot)")
    if travel < COARSE_TRAVEL_ROT:
        # Encoder resolution divided by travel - fine for 50 vs 25, not for 50 vs 49.
        print(f"    coarse: good to about {100 / (travel * 4096):.0f}%. Move it further to sharpen.")


def zero_and_gravity(series, name):
    """Magnet offset, and kG for whichever gravity type the holding voltages actually fit."""
    angle = f"/Hardware/CANcoder/{name}/PositionRot"
    volts = f"/Hardware/TalonFX/{name}/AppliedVolts"
    speed = f"/Hardware/TalonFX/{name}/VelocityRps"

    absolute = last(series, f"/Hardware/CANcoder/{name}/AbsolutePositionRot")
    if absolute is not None:
        # AbsolutePosition already has MagnetOffset applied, so negating it is the delta that
        # makes this pose read zero.
        print(f"  to zero the CANcoder where it stopped, ADD {-absolute:+.4f} to MagnetOffset")

    # At each pose the mechanism is still, so the holding voltage is pure gravity feedforward.
    # Disabled it sags onto a hard stop at 0 V, which is not a holding voltage.
    grid = resample(series, [ENABLED, angle, volts, speed])
    holds = spans(grid, lambda row: row[ENABLED] and abs(row[speed]) < STILL_RPS)
    if not holds:
        print("  no still-and-powered pose - kG needs it holding position")
        return

    poses = []
    print("  holding voltage by pose:")
    for rows in holds:
        settled = rows[len(rows) // 2 :]  # drop the approach, keep the settled half
        rot = mean(settled, angle)
        volt = mean(settled, volts)
        poses.append((rot, volt))
        print(f"    {rot:+.4f} rot ({360 * rot:6.1f} deg)   {volt:+.3f} V")
    if len(poses) < 2:
        print(f"  only one pose held - kG is {poses[0][1]:.3f} if this is an elevator")
        return

    # An arm's holding voltage follows cos(angle); an elevator's is the same at every height.
    # Fit both and let the residuals say which mechanism this is - that catches Arm_Cosine set on
    # an elevator, and Elevator_Static set on an arm.
    cosines = [math.cos(2 * math.pi * rot) for rot, _ in poses]
    denominator = sum(c * c for c in cosines)
    cosine_kg = sum(v * c for (_, v), c in zip(poses, cosines)) / denominator if denominator else 0.0
    constant_kg = sum(v for _, v in poses) / len(poses)

    cosine_error = sum((v - cosine_kg * c) ** 2 for (_, v), c in zip(poses, cosines))
    constant_error = sum((v - constant_kg) ** 2 for _, v in poses)

    if cosine_error <= constant_error:
        print(f"  kG = {cosine_kg:.3f}  (fits V = kG*cos(angle) - an ARM, GravityType Arm_Cosine)")
    else:
        print(f"  kG = {constant_kg:.3f}  (same volts at every pose - an ELEVATOR, Elevator_Static)")


def open_loop_report(grid, volts, speed):
    """Direction and kV from a held-voltage run: kV is the slope of volts against rps."""
    # Group the settled rows by voltage step. Rounding to 0.1 V is coarser than the steps a
    # bring-up OpMode holds and finer than the gap between them.
    levels = {}
    for t, row in grid:
        if abs(row[volts]) > 0.25:
            levels.setdefault(round(row[volts], 1), []).append((t, row))

    points = []
    for level in sorted(levels):
        rows = levels[level]
        if len(rows) * 0.02 < MIN_DWELL_S:
            continue
        settled = rows[len(rows) // 2 :]  # drop the spin-up, keep the settled half
        points.append((mean(settled, speed), mean(settled, volts)))

    if not points:
        print("  never commanded to spin - nothing to measure")
        return

    print("  open-loop steps:")
    for rps, volt in points:
        per = f"{volt / rps:.4f}" if rps else "n/a"
        print(f"    {volt:+.3f} V  ->  {rps:+8.2f} rps   (volts/rps {per})")

    rps, volt = points[-1]
    print(f"  direction: {'ok' if volt * rps > 0 else 'BACKWARDS - flip MotorOutput.Inverted'}")
    if len(points) < 2:
        print(f"  only one step held - kV is {volt / rps:.4f} if kS is zero")
        return

    # Least squares on V = kS + kV*rps. The intercept absorbs kS, so the slope is a cleaner kV
    # than any single point's volts/rps.
    n = len(points)
    mean_rps = sum(r for r, _ in points) / n
    mean_volt = sum(v for _, v in points) / n
    spread = sum((r - mean_rps) ** 2 for r, _ in points)
    kv = sum((r - mean_rps) * (v - mean_volt) for r, v in points) / spread if spread else 0.0
    print(f"  kV (slope over {n} steps) = {kv:.4f}   <- put this in Slot0.kV")
    print(f"  kS (intercept) = {mean_volt - kv * mean_rps:+.4f}")


def velocity_report(series, name):
    """Direction and kV, for a motor run under closed-loop velocity."""
    volts = f"/Hardware/TalonFX/{name}/AppliedVolts"
    speed = f"/Hardware/TalonFX/{name}/VelocityRps"
    reference = f"/Hardware/TalonFX/{name}/ClosedLoopReference"

    requests = {value for _, value in series.get(f"{REQUEST}{name}/Request", [])}

    grid = resample(series, [volts, speed, reference])

    # Prefer an open-loop run whenever the log has one: it is the only way to measure kV without
    # already trusting a kV. The closed-loop reference cannot tell the two apart - Motion Magic
    # ramps its setpoint down from whatever the wheel was doing, so a plain stop() leaves a
    # reference as high as the last speed reached.
    if "VoltageOut" in requests:
        open_loop_report(grid, volts, speed)
        return

    target = max((row[reference] for _, row in grid), default=0.0)
    if target <= 0:
        print("  never commanded to spin - nothing to measure")
        return

    # Up to speed: commanded at the top speed and no longer accelerating.
    at_speed = [
        (t, row)
        for i, (t, row) in enumerate(grid)
        if i > 0
        and abs(row[reference] - target) <= 1e-6
        and abs(row[speed] - grid[i - 1][1][speed]) < SETTLE_RPS
    ]
    if not at_speed:
        print(f"  commanded {target:.1f} rps but never settled there")
        return

    rps = mean(at_speed, speed)
    volt = mean(at_speed, volts)
    print(f"  commanded {target:.2f} rps, settled at {rps:.2f} rps on {volt:+.3f} V")
    print(f"  direction: {'ok' if volt * rps > 0 else 'BACKWARDS - flip MotorOutput.Inverted'}")
    error = rps - target
    if abs(error) > 0.02 * abs(target):
        print(f"  off by {error:+.2f} rps ({100 * error / target:+.1f}%) - feedforward is mistuned")
    print(f"  measured volts per rps = {volt / rps:.4f}   <- start kV here, then re-run")


def selftest():
    """Feed open_loop_report a synthetic V = 0.1 + 0.12*rps run and check it recovers kV and kS."""
    volts, speed = "V", "rps"
    grid = []
    t = 0.0
    for step in (1.0, 2.0, 3.0):
        rps = (step - 0.1) / 0.12
        for i in range(100):  # 2 s at 20 ms; first half ramps, second half settled
            grid.append((t, {volts: step, speed: rps * min(1.0, i / 25)}))
            t += 0.02
    open_loop_report(grid, volts, speed)
    print("selftest: expected kV 0.1200, kS +0.1000")


def main():
    if "--selftest" in sys.argv:
        selftest()
        return
    path = sys.argv[1] if len(sys.argv) > 1 else None
    if path is None:
        logs = glob.glob(os.path.join("logs", "*.wpilog"))
        if not logs:
            sys.exit("No logs found. Record one with simulateJavaAgent -Pmode=utility:Bring-Up")
        path = max(logs, key=os.path.getmtime)
    print(f"log: {path}")

    names = entry_names(path)
    measured = sorted(
        key[len(BRINGUP) : -len(RATIO)]
        for key in names
        if key.startswith(BRINGUP) and key.endswith(RATIO)
    )
    motors = sorted(
        key[len("/Hardware/TalonFX/") : -len("/VelocityRps")]
        for key in names
        if key.startswith("/Hardware/TalonFX/") and key.endswith("/VelocityRps")
    )
    # A motor with no CANcoder and no leader was never ratio-measured; check its velocity loop.
    velocity_only = [m for m in motors if m not in measured]

    keys = [ENABLED]
    for name in measured + velocity_only:
        keys += [
            f"{BRINGUP}{name}{RATIO}",
            f"{BRINGUP}{name}/SensorTravelRot",
            f"/Hardware/CANcoder/{name}/PositionRot",
            f"/Hardware/CANcoder/{name}/AbsolutePositionRot",
            f"/Hardware/TalonFX/{name}/AppliedVolts",
            f"/Hardware/TalonFX/{name}/VelocityRps",
            f"/Hardware/TalonFX/{name}/ClosedLoopReference",
            f"{REQUEST}{name}/Request",
        ]
    series = read(path, keys)

    if not measured and not velocity_only:
        print("\nNothing to report - no TalonFX in this log.")
        return

    for name in measured:
        print(f"\n{name.upper()}")
        ratio_report(series, name)
        if series.get(f"/Hardware/CANcoder/{name}/PositionRot"):
            zero_and_gravity(series, name)

    for name in velocity_only:
        print(f"\n{name.upper()}")
        velocity_report(series, name)


if __name__ == "__main__":
    main()
