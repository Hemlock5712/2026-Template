"""Where did the auto spend its time, and was the arm ready?

    python tools/auto_report.py logs/akit_<stamp>.wpilog           # one recording
    python tools/auto_report.py logs/akit_<stamp>_replay.wpilog    # recording vs. replay, side by side

Reads the robot's own log keys (see the log-reading skill). Needs `pip install robotpy-wpiutil`.
"""

import struct
import sys

from wpiutil.log import DataLogReader

KRAKEN_FREE_SPEED_RPS = 100.0  # Kraken X60, 6000 rpm
ARM_GEAR_RATIO = 50.0  # Arm.GEAR_RATIO - keep in sync


def read(path):
    """{key: [(t, value)]} for the handful of keys this report uses."""
    names = {}
    series = {}
    for r in DataLogReader(path):
        if r.isStart():
            d = r.getStartData()
            names[d.entry] = (d.name, d.type)
            continue
        if r.isFinish() or r.isControl() or r.isSetMetadata():
            continue
        name, typ = names.get(r.getEntry(), ("", ""))
        t = r.getTimestamp() / 1e6
        if typ == "double":
            v = r.getDouble()
        elif typ == "int64":
            v = r.getInteger()
        elif typ == "boolean":
            v = r.getBoolean()
        elif typ == "string":
            v = r.getString()
        elif typ == "struct:ChassisVelocities":
            vx, vy, _ = struct.unpack("<ddd", bytes(r.getRaw()))
            v = (vx * vx + vy * vy) ** 0.5
        else:
            continue
        series.setdefault(name, []).append((t, v))
    return series


def value_at(points, t):
    """Zero-order hold: the last value written at or before t."""
    last = None
    for ts, v in points:
        if ts > t:
            break
        last = v
    return last


def steps(series, prefix):
    """[(name, start, end)] from Auto/Step if present, else from the drive request changing."""
    marks = series.get(prefix + "Auto/Step")
    if not marks:
        # No markers in this log: infer steps from the drivetrain being commanded or idle.
        marks = [
            (t, "drive" if v != "Idle" else "arm")
            for t, v in series.get(prefix + "Drivetrain/Request", [])
        ]
    out = []
    for i, (t, name) in enumerate(marks):
        next_t = marks[i + 1][0] if i + 1 < len(marks) else t  # last marker is a point
        out.append((name, t, next_t))
    return out


def report(series, prefix):
    arm = series.get("/RealOutputs/Arm/AngleDegrees", [])  # an input-derived key: same in both
    speed = series.get("/RealOutputs/Drivetrain/TranslationSpeedMps", [])
    print(f"\n{prefix}  steps:")
    print(f"  {'step':<8} {'start':>7} {'end':>7} {'secs':>6}  arm at start  robot moving?")
    for name, start, end in steps(series, prefix):
        angle = value_at(arm, start)
        moving = any(v > 0.05 for t, v in speed if start <= t <= end)
        stowed = series.get(prefix + "Auto/ArmStowedAtStepStart")
        flag = value_at(stowed, start) if stowed else None
        stowed_txt = "" if flag is None else ("stowed" if flag else "NOT stowed")
        print(
            f"  {name:<8} {start:7.3f} {end:7.3f} {end - start:6.3f}  "
            f"{angle if angle is None else round(angle, 1):>6}  {stowed_txt:<10}  "
            f"{'yes' if moving else 'no'}"
        )
    return steps(series, prefix)


def main(path):
    series = read(path)
    enabled = [t for t, v in series.get("/DriverStation/Enabled", []) if v]
    if not enabled:
        sys.exit("robot never enabled in this log")
    t0 = enabled[0]
    print(f"{path}\nenabled at {t0:.3f} s")

    real = report(series, "/RealOutputs/")
    if any(k.startswith("/ReplayOutputs/") for k in series):
        replay = report(series, "/ReplayOutputs/")
        print("\nreplay vs recording (same log, this build of the code):")
        for (name, rs, re_), (_, ps, pe) in zip(real, replay):
            print(f"  {name:<8} starts {ps - rs:+.3f} s, lasts {(pe - ps) - (re_ - rs):+.3f} s")

    # Idle time: robot stopped while the arm was still moving.
    speed = series.get("/RealOutputs/Drivetrain/TranslationSpeedMps", [])
    arm_vel = series.get("/Hardware/TalonFX/Arm/VelocityRps", [])
    idle = 0.0
    for i in range(1, len(arm_vel)):
        t, v = arm_vel[i]
        if t > t0 and abs(v) > 0.02 and (value_at(speed, t) or 0.0) < 0.05:
            # ponytail: records land only on change, so cap the gap instead of carrying a long
            # standstill forward as motion.
            idle += min(t - arm_vel[i - 1][0], 0.05)
    total = (real[-1][1] if real else series["/Timestamp"][-1][0]) - t0
    print(f"\nrobot stationary while the arm moved: {idle:.2f} s of a {total:.2f} s auto")

    # Motor headroom: how hard did the arm motor actually work?
    volts = [abs(v) for t, v in series.get("/Hardware/TalonFX/Arm/MotorVoltage", []) if t > t0]
    peak_mech_rps = max((abs(v) for t, v in arm_vel if t > t0), default=0.0)
    if volts:
        print(
            f"arm motor peak: {max(volts):.1f} V of 12 V, "
            f"rotor {peak_mech_rps * ARM_GEAR_RATIO:.0f} of {KRAKEN_FREE_SPEED_RPS:.0f} rps free speed"
        )


if __name__ == "__main__":
    if len(sys.argv) != 2:
        sys.exit(__doc__)
    main(sys.argv[1])
