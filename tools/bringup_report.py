"""Reads a bring-up .wpilog and prints what the mechanisms measured.

    python tools/bringup_report.py [log.wpilog]

Defaults to the newest log in ./logs. Needs `pip install robotpy-wpiutil`.
Record one first with:  ./gradlew simulateJavaAgent '-Pmode=utility:Bring-Up' -PstopAfter=14
See the device-bringup skill for what the numbers mean.
"""

import glob
import math
import os
import struct
import sys

from wpiutil.log import DataLogReader

# Arm angles use the Arm_Cosine frame: 0 deg is straight out horizontally.
ARM_STILL_RPS = 0.02  # below this the arm counts as holding, not moving
MIN_DWELL_S = 0.5  # ignore momentary pauses
FLYWHEEL_SETTLE_RPS = 0.2  # below this much change the wheel counts as up to speed


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
    return series


def resample(series, keys, step=0.02):
    """Step-hold every key onto a common time grid - the log only records on change."""
    times = sorted({t for key in keys for t, _ in series[key]})
    if not times:
        return []
    cursors = {key: 0 for key in keys}
    held = {key: 0.0 for key in keys}
    grid = []
    t = times[0]
    while t <= times[-1]:
        for key in keys:
            samples = series[key]
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


ENABLED = "/DriverStation/Enabled"

ARM_KEYS = [
    ENABLED,
    "/RealOutputs/BringUp/Arm/BestMeasuredRatio",
    "/RealOutputs/BringUp/Arm/SensorTravelRot",
    "/RealOutputs/BringUp/Arm/MagnetOffsetDelta",
    "/RealOutputs/Arm/AngleDegrees",
    "/Hardware/TalonFX/Arm/AppliedVolts",
    "/Hardware/TalonFX/Arm/VelocityRps",
]
FLYWHEEL_KEYS = [
    "/Hardware/TalonFX/Flywheel/AppliedVolts",
    "/Hardware/TalonFX/Flywheel/VelocityRps",
    "/Hardware/TalonFX/Flywheel/ClosedLoopReference",
]


def arm_report(series):
    print("ARM")
    ratios = [v for _, v in series["/RealOutputs/BringUp/Arm/BestMeasuredRatio"] if not math.isnan(v)]
    travel = max((abs(v) for _, v in series["/RealOutputs/BringUp/Arm/SensorTravelRot"]), default=0.0)
    if not ratios:
        print(f"  no ratio: only {travel:.3f} rot of travel. Move the mechanism further.")
    else:
        print(f"  rotor:mechanism ratio = {ratios[-1]:.2f}   (measured over {travel:.2f} rot)")

    offsets = series["/RealOutputs/BringUp/Arm/MagnetOffsetDelta"]
    if offsets:
        print(f"  to zero the CANcoder where it stopped, ADD {offsets[-1][1]:+.4f} to MagnetOffset")

    # kG: at each pose the arm is still, so the holding voltage is pure gravity feedforward.
    # V = kG * cos(angle), least squares over every pose the sweep stopped at.
    # Disabled, the arm sags onto its hard stop at 0 V - that is not a holding voltage.
    grid = resample(series, ARM_KEYS)
    holds = spans(
        grid,
        lambda row: row[ENABLED] and abs(row["/Hardware/TalonFX/Arm/VelocityRps"]) < ARM_STILL_RPS,
    )
    if not holds:
        print("  no still-and-powered pose found - kG needs the arm holding position")
        return
    num = den = 0.0
    print("  holding voltage by pose:")
    for rows in holds:
        settled = rows[len(rows) // 2 :]  # drop the approach, keep the settled half
        angle = mean(settled, "/RealOutputs/Arm/AngleDegrees")
        volts = mean(settled, "/Hardware/TalonFX/Arm/AppliedVolts")
        cos = math.cos(math.radians(angle))
        num += volts * cos
        den += cos * cos
        print(f"    {angle:7.1f} deg   {volts:+.3f} V")
    if den > 1e-6:
        print(f"  kG (fit of V = kG*cos(angle) over those poses) = {num / den:.3f}")


def flywheel_report(series):
    print("\nFLYWHEEL")
    grid = resample(series, FLYWHEEL_KEYS)
    if not grid:
        print("  no flywheel data")
        return
    target = max(row["/Hardware/TalonFX/Flywheel/ClosedLoopReference"] for _, row in grid)
    if target <= 0:
        print("  never commanded to spin - nothing to measure")
        return

    # Up to speed: commanded at the top speed and no longer accelerating.
    at_speed = []
    for i, (t, row) in enumerate(grid):
        if abs(row["/Hardware/TalonFX/Flywheel/ClosedLoopReference"] - target) > 1e-6:
            continue
        if i == 0:
            continue
        change = abs(
            row["/Hardware/TalonFX/Flywheel/VelocityRps"]
            - grid[i - 1][1]["/Hardware/TalonFX/Flywheel/VelocityRps"]
        )
        if change < FLYWHEEL_SETTLE_RPS:
            at_speed.append((t, row))
    if not at_speed:
        print(f"  commanded {target:.1f} rps but never settled there")
        return

    rps = mean(at_speed, "/Hardware/TalonFX/Flywheel/VelocityRps")
    volts = mean(at_speed, "/Hardware/TalonFX/Flywheel/AppliedVolts")
    print(f"  commanded {target:.2f} rps, settled at {rps:.2f} rps on {volts:+.3f} V")
    print(f"  direction: {'ok' if volts * rps > 0 else 'BACKWARDS - flip MotorOutput.Inverted'}")
    error = rps - target
    if abs(error) > 0.02 * abs(target):
        print(f"  off by {error:+.2f} rps ({100 * error / target:+.1f}%) - feedforward is mistuned")
    print(f"  measured volts per rps = {volts / rps:.4f}   <- start kV here, then re-run")


def main():
    path = sys.argv[1] if len(sys.argv) > 1 else None
    if path is None:
        logs = glob.glob(os.path.join("logs", "*.wpilog"))
        if not logs:
            sys.exit("No logs found. Record one with simulateJavaAgent -Pmode=utility:Bring-Up")
        path = max(logs, key=os.path.getmtime)
    print(f"log: {path}\n")
    series = read(path, ARM_KEYS + FLYWHEEL_KEYS)
    arm_report(series)
    flywheel_report(series)


if __name__ == "__main__":
    main()
