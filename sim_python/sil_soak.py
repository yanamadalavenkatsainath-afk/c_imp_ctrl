"""
sil_soak.py -- long-duration lightweight C SIL soak test
=======================================================

Runs the C flight loop through the realtime driver for many ticks and
checks timing, covariance boundedness, finite telemetry, and command limits.
This is intentionally faster than closed-loop docking MC, so it can live in
the normal build gate.
"""

import argparse
import math
import os
import sys
import time

import numpy as np

_HERE = os.path.dirname(os.path.abspath(__file__))
_ROOT = os.path.dirname(_HERE)
if _ROOT not in sys.path:
    sys.path.insert(0, _ROOT)

from sim_python.realtime_driver import run_sil


MAX_LOOP_MS = 10.0
MAX_MISSED = 0
MAX_POS_STD_M = 100.0
MAX_ACCEL_MS2 = 0.025


def _finite_row(row) -> bool:
    arrays = [row.pos_lvlh, row.vel_lvlh, row.pos_std, row.accel_cmd, row.att_bias]
    return all(np.all(np.isfinite(a)) for a in arrays)


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--ticks", type=int, default=100000,
                        help="100 Hz ticks; 100000 = 1000 seconds")
    parser.add_argument("--seed", type=int, default=4242)
    parser.add_argument("--scenario", default="delayed_packets",
                        choices=["nominal", "range_dropout", "camera_spike",
                                 "gyro_bias", "gyro_dropout", "lidar_dropout",
                                 "stale_port", "frozen_packets",
                                 "delayed_packets", "bad_timestamps",
                                 "actuator_saturation"])
    args = parser.parse_args()

    print("=" * 72)
    print("  C SIL long-duration soak")
    print("=" * 72)
    print(f"  scenario={args.scenario} seed={args.seed} "
          f"ticks={args.ticks} ({args.ticks/100.0:.1f}s)")

    t0 = time.perf_counter()
    log = run_sil(n_ticks=args.ticks, scenario=args.scenario,
                  verbose=False, rng_seed=args.seed)
    wall = time.perf_counter() - t0

    notes: list[str] = []
    if not log:
        notes.append("no telemetry")
    else:
        max_loop = max(r.max_loop_time_ms for r in log)
        missed = max(r.missed_deadlines for r in log)
        max_std = max(float(np.max(r.pos_std)) for r in log)
        max_accel = max(float(np.linalg.norm(r.accel_cmd)) for r in log)
        any_bad = any(not _finite_row(r) for r in log)

        if max_loop >= MAX_LOOP_MS:
            notes.append(f"max_loop={max_loop:.3f}ms >= {MAX_LOOP_MS:.1f}ms")
        if missed != MAX_MISSED:
            notes.append(f"missed_deadlines={missed}")
        if max_std >= MAX_POS_STD_M:
            notes.append(f"pos_std={max_std:.2f}m >= {MAX_POS_STD_M:.1f}m")
        if max_accel > MAX_ACCEL_MS2:
            notes.append(f"accel={max_accel:.4f}m/s2 > {MAX_ACCEL_MS2:.4f}m/s2")
        if any_bad:
            notes.append("non-finite telemetry")

        print(f"  rows={len(log)} wall={wall:.2f}s "
              f"max_loop={max_loop:.4f}ms missed={missed} "
              f"max_pos_std={max_std:.3f}m max_accel={max_accel:.5f}m/s2")
        print(f"  final_range={log[-1].range_m:.2f}m "
              f"final_mode={log[-1].rpod_mode} "
              f"pose_age={log[-1].pose_age_s:.1f}s")

    print("=" * 72)
    if notes:
        print("  FAIL  " + "; ".join(notes))
        return 1
    print("  PASS  soak envelopes clear")
    return 0


if __name__ == "__main__":
    sys.exit(main())
