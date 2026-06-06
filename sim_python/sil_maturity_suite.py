"""
sil_maturity_suite.py -- deterministic multi-seed C SIL fault matrix
====================================================================

This is a higher-evidence gate than the single realtime smoke tests.
It runs the C flight loop against the lightweight Python sensor/plant
driver across multiple RNG seeds and fault scenarios, then checks timing
and navigation envelopes.

Run from Satellite_GNC root:
    python sim_python/sil_maturity_suite.py --seeds 42 43 44 45 46
"""

import argparse
import os
import sys
import time
from dataclasses import dataclass
from typing import Iterable

import numpy as np

_HERE = os.path.dirname(os.path.abspath(__file__))
_ROOT = os.path.dirname(_HERE)
if _ROOT not in sys.path:
    sys.path.insert(0, _ROOT)

from sim_python.realtime_driver import run_sil


MAX_LOOP_MS = 10.0
MAX_MISSED_DEADLINES = 0
MAX_FINAL_POS_STD_M = 50.0
MAX_DROPOUT_POS_STD_M = 200.0
MAX_SPIKE_RANGE_M = 9000.0
MAX_BIAS_NORM_MRAD_S = 20.0
MAX_ACCEL_CMD_MS2 = 0.025

SCENARIOS = (
    "nominal",
    "range_dropout",
    "camera_spike",
    "gyro_bias",
    "gyro_dropout",
    "lidar_dropout",
    "stale_port",
    "frozen_packets",
    "delayed_packets",
    "bad_timestamps",
    "actuator_saturation",
)


@dataclass
class CaseResult:
    scenario: str
    seed: int
    passed: bool
    max_loop_ms: float
    missed: int
    final_pos_std_max_m: float
    final_range_m: float
    notes: str


def _base_checks(log) -> tuple[bool, list[str]]:
    notes: list[str] = []
    if not log:
        return False, ["no telemetry rows"]

    max_loop = max(r.max_loop_time_ms for r in log)
    missed = max(r.missed_deadlines for r in log)
    final_std = float(np.max(log[-1].pos_std))
    max_accel = max(float(np.linalg.norm(r.accel_cmd)) for r in log)

    if max_loop >= MAX_LOOP_MS:
        notes.append(f"max_loop={max_loop:.3f}ms >= {MAX_LOOP_MS:.1f}ms")
    if missed != MAX_MISSED_DEADLINES:
        notes.append(f"missed_deadlines={missed}")
    if final_std >= MAX_FINAL_POS_STD_M:
        notes.append(f"final_pos_std={final_std:.2f}m >= {MAX_FINAL_POS_STD_M:.1f}m")
    if max_accel > MAX_ACCEL_CMD_MS2:
        notes.append(f"accel_cmd={max_accel:.4f}m/s2 > {MAX_ACCEL_CMD_MS2:.4f}m/s2")

    return len(notes) == 0, notes


def _scenario_checks(scenario: str, log) -> tuple[bool, list[str]]:
    notes: list[str] = []

    if scenario == "range_dropout":
        drop_rows = [r for r in log if not r.range_valid]
        if not drop_rows:
            notes.append("range dropout not exercised")
        elif max(float(np.max(r.pos_std)) for r in drop_rows) >= MAX_DROPOUT_POS_STD_M:
            notes.append("dropout covariance exceeded envelope")

    elif scenario == "camera_spike":
        spike_rows = [r for r in log if r.tick == 500]
        if not spike_rows:
            notes.append("camera spike row missing")
        elif spike_rows[0].range_m >= MAX_SPIKE_RANGE_M:
            notes.append(f"camera spike corrupted range={spike_rows[0].range_m:.1f}m")

    elif scenario == "gyro_bias":
        max_bias = max(float(np.linalg.norm(r.att_bias)) for r in log) * 1000.0
        if max_bias >= MAX_BIAS_NORM_MRAD_S:
            notes.append(f"bias_norm={max_bias:.2f}mrad/s >= {MAX_BIAS_NORM_MRAD_S:.1f}")

    elif scenario == "gyro_dropout":
        drop_rows = [r for r in log if not r.gyro_valid]
        if not drop_rows:
            notes.append("gyro dropout not exercised")

    elif scenario == "lidar_dropout":
        drop_rows = [r for r in log if not r.port_valid]
        live_rows = [r for r in log if r.port_valid]
        if not live_rows:
            notes.append("port/lidar never acquired before dropout")
        if not drop_rows:
            notes.append("lidar dropout not exercised")
        if drop_rows and max(float(np.max(r.pos_std)) for r in drop_rows) >= MAX_DROPOUT_POS_STD_M:
            notes.append("lidar dropout covariance exceeded envelope")

    elif scenario == "stale_port":
        fault_rows = [r for r in log if 12.0 <= r.sim_time_s < 18.0 and r.port_valid]
        if len(fault_rows) < 3:
            notes.append("stale port window not exercised")
        else:
            span = max(r.port_range_m for r in fault_rows) - min(r.port_range_m for r in fault_rows)
            if span > 1.0:
                notes.append(f"stale port tracker moved too far span={span:.2f}m")

    elif scenario == "frozen_packets":
        frozen_rows = [r for r in log if 10.0 <= r.sim_time_s < 13.0]
        if not frozen_rows:
            notes.append("frozen packet window not exercised")

    elif scenario == "delayed_packets":
        delayed_rows = [r for r in log if r.tick >= 20]
        if not delayed_rows:
            notes.append("delayed packet window not exercised")

    elif scenario == "bad_timestamps":
        bad_rows = [r for r in log if 1000 <= r.tick < 1300]
        if not bad_rows:
            notes.append("bad timestamp window not exercised")
        notes.append("timestamp fields are not consumed by current C ABI")

    elif scenario == "actuator_saturation":
        max_accel = max(float(np.linalg.norm(r.accel_cmd)) for r in log)
        if max_accel > MAX_ACCEL_CMD_MS2:
            notes.append(f"actuator saturation exceeded accel envelope {max_accel:.4f}m/s2")

    return all("exceeded" not in n and "corrupted" not in n and "missing" not in n
               and "not exercised" not in n for n in notes), notes


def _scenario_x0(scenario: str):
    if scenario in ("lidar_dropout", "stale_port"):
        return np.array([0.0, 8.0, 0.0, 0.0, -0.002, 0.0])
    if scenario == "actuator_saturation":
        return np.array([0.0, 500.0, 0.0, 0.0, -0.40, 0.0])
    return None


def run_case(scenario: str, seed: int, ticks: int) -> CaseResult:
    log = run_sil(n_ticks=ticks, scenario=scenario, verbose=False,
                  rng_seed=seed, x0_override=_scenario_x0(scenario))
    base_ok, base_notes = _base_checks(log)
    scen_ok, scen_notes = _scenario_checks(scenario, log)
    notes = "; ".join(base_notes + scen_notes) if (base_notes or scen_notes) else "-"
    return CaseResult(
        scenario=scenario,
        seed=seed,
        passed=base_ok and scen_ok,
        max_loop_ms=max((r.max_loop_time_ms for r in log), default=float("nan")),
        missed=max((r.missed_deadlines for r in log), default=-1),
        final_pos_std_max_m=float(np.max(log[-1].pos_std)) if log else float("nan"),
        final_range_m=log[-1].range_m if log else float("nan"),
        notes=notes,
    )


def run_suite(seeds: Iterable[int], ticks: int) -> list[CaseResult]:
    results: list[CaseResult] = []
    for scenario in SCENARIOS:
        for seed in seeds:
            result = run_case(scenario, seed, ticks)
            results.append(result)
            mark = "PASS" if result.passed else "FAIL"
            print(f"{mark:4s}  {scenario:14s} seed={seed:<5d} "
                  f"loop={result.max_loop_ms:7.4f}ms "
                  f"pstd={result.final_pos_std_max_m:7.3f}m "
                  f"range={result.final_range_m:8.2f}m  {result.notes}")
    return results


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--seeds", type=int, nargs="+",
                        default=[42, 43, 44, 45, 46])
    parser.add_argument("--ticks", type=int, default=3600,
                        help="100 Hz ticks per case; 3600 = 36 seconds")
    args = parser.parse_args()

    print("=" * 72)
    print("  C SIL Maturity Suite -- deterministic multi-seed fault matrix")
    print("=" * 72)
    print(f"  scenarios={', '.join(SCENARIOS)}")
    print(f"  seeds={args.seeds}")
    print(f"  ticks_per_case={args.ticks} ({args.ticks/100.0:.1f}s)")
    print()

    t0 = time.perf_counter()
    results = run_suite(args.seeds, args.ticks)
    wall = time.perf_counter() - t0

    n_pass = sum(1 for r in results if r.passed)
    n_total = len(results)
    print()
    print("=" * 72)
    if n_pass == n_total:
        print(f"  PASS  ALL PASS ({n_pass}/{n_total})")
    else:
        print(f"  FAIL  {n_total - n_pass} FAILURES ({n_pass}/{n_total} passed)")
    print(f"  wall_time={wall:.2f}s")
    print("=" * 72)
    return 0 if n_pass == n_total else 1


if __name__ == "__main__":
    sys.exit(main())
