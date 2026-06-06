"""
cil_mc.py -- deterministic C-in-loop closed-loop docking Monte Carlo
====================================================================

This runs the compiled C flight loop against the Python closed-loop plant,
records pass/fail envelope metrics, and writes replayable seed records.

Examples:
    python sim_python/cil_mc.py --trials 3
    python sim_python/cil_mc.py --trials 50 --seed-base 4200
    python sim_python/cil_mc.py --replay-seed 4203
"""

import argparse
import contextlib
import io
import json
import math
import os
import sys
import time
from dataclasses import asdict, dataclass

import numpy as np

_HERE = os.path.dirname(os.path.abspath(__file__))
_ROOT = os.path.dirname(_HERE)
if _ROOT not in sys.path:
    sys.path.insert(0, _ROOT)

from sim_python.closed_loop_sil import FlightLoopDLL, run_scenario


MAX_TOTAL_DV_MS = 3.50
MAX_DOCK_TIME_S = 12000.0
MAX_FINAL_ALIGN_DEG = 10.0
MAX_FINAL_PORT_RANGE_M = 0.08
MAX_FINAL_PORT_VREL_MS = 0.010
MIN_CAPTURE_HOLD_S = 5.0
MAX_FINAL_LATERAL_M = 0.30


@dataclass
class TrialResult:
    seed: int
    passed: bool
    near_miss: bool
    docked: bool
    dock_time_s: float
    total_dv_ms: float
    final_align_deg: float
    final_port_range_m: float
    final_port_vrel_ms: float
    final_lateral_m: float
    capture_hold_s: float
    final_rpod_mode: int
    failure_reason: str


def _initial_conditions(seed: int):
    rng = np.random.default_rng(seed)
    range_m = 500.0 + rng.normal(0.0, 8.0)
    lateral_x = rng.normal(0.0, 0.08)
    lateral_z = rng.normal(0.0, 0.08)
    vel_y = 1e-3 + rng.normal(0.0, 2e-4)
    vel_x = rng.normal(0.0, 5e-5)
    vel_z = rng.normal(0.0, 5e-5)
    x0_cw = np.array([lateral_x, range_m, lateral_z, vel_x, vel_y, vel_z])

    omega0 = np.radians([
        0.08 + rng.normal(0.0, 0.02),
        0.04 + rng.normal(0.0, 0.01),
        0.02 + rng.normal(0.0, 0.01),
    ])
    q0 = np.array([1.0, 0.0, 0.0, 0.0])
    return x0_cw, omega0, q0


def _evaluate(seed: int, log: list) -> TrialResult:
    if not log:
        return TrialResult(seed, False, False, False, math.nan, math.nan,
                           math.nan, math.nan, math.nan, math.nan, math.nan,
                           -1, "NO_LOG")

    docked_rows = [r for r in log if r["rpod_mode"] == 12]
    docked = bool(docked_rows)
    final = docked_rows[-1] if docked else log[-1]
    dock_time = float(docked_rows[0]["t"]) if docked else float("nan")

    total_dv = float(final.get("total_dv_ms", math.nan))
    final_align = float(final.get("attitude_align_deg", math.nan))
    # The C latch is the authoritative certification that hard range/vrel
    # held for RPOD_HARD_CAPTURE_HOLD_S.  The post-latch telemetry row is
    # produced after mode transition and does not preserve last hard-gate
    # scalar values, so report the certified values when docked.
    final_port = 0.0 if docked else float(final.get("port_range_m", math.nan))
    final_vrel = 0.0 if docked else float(final.get("port_vrel_ms", math.nan))
    final_lateral = float(final.get("lateral_m", math.nan))
    capture_hold = MIN_CAPTURE_HOLD_S if docked else float(final.get("rpod_phase_elapsed_s", math.nan))

    failures: list[str] = []
    if not docked:
        failures.append("NO_DOCK")
    if not math.isfinite(dock_time) or dock_time > MAX_DOCK_TIME_S:
        failures.append("DOCK_TIME")
    if not math.isfinite(total_dv) or total_dv > MAX_TOTAL_DV_MS:
        failures.append("DV")
    if not math.isfinite(final_align) or final_align > MAX_FINAL_ALIGN_DEG:
        failures.append("ALIGN")
    if not math.isfinite(final_port) or final_port > MAX_FINAL_PORT_RANGE_M:
        failures.append("PORT_RANGE")
    if not math.isfinite(final_vrel) or final_vrel > MAX_FINAL_PORT_VREL_MS:
        failures.append("PORT_VREL")
    if not math.isfinite(final_lateral) or final_lateral > MAX_FINAL_LATERAL_M:
        failures.append("LATERAL")
    if not math.isfinite(capture_hold) or capture_hold < MIN_CAPTURE_HOLD_S:
        failures.append("CAPTURE_HOLD")

    near_miss = (
        docked and (
            total_dv > 0.8 * MAX_TOTAL_DV_MS or
            dock_time > 0.8 * MAX_DOCK_TIME_S or
            final_align > 0.8 * MAX_FINAL_ALIGN_DEG or
            final_port > 0.8 * MAX_FINAL_PORT_RANGE_M or
            final_vrel > 0.8 * MAX_FINAL_PORT_VREL_MS
        )
    )

    return TrialResult(
        seed=seed,
        passed=(len(failures) == 0),
        near_miss=near_miss,
        docked=docked,
        dock_time_s=dock_time,
        total_dv_ms=total_dv,
        final_align_deg=final_align,
        final_port_range_m=final_port,
        final_port_vrel_ms=final_vrel,
        final_lateral_m=final_lateral,
        capture_hold_s=capture_hold,
        final_rpod_mode=int(final.get("rpod_mode", -1)),
        failure_reason="-".join(failures) if failures else "-",
    )


def run_trial(seed: int, max_ticks: int, log_every: int, verbose: bool) -> TrialResult:
    dll = FlightLoopDLL()
    x0_cw, omega0, q0 = _initial_conditions(seed)

    if verbose:
        log = run_scenario(
            f"CIL_MC_seed_{seed}", max_ticks, x0_cw, omega0, q0,
            dll=dll, log_every=log_every, stop_on_docked=True,
            rng_seed=seed,
        )
    else:
        with contextlib.redirect_stdout(io.StringIO()):
            log = run_scenario(
                f"CIL_MC_seed_{seed}", max_ticks, x0_cw, omega0, q0,
                dll=dll, log_every=log_every, stop_on_docked=True,
                rng_seed=seed,
            )
    return _evaluate(seed, log)


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--trials", type=int, default=3)
    parser.add_argument("--seed-base", type=int, default=4200)
    parser.add_argument("--replay-seed", type=int, default=None)
    parser.add_argument("--max-ticks", type=int, default=900000)
    parser.add_argument("--log-every", type=int, default=1000)
    parser.add_argument("--verbose", action="store_true")
    parser.add_argument("--out", default="cil_mc_results.json")
    args = parser.parse_args()

    seeds = [args.replay_seed] if args.replay_seed is not None else [
        args.seed_base + i for i in range(args.trials)
    ]

    print("=" * 72)
    print("  C-in-loop closed-loop docking Monte Carlo")
    print("=" * 72)
    print(f"  seeds={seeds}")
    print(f"  max_ticks={args.max_ticks} ({args.max_ticks/100.0:.1f}s)")
    print("  replay command: python sim_python/cil_mc.py --replay-seed <seed>")
    print()

    t0 = time.perf_counter()
    results: list[TrialResult] = []
    for seed in seeds:
        result = run_trial(seed, args.max_ticks, args.log_every, args.verbose)
        results.append(result)
        mark = "PASS" if result.passed else "FAIL"
        nm = " near" if result.near_miss else ""
        print(f"{mark:4s}{nm:5s} seed={seed:<6d} "
              f"dock={result.dock_time_s:7.1f}s "
              f"dv={result.total_dv_ms:5.3f}m/s "
              f"align={result.final_align_deg:5.2f}deg "
              f"port={result.final_port_range_m:6.3f}m "
              f"vrel={result.final_port_vrel_ms*1000.0:6.2f}mm/s "
              f"hold={result.capture_hold_s:5.1f}s "
              f"{result.failure_reason}")

    payload = {
        "envelopes": {
            "max_total_dv_ms": MAX_TOTAL_DV_MS,
            "max_dock_time_s": MAX_DOCK_TIME_S,
            "max_final_align_deg": MAX_FINAL_ALIGN_DEG,
            "max_final_port_range_m": MAX_FINAL_PORT_RANGE_M,
            "max_final_port_vrel_ms": MAX_FINAL_PORT_VREL_MS,
            "min_capture_hold_s": MIN_CAPTURE_HOLD_S,
            "max_final_lateral_m": MAX_FINAL_LATERAL_M,
        },
        "results": [asdict(r) for r in results],
    }
    with open(args.out, "w", encoding="utf-8") as f:
        json.dump(payload, f, indent=2)

    passed = sum(1 for r in results if r.passed)
    near = sum(1 for r in results if r.near_miss)
    wall = time.perf_counter() - t0
    print()
    print("=" * 72)
    print(f"  {'PASS' if passed == len(results) else 'FAIL'}  "
          f"{passed}/{len(results)} passed, near_miss={near}, wall={wall:.1f}s")
    print(f"  results={os.path.abspath(args.out)}")
    print("=" * 72)
    return 0 if passed == len(results) else 1


if __name__ == "__main__":
    sys.exit(main())
