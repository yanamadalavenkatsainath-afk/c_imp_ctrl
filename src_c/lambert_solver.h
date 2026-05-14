/**
 * lambert_solver.h — Lambert's Problem Solver (Universal Variable)
 * =================================================================
 * Port of lambert_solver.py — Curtis Algorithm 5.2.
 *
 * Given two ECI position vectors r1, r2 and a time of flight tof,
 * finds the velocity vectors v1, v2 of the connecting conic arc.
 *
 * Also provides:
 *   - RK4 Keplerian propagator  (propagate_keplerian)
 *   - Min-ΔV TOF scanner        (lambert_min_dv)
 *
 * Key usage for GEO rendezvous:
 *   r1       = deputy ECI position NOW
 *   r2       = chief ECI position at t + TOF  (propagate chief first!)
 *   v1_cur   = deputy ECI velocity NOW
 *   v2_target= chief ECI velocity at t + TOF
 *   dv1      = v1_lambert - v1_cur   (departure burn)
 *   dv2      = v2_target - v2_lambert (arrival burn)
 *
 * Reference:
 *   Curtis, H. (2014). "Orbital Mechanics for Engineering Students."
 *   Algorithm 5.2, §5.3. Elsevier.
 *   Bate, Mueller & White (1971). §5.3.
 *
 * No malloc. All temporaries stack-allocated.
 */

#ifndef LAMBERT_SOLVER_H
#define LAMBERT_SOLVER_H

#include <math.h>

#define LAMBERT_MU_DEFAULT  3.986004418e14   /* m³/s² */

/* ── Result struct ────────────────────────────────────────────── */

typedef struct {
    double v1[3];   /* departure velocity [m/s] at r1 */
    double v2[3];   /* arrival  velocity  [m/s] at r2 */
    int    ok;      /* 1 = solution found, 0 = degenerate */
} Lambert_Result;

typedef struct {
    double best_tof;    /* optimal TOF [s],  NaN if no solution */
    double dv1[3];      /* departure ΔV [m/s] */
    double dv2[3];      /* arrival  ΔV [m/s]  */
    double total_dv;    /* |dv1| + |dv2| [m/s], HUGE_VAL if none */
    int    ok;          /* 1 = solution found */
} Lambert_MinDV;

/* ── API ──────────────────────────────────────────────────────── */

/**
 * LAMBERT_solve — solve Lambert's problem (single TOF).
 *
 * Mirrors Python LambertSolver.solve().
 *
 * r1[3]    : ECI departure position [m]
 * r2[3]    : ECI arrival  position  [m]
 * tof      : time of flight [s]
 * prograde : 1 = prograde arc, 0 = retrograde
 * mu       : gravitational parameter [m³/s²]
 *
 * Returns Lambert_Result with .ok=1 on success, .ok=0 on failure.
 */
Lambert_Result LAMBERT_solve(const double r1[3],
                              const double r2[3],
                              double tof,
                              int prograde,
                              double mu);

/**
 * LAMBERT_propagate_keplerian — RK4 two-body propagator.
 *
 * Mirrors Python LambertSolver.propagate_keplerian().
 *
 * pos_in[3]  : ECI position [m]
 * vel_in[3]  : ECI velocity [m/s]
 * dt         : propagation time [s]
 * n_steps    : RK4 substeps (200 is accurate for GEO)
 * mu         : gravitational parameter [m³/s²]
 * pos_out[3] : propagated ECI position [m]
 * vel_out[3] : propagated ECI velocity [m/s]
 */
void LAMBERT_propagate_keplerian(const double pos_in[3],
                                  const double vel_in[3],
                                  double dt,
                                  int n_steps,
                                  double mu,
                                  double pos_out[3],
                                  double vel_out[3]);

/**
 * LAMBERT_min_dv — find minimum ΔV transfer by scanning TOF.
 *
 * Mirrors Python LambertSolver.min_dv_transfer().
 *
 * IMPORTANT: r2_chief and v2_chief must be the chief's ECI state
 * at the ARRIVAL TIME (t + tof), not at the current time.
 * Call LAMBERT_propagate_keplerian() to advance the chief first.
 *
 * r1_dep[3]   : deputy ECI position NOW [m]
 * v1_dep[3]   : deputy ECI velocity NOW [m/s]
 * r2_chief[3] : chief ECI position at ARRIVAL [m]
 * v2_chief[3] : chief ECI velocity at ARRIVAL [m/s]
 * tof_min     : minimum TOF to scan [s]
 * tof_max     : maximum TOF to scan [s]
 * n_scan      : number of TOF samples (default 80 in Python)
 * dv_cap      : per-burn ΔV cap [m/s] (default 0.5 in Python)
 * mu          : gravitational parameter [m³/s²]
 *
 * Returns Lambert_MinDV with .ok=1 if a valid solution was found.
 */
Lambert_MinDV LAMBERT_min_dv(const double r1_dep[3],
                               const double v1_dep[3],
                               const double r2_chief[3],
                               const double v2_chief[3],
                               double tof_min,
                               double tof_max,
                               int n_scan,
                               double dv_cap,
                               double mu);

#endif /* LAMBERT_SOLVER_H */