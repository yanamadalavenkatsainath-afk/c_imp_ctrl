/**
 * rpod_ctrl.h — GEO RPOD Controller (PROX_OPS + TERMINAL)
 * =========================================================
 * Port of lambert_controller.py — PROX_OPS and TERMINAL phases only.
 * Lambert planning stays in Python (too complex for initial SIL port).
 *
 * These two functions are the safety-critical inner loop:
 *   RPOD_prox_ops()  — closes from 500m to 0.8m
 *   RPOD_terminal()  — closes from 0.8m to dock
 *
 * Both are pure functions (no state machine, no malloc).
 * The mode state machine stays in Python and calls these as needed.
 *
 * Compile:
 *   gcc -DMEKF_NO_CMSIS -shared -fPIC -O2 -o gnc_lib.dll src_c/rpod_ctrl.c -lm
 */

#ifndef RPOD_CTRL_H
#define RPOD_CTRL_H

/* ── Constants — match lambert_controller.py exactly ─────────── */
#define RPOD_FAR_FIELD_M   500.0   /* Lambert → PROX_OPS handoff [m]   */
#define RPOD_TERMINAL_M      0.8   /* PROX_OPS → TERMINAL handoff [m]  */
#define RPOD_PROX_TAU        5.0   /* velocity error time constant [s]  */
#define RPOD_TERMINAL_K      0.010 /* terminal speed law gain [1/s]     */
#define RPOD_TERMINAL_VMAX   0.005 /* terminal max speed [m/s]          */

/* PROX_OPS velocity profile — (range_m, v_close_ms)
   Matches PROX_V_PROFILE in lambert_controller.py */
#define RPOD_PROFILE_LEN  8
typedef struct { double range_m; double v_close_ms; } RPOD_ProfileEntry;

/* ── Input/output struct ──────────────────────────────────────── */

/**
 * RPOD_State — LVLH relative state [m, m/s]
 * pos[3]: relative position in LVLH
 * vel[3]: relative velocity in LVLH
 */
typedef struct {
    double pos[3];
    double vel[3];
} RPOD_State;

/* ── API ──────────────────────────────────────────────────────── */

/**
 * RPOD_prox_ops — PD closure from FAR_FIELD_M to TERMINAL_M.
 *
 * state       : current LVLH relative state
 * truth_range : fresh range from caller [m]
 * n_chief     : chief mean motion [rad/s]
 * accel_max   : max accel [m/s²]
 * accel_out   : commanded acceleration output [m/s²]
 *
 * Returns 1 if transitioning to TERMINAL (range < RPOD_TERMINAL_M),
 *         0 otherwise.
 */
int RPOD_prox_ops(const RPOD_State *state,
                  double truth_range,
                  double n_chief,
                  double accel_max,
                  double accel_out[3]);

/**
 * RPOD_terminal — range-proportional deceleration to dock.
 *
 * state      : current LVLH relative state
 * accel_max  : max accel [m/s²]
 * accel_out  : commanded acceleration output [m/s²]
 *
 * Returns 1 if docking condition met (range < 0.05m),
 *         0 otherwise.
 */
int RPOD_terminal(const RPOD_State *state,
                  double accel_max,
                  double accel_out[3]);

#endif /* RPOD_CTRL_H */