/**
 * rpod_ctrl.h — GEO RPOD Controller (PROX_OPS + TERMINAL)
 * =========================================================
 * Port of lambert_controller.py — PROX_OPS and TERMINAL phases only.
 * Lambert planning stays in Python (too complex for initial SIL port).
 *
 * Updated to match current Python lambert_controller.py:
 *
 *   PROX_OPS:
 *     - sqrt closing-speed law: v_close = K_SQRT * sqrt(range)
 *       replaces the old step-profile table.
 *     - Cap at V_CLOSE_MAX_MS (200mm/s) and V_CLOSE_NEAR_MS (5mm/s < 10m).
 *
 *   TERMINAL:
 *     - Port targeting: drives to port_lvlh, not just CoM.
 *     - EKF spike guard: if port_lvlh > PORT_SANITY_M from deputy, fall
 *       back to CoM (origin).
 *     - TAU gain scheduling: 2s (far), 3s (mid), 5s (close).
 *     - Entry brake: decelerates to BRAKE_DONE_MS if entering fast.
 *     - Speed law: sqrt on com_range, capped at V_TERM_MAX_MS.
 *     - Capture: creep at V_CAPTURE_MS until docked.
 *
 *   LOST_TARGET:
 *     - Decelerate to zero when camera is lost during close approach.
 *
 *   FORMATION_HOLD:
 *     - PD to standoff point.
 *
 * Compile:
 *   gcc -DMEKF_NO_CMSIS -shared -fPIC -O2 -Isrc_c \
 *       -o gnc_lib.dll src_c/rpod_ctrl.c -lm
 */

#ifndef RPOD_CTRL_H
#define RPOD_CTRL_H

/* ── PROX_OPS constants ─────────────────────────────────────────
 * sqrt closing-speed law:  v_close = K_SQRT * sqrt(range)
 *   K_SQRT = V_CLOSE_MAX_MS / sqrt(FAR_FIELD_M) = 0.200/sqrt(500)
 * caps:
 *   V_CLOSE_MAX_MS  = 200mm/s  (global cap)
 *   V_CLOSE_NEAR_MS =   5mm/s  (< 10m)
 */
#define RPOD_FAR_FIELD_M        500.0   /* Lambert → PROX_OPS handoff [m]  */
#define RPOD_TERMINAL_M           0.8   /* PROX_OPS → TERMINAL handoff [m] */
#define RPOD_PROX_TAU             5.0   /* velocity-error time constant [s] */

#define RPOD_V_CLOSE_MAX_MS    0.200    /* max closing speed [m/s]          */
#define RPOD_V_CLOSE_NEAR_MS   0.005    /* closing speed cap below 10m [m/s]*/
#define RPOD_V_CLOSE_NEAR_RNG  10.0    /* range below which near-cap applies [m] */
/* K_SQRT = 0.200 / sqrt(500) ≈ 0.008944  [m/s / sqrt(m)] */
#define RPOD_K_SQRT             0.008944272

/* ── TERMINAL constants ──────────────────────────────────────── */
#define RPOD_V_TERM_MAX_MS     0.025    /* max speed in TERMINAL [m/s]      */
#define RPOD_V_CAPTURE_MS      0.0015   /* speed inside capture zone [m/s]  */
#define RPOD_DOCK_RANGE_M      0.30     /* capture sphere radius [m]        */
#define RPOD_DOCK_DONE_M       0.20     /* docking complete [m]             */
#define RPOD_DOCK_MAX_SPEED_MS 0.010    /* max port-relative dock speed     */
#define RPOD_PORT_SANITY_M     2.0      /* EKF spike guard: max port dist   */
#define RPOD_BRAKE_DONE_MS     0.010    /* entry brake target speed [m/s]   */
#define RPOD_BRAKE_ENTRY_MS    0.015    /* brake if entry speed > this [m/s]*/

/* TAU gain scheduling thresholds */
#define RPOD_TAU_CLOSE         8.0      /* TAU [s] for com_range < 0.30m   */
#define RPOD_TAU_MID           5.0      /* TAU [s] for 0.30m-0.60m         */
#define RPOD_TAU_FAR           3.0      /* TAU [s] for com_range >= 0.60m  */

/* Terminal sqrt-law gain: V_TERM_MAX_MS / sqrt(TERMINAL_M) */
#define RPOD_K_SQRT_TERM       0.027951 /* = 0.025 / sqrt(0.8) */

/* ── TERMINAL sqrt-law gain (for test_rpod.c compatibility) ─── */
/* K_SQRT_TERM defined above as RPOD_K_SQRT_TERM.
 * test_rpod.c uses RPOD_TERMINAL_K and RPOD_TERMINAL_VMAX aliases. */
#define RPOD_TERMINAL_K     RPOD_K_SQRT_TERM
#define RPOD_TERMINAL_VMAX  RPOD_V_TERM_MAX_MS

/* ── FORMATION HOLD constants ───────────────────────────────── */
#define RPOD_STANDOFF_M        1000.0   /* formation hold standoff [m]     */

/* ── Input/output structs ──────────────────────────────────────
 * RPOD_State: LVLH relative state [m, m/s] — same as before.
 * RPOD_TermState: extended input for TERMINAL phase (port + axis).
 */
typedef struct {
    double pos[3];   /* LVLH relative position [m]   */
    double vel[3];   /* LVLH relative velocity [m/s] */
} RPOD_State;

/**
 * RPOD_TermState — extended TERMINAL input with port information.
 *
 * port_lvlh[3]:      docking port position in LVLH [m].
 * port_axis_lvlh[3]: outward docking axis in LVLH, reserved for alignment gates.
 * port_vel_lvlh[3]:  docking port velocity in LVLH [m/s].
 * has_port:          1 if port fields are valid, 0 to use CoM.
 */
typedef struct {
    double pos[3];
    double vel[3];
    double port_lvlh[3];
    double port_axis_lvlh[3];
    double port_vel_lvlh[3];
    int    has_port;
} RPOD_TermState;

/**
 * RPOD_FormHoldState — input for formation hold.
 * n_chief: chief mean motion [rad/s] (used for PD gain derivation).
 */
typedef struct {
    double pos[3];
    double vel[3];
    double n_chief;
    double accel_max;
} RPOD_FormHoldState;

/* ── API ──────────────────────────────────────────────────────── */

/**
 * RPOD_prox_ops — sqrt-law PD closure from FAR_FIELD_M to TERMINAL_M.
 *
 * Mirrors Python _prox_ops().
 *
 * state       : LVLH relative state (from EKF)
 * truth_range : fresh range from caller [m] (from true_cw[:3] norm)
 * accel_max   : max accel [m/s²]
 * accel_out   : commanded acceleration [m/s²]
 *
 * Returns 1 if transitioning to TERMINAL (truth_range < RPOD_TERMINAL_M),
 *         0 for normal PROX_OPS, -1 for deadband (range < dock gate).
 *
 * NOTE: Lambert planning and mode transitions are handled in Python.
 *       This function is the inner guidance loop only.
 */
int RPOD_prox_ops(const RPOD_State *state,
                  double truth_range,
                  double n_chief,      /* chief mean motion [rad/s] — reserved */
                  double accel_max,
                  double accel_out[3]);

/**
 * RPOD_terminal — port-targeting deceleration to dock.
 *
 * Mirrors Python _terminal().
 *
 * state     : extended state with port_lvlh (use RPOD_TermState).
 * accel_max : max accel [m/s²]
 * accel_out : commanded acceleration [m/s²]
 * is_braking: pass pointer to int; caller must preserve across calls for
 *             the entry-brake state machine. Initialize to -1 (unset).
 *             RPOD_terminal sets *is_braking = 1 on entry if fast,
 *             = 0 when brake is done or not needed.
 *
 * Returns:
 *    2 = DOCKED (range < RPOD_DOCK_DONE_M, zero accel)
 *    1 = in capture zone (port_range < RPOD_DOCK_RANGE_M)
 *    0 = normal approach
 *   -1 = entry brake in progress
 */
int RPOD_terminal(const RPOD_TermState *state,
                  double accel_max,
                  double accel_out[3],
                  int *is_braking);

/**
 * RPOD_terminal_simple — 3-argument version for test_rpod.c compatibility.
 * Uses an internal static is_braking flag (reset on each call with state).
 * For production use call RPOD_terminal() with explicit is_braking pointer.
 */
int RPOD_terminal_simple(const RPOD_State *state,
                          double accel_max,
                          double accel_out[3]);

/**
 * RPOD_lost_target — hold position when camera is lost.
 *
 * Mirrors Python _lost_target().
 * Decelerates to zero velocity. Caller re-enters PROX_OPS when camera
 * recovers (handled in Python mode manager).
 *
 * state    : current LVLH relative state
 * accel_max: max accel [m/s²]
 * accel_out: commanded acceleration [m/s²]
 */
void RPOD_lost_target(const RPOD_State *state,
                      double accel_max,
                      double accel_out[3]);

/**
 * RPOD_formation_hold — PD hold at standoff point.
 *
 * Mirrors Python _formation_hold().
 * Standoff target: [0, -RPOD_STANDOFF_M, 0] in LVLH.
 *
 * state    : form-hold state with n_chief and accel_max baked in
 * accel_out: commanded acceleration [m/s²]
 */
void RPOD_formation_hold(const RPOD_FormHoldState *state,
                         double accel_out[3]);

#endif /* RPOD_CTRL_H */
