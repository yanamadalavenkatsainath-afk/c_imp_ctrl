/**
 * mode_manager.h — FSW Mode Management State Machine
 * ====================================================
 * Direct port of mode_manager.py — ModeManager class.
 *
 * Manages transitions between:
 *   SAFE_MODE       — fault state, all actuators off
 *   DETUMBLE        — B-dot rate damping
 *   SUN_ACQUISITION — slew for QUEST geometry
 *   FINE_POINTING   — MEKF + PD control
 *   MOMENTUM_DUMP   — magnetorquer desaturation
 *
 * Transition logic mirrors Python ModeManager.update() exactly.
 *
 * Reference:
 *   Wertz, "Space Mission Engineering", §11.3
 *   Sidi, "Spacecraft Dynamics and Control", §9.4
 */

#ifndef MODE_MANAGER_H
#define MODE_MANAGER_H

#include <stdint.h>
#include <math.h>

/* ── FSW modes (matches Python Mode enum values) ─────────────── */
typedef enum {
    MODE_SAFE_MODE       = 0,
    MODE_DETUMBLE        = 1,
    MODE_SUN_ACQUISITION = 2,
    MODE_FINE_POINTING   = 3,
    MODE_MOMENTUM_DUMP   = 4,
} FSW_Mode;

/* ── Transition thresholds (mirrors Python ModeManager constants) */
#define MM_SAFE_RATE_RAD_S      (40.0 * 3.14159265358979 / 180.0)  /* 40 deg/s */
#define MM_DETUMBLE_THRESH_RAD_S (3.5 * 3.14159265358979 / 180.0)  /* 3.5 deg/s */
#define MM_TRIAD_ERR_DEG        15.0      /* deg — QUEST/TRIAD accepted */
#define MM_SUN_ACQ_TIMEOUT_S    600.0     /* s */
#define MM_DUMP_TRIGGER_NMS     3.000     /* N·m·s — 75% of 4.0 N·m·s wheel */
#define MM_DUMP_COMPLETE_NMS    0.800     /* N·m·s — 20% */
#define MM_DUMP_POINTING_GUARD_DEG 5.0   /* deg — prevent thrashing */

/* ── Fault flag bits ─────────────────────────────────────────── */
#define MM_FAULT_RATE_EXCEEDED  (1u << 0)
#define MM_FAULT_EXTERNAL       (1u << 1)

/* ── State struct ─────────────────────────────────────────────── */
typedef struct {
    FSW_Mode  mode;            /* current mode                     */
    FSW_Mode  prev_mode;       /* previous mode                    */
    double    mode_entry_t;    /* time of last transition [s]      */
    uint32_t  fault_flags;     /* bitmask of active faults         */

    double    triad_err_deg;   /* last known QUEST/TRIAD error     */
    int       triad_err_valid; /* 1 if triad_err_deg has been set  */

    double    pointing_err_deg; /* last known MEKF pointing error  */
    int       pointing_err_valid;

    /* Transition log (last 32 transitions) */
    struct {
        double    t;
        FSW_Mode  mode;
    } history[32];
    int history_count;
} MM_State;

/* ── API ──────────────────────────────────────────────────────── */

/**
 * MM_init — initialise state machine, starting in DETUMBLE.
 * Mirrors Python ModeManager.__init__().
 */
void MM_init(MM_State *s);

/**
 * MM_update — evaluate transitions and update mode.
 *
 * Mirrors Python ModeManager.update() exactly.
 *
 * s                 : state (modified in place)
 * t                 : current simulation time [s]
 * omega[3]          : angular rate vector [rad/s]
 * wheel_h[3]        : reaction wheel momentum [N·m·s]
 * quest_err_deg     : QUEST/TRIAD init error [deg]; <0 = not available
 * fault             : 1 = external fault asserted
 * pointing_err_deg  : MEKF pointing error [deg]; <0 = not available
 *
 * Returns current FSW_Mode.
 */
FSW_Mode MM_update(MM_State *s,
                   double t,
                   const double omega[3],
                   const double wheel_h[3],
                   double quest_err_deg,
                   int fault,
                   double pointing_err_deg);

/* ── Mode query helpers ───────────────────────────────────────── */

static inline int MM_is_detumbling(const MM_State *s)   { return s->mode == MODE_DETUMBLE; }
static inline int MM_is_sun_acq(const MM_State *s)      { return s->mode == MODE_SUN_ACQUISITION; }
static inline int MM_is_fine_pointing(const MM_State *s){ return s->mode == MODE_FINE_POINTING; }
static inline int MM_is_momentum_dump(const MM_State *s){ return s->mode == MODE_MOMENTUM_DUMP; }
static inline int MM_is_safe(const MM_State *s)         { return s->mode == MODE_SAFE_MODE; }

static inline double MM_time_in_mode(const MM_State *s, double t) {
    return t - s->mode_entry_t;
}

/** MM_mode_name — return human-readable string for a mode. */
const char *MM_mode_name(FSW_Mode mode);

#endif /* MODE_MANAGER_H */