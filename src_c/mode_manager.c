/**
 * mode_manager.c — FSW Mode Management State Machine
 * ====================================================
 * Port of mode_manager.py — ModeManager class.
 * Transition logic is a 1-to-1 translation of Python ModeManager.update().
 */

#include "mode_manager.h"
#include "target_port.h"
#include <math.h>
#include <string.h>

/* ── Internal helper ──────────────────────────────────────────── */

static void _transition(MM_State *s, FSW_Mode new_mode, double t) {
    if (new_mode == s->mode) return;

    GNC_LOG("  FSW [%7.1fs] %-20s -> %s\n",
           t, MM_mode_name(s->mode), MM_mode_name(new_mode));

    s->prev_mode    = s->mode;
    s->mode         = new_mode;
    s->mode_entry_t = t;

    if (s->history_count < 32) {
        s->history[s->history_count].t    = t;
        s->history[s->history_count].mode = new_mode;
        s->history_count++;
    }
}

/* ── Public API ───────────────────────────────────────────────── */

void MM_init(MM_State *s) {
    memset(s, 0, sizeof(MM_State));
    s->mode              = MODE_DETUMBLE;
    s->prev_mode         = MODE_DETUMBLE;
    s->mode_entry_t      = 0.0;
    s->fault_flags       = 0;
    s->triad_err_valid   = 0;
    s->triad_err_deg     = -1.0;
    s->pointing_err_valid = 0;
    s->pointing_err_deg  = -1.0;
    s->history_count     = 0;
    GNC_LOG("  FSW ModeManager initialised — starting in DETUMBLE\n");
}

FSW_Mode MM_update(MM_State *s,
                   double t,
                   const double omega[3],
                   const double wheel_h[3],
                   double quest_err_deg,
                   int fault,
                   double pointing_err_deg) {

    double rate  = sqrt(omega[0]*omega[0] + omega[1]*omega[1] + omega[2]*omega[2]);
    double h_max = 0.0;
    for (int i=0;i<3;i++) {
        double ah = fabs(wheel_h[i]);
        if (ah > h_max) h_max = ah;
    }

    /* Update optional inputs */
    if (pointing_err_deg >= 0.0) {
        s->pointing_err_deg   = pointing_err_deg;
        s->pointing_err_valid = 1;
    }

    /* ── Fault / safe mode — highest priority ─────────────────── */
    if (fault || rate > MM_SAFE_RATE_RAD_S) {
        if (rate > MM_SAFE_RATE_RAD_S)
            s->fault_flags |= MM_FAULT_RATE_EXCEEDED;
        if (fault)
            s->fault_flags |= MM_FAULT_EXTERNAL;
        _transition(s, MODE_SAFE_MODE, t);
        return s->mode;
    }

    /* ── Recover from safe mode ────────────────────────────────── */
    if (s->mode == MODE_SAFE_MODE) {
        if (rate < MM_DETUMBLE_THRESH_RAD_S * 5.0 && !fault) {
            s->fault_flags = 0;
            _transition(s, MODE_DETUMBLE, t);
        }
        return s->mode;
    }

    /* ── DETUMBLE exit ─────────────────────────────────────────── */
    if (s->mode == MODE_DETUMBLE) {
        if (rate < MM_DETUMBLE_THRESH_RAD_S)
            _transition(s, MODE_SUN_ACQUISITION, t);
        return s->mode;
    }

    /* ── SUN_ACQUISITION exit ──────────────────────────────────── */
    if (s->mode == MODE_SUN_ACQUISITION) {
        double time_in_mode = t - s->mode_entry_t;

        if (quest_err_deg >= 0.0) {
            s->triad_err_deg   = quest_err_deg;
            s->triad_err_valid = 1;
        }

        int triad_ok  = (s->triad_err_valid &&
                         s->triad_err_deg < MM_TRIAD_ERR_DEG);
        int timed_out = (time_in_mode > MM_SUN_ACQ_TIMEOUT_S);

        if (triad_ok || timed_out) {
            if (timed_out && !triad_ok)
                GNC_LOG("  FSW: Sun acq timeout at t=%.1fs — proceeding with q_exit seed\n", t);
            _transition(s, MODE_FINE_POINTING, t);
        }
        return s->mode;
    }

    /* ── FINE_POINTING ↔ MOMENTUM_DUMP ───────────────────────── */
    if (s->mode == MODE_FINE_POINTING) {
        int pointing_ok = (!s->pointing_err_valid ||
                           s->pointing_err_deg < MM_DUMP_POINTING_GUARD_DEG);
        if (h_max > MM_DUMP_TRIGGER_NMS && pointing_ok)
            _transition(s, MODE_MOMENTUM_DUMP, t);
        return s->mode;
    }

    if (s->mode == MODE_MOMENTUM_DUMP) {
        if (h_max < MM_DUMP_COMPLETE_NMS)
            _transition(s, MODE_FINE_POINTING, t);
        return s->mode;
    }

    return s->mode;
}

const char *MM_mode_name(FSW_Mode mode) {
    switch (mode) {
        case MODE_SAFE_MODE:       return "SAFE_MODE";
        case MODE_DETUMBLE:        return "DETUMBLE";
        case MODE_SUN_ACQUISITION: return "SUN_ACQUISITION";
        case MODE_FINE_POINTING:   return "FINE_POINTING";
        case MODE_MOMENTUM_DUMP:   return "MOMENTUM_DUMP";
        default:                   return "UNKNOWN";
    }
}
