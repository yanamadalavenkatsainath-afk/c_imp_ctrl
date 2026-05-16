#ifndef TERMINAL_FILTER_H
#define TERMINAL_FILTER_H

/*
 * terminal_filter.h
 * Lightweight terminal-phase relative navigation smoothing.
 *
 * This is intentionally a small FSW alpha-beta filter, not a replacement
 * for TH-EKF. It smooths the terminal relative COM state and gates large
 * vision innovations before RPOD terminal guidance consumes the state.
 */

typedef struct {
    int initialized;
    double pos[3];
    double vel[3];
} TermNavFilter;

void TNF_reset(TermNavFilter *f);

void TNF_update(TermNavFilter *f,
                const double pos_meas[3],
                const double vel_meas[3],
                int has_camera,
                double dt,
                double pos_out[3],
                double vel_out[3]);

#endif /* TERMINAL_FILTER_H */
