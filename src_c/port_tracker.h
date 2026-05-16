#ifndef PORT_TRACKER_H
#define PORT_TRACKER_H

/*
 * port_tracker.h
 * Lightweight docking-port measurement tracker.
 *
 * Tracks the docking port independently from the terminal COM navigation
 * filter so camera/port measurement gating and short coasts are explicit
 * flight-software state, not hidden inside RPOD control logic.
 */

typedef struct {
    int initialized;
    double pos[3];
    double coast_s;
} PortTracker;

void PT_reset(PortTracker *pt);

int PT_update(PortTracker *pt,
              const double port_meas[3],
              int has_port,
              double dt,
              double port_out[3]);

#endif /* PORT_TRACKER_H */
