/**
 * spin_sync_controller.h -- target spin synchronisation rate command.
 *
 * Small hardware-facing port of control/spin_sync_controller.py.  The module
 * owns only command shaping: transform/clamp/blend a target angular-rate
 * estimate into the deputy body-frame rate command used by ADCS.
 */

#ifndef SPIN_SYNC_CONTROLLER_H
#define SPIN_SYNC_CONTROLLER_H

typedef struct {
    double rate_blend;
    double max_rate_rad_s;
    double omega_cmd[3];
} SpinSyncController;

void SSC_init(SpinSyncController *s,
              double rate_blend,
              double max_rate_rad_s);

void SSC_reset(SpinSyncController *s);

void SSC_compute_rate_body(SpinSyncController *s,
                           const double omega_target_body[3],
                           double omega_cmd_out[3]);

void SSC_compute_rate_command(SpinSyncController *s,
                              const double omega_target_lvlh[3],
                              const double R_lvlh_to_body[3][3],
                              double omega_cmd_out[3]);

double SSC_sync_quality(const double omega_body[3],
                        const double omega_cmd[3]);

#endif /* SPIN_SYNC_CONTROLLER_H */
