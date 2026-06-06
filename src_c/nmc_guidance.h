/*
 * nmc_guidance.h - Natural-motion circumnavigation guidance kernel.
 *
 * Mirrors control/nmc_guidance.py without Python class state.  This module is
 * suitable for flight software calls when the RPOD sequencer chooses an
 * observability/stand-off circumnavigation mode.
 */

#ifndef NMC_GUIDANCE_H
#define NMC_GUIDANCE_H

#include "sim_config.h"

typedef struct {
    double radius_m;
    double vertical_amp_m;
    double period_s;
    double kp;
    double kd;
    double accel_max;
} NMC_Config;

typedef struct {
    double pos[3];
    double vel[3];
} NMC_State;

void NMC_default_config(NMC_Config *cfg);
void NMC_reference(const NMC_Config *cfg,
                   double t_since_start_s,
                   double ref_pos[3],
                   double ref_vel[3]);
void NMC_compute(const NMC_Config *cfg,
                 const NMC_State *state,
                 double t_since_start_s,
                 double accel_out[3],
                 double ref_pos_out[3],
                 double ref_vel_out[3]);

#endif /* NMC_GUIDANCE_H */
