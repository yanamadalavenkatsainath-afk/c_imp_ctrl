#include "nmc_guidance.h"
#include <math.h>

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

static double nmc_norm3(const double v[3]) {
    return sqrt(v[0]*v[0] + v[1]*v[1] + v[2]*v[2]);
}

static void nmc_clamp(double v[3], double vmax) {
    double n = nmc_norm3(v);
    if (n > vmax && n > 0.0) {
        double s = vmax / n;
        v[0] *= s; v[1] *= s; v[2] *= s;
    }
}

void NMC_default_config(NMC_Config *cfg) {
    cfg->radius_m = CFG_NMC_RADIUS_M;
    cfg->vertical_amp_m = CFG_NMC_VERTICAL_AMP_M;
    cfg->period_s = CFG_NMC_PERIOD_S;
    cfg->kp = CFG_NMC_KP;
    cfg->kd = CFG_NMC_KD;
    cfg->accel_max = CFG_NMC_ACCEL_MAX_MS2;
}

void NMC_reference(const NMC_Config *cfg,
                   double t_since_start_s,
                   double ref_pos[3],
                   double ref_vel[3]) {
    double tau = t_since_start_s > 0.0 ? t_since_start_s : 0.0;
    double period = cfg->period_s > 1.0 ? cfg->period_s : 1.0;
    double w = 2.0 * M_PI / period;
    double th = w * tau;

    ref_pos[0] = cfg->radius_m * cos(th);
    ref_pos[1] = cfg->radius_m * sin(th);
    ref_pos[2] = cfg->vertical_amp_m * sin(0.5 * th);

    ref_vel[0] = -cfg->radius_m * w * sin(th);
    ref_vel[1] =  cfg->radius_m * w * cos(th);
    ref_vel[2] =  0.5 * cfg->vertical_amp_m * w * cos(0.5 * th);
}

void NMC_compute(const NMC_Config *cfg,
                 const NMC_State *state,
                 double t_since_start_s,
                 double accel_out[3],
                 double ref_pos_out[3],
                 double ref_vel_out[3]) {
    NMC_reference(cfg, t_since_start_s, ref_pos_out, ref_vel_out);
    for (int i = 0; i < 3; ++i) {
        accel_out[i] = cfg->kp * (ref_pos_out[i] - state->pos[i])
                     + cfg->kd * (ref_vel_out[i] - state->vel[i]);
    }
    nmc_clamp(accel_out, cfg->accel_max);
}
