#include "spin_sync_controller.h"
#include <math.h>
#include <string.h>

static double _norm3(const double v[3]) {
    return sqrt(v[0]*v[0] + v[1]*v[1] + v[2]*v[2]);
}

void SSC_init(SpinSyncController *s,
              double rate_blend,
              double max_rate_rad_s) {
    memset(s, 0, sizeof(*s));
    s->rate_blend = rate_blend;
    s->max_rate_rad_s = max_rate_rad_s;
}

void SSC_reset(SpinSyncController *s) {
    s->omega_cmd[0] = 0.0;
    s->omega_cmd[1] = 0.0;
    s->omega_cmd[2] = 0.0;
}

void SSC_compute_rate_body(SpinSyncController *s,
                           const double omega_target_body[3],
                           double omega_cmd_out[3]) {
    double omega_body[3] = {
        omega_target_body[0],
        omega_target_body[1],
        omega_target_body[2]
    };
    double mag = _norm3(omega_body);
    if(mag > s->max_rate_rad_s && mag > 1e-12) {
        double scale = s->max_rate_rad_s / mag;
        omega_body[0] *= scale;
        omega_body[1] *= scale;
        omega_body[2] *= scale;
    }

    for(int i=0; i<3; ++i) {
        s->omega_cmd[i] = (1.0 - s->rate_blend) * s->omega_cmd[i]
                        + s->rate_blend * omega_body[i];
        omega_cmd_out[i] = s->omega_cmd[i];
    }
}

void SSC_compute_rate_command(SpinSyncController *s,
                              const double omega_target_lvlh[3],
                              const double R_lvlh_to_body[3][3],
                              double omega_cmd_out[3]) {
    double omega_body[3];
    for(int r=0; r<3; ++r) {
        omega_body[r] = R_lvlh_to_body[r][0] * omega_target_lvlh[0]
                      + R_lvlh_to_body[r][1] * omega_target_lvlh[1]
                      + R_lvlh_to_body[r][2] * omega_target_lvlh[2];
    }
    SSC_compute_rate_body(s, omega_body, omega_cmd_out);
}

double SSC_sync_quality(const double omega_body[3],
                        const double omega_cmd[3]) {
    double denom = _norm3(omega_cmd);
    if(denom < 1e-9) denom = 1e-9;

    double err_vec[3] = {
        omega_body[0] - omega_cmd[0],
        omega_body[1] - omega_cmd[1],
        omega_body[2] - omega_cmd[2]
    };
    double q = 1.0 - _norm3(err_vec) / denom;
    if(q < 0.0) return 0.0;
    if(q > 1.0) return 1.0;
    return q;
}
