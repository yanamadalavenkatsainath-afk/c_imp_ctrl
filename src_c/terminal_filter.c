#include "terminal_filter.h"
#include "sim_config.h"

#include <math.h>
#include <string.h>

static double _norm3d(const double v[3]){
    return sqrt(v[0]*v[0] + v[1]*v[1] + v[2]*v[2]);
}

void TNF_reset(TermNavFilter *f){
    if(!f) return;
    memset(f, 0, sizeof(*f));
}

void TNF_update(TermNavFilter *f,
                const double pos_meas[3],
                const double vel_meas[3],
                int has_camera,
                double dt,
                double pos_out[3],
                double vel_out[3]){
    double innov[3];

    if(!f) return;
    if(dt <= 0.0) dt = 0.1;

    if(!f->initialized){
        for(int i=0;i<3;i++){
            f->pos[i] = pos_meas[i];
            f->vel[i] = vel_meas[i];
        }
        f->initialized = 1;
    }else{
        for(int i=0;i<3;i++) f->pos[i] += f->vel[i] * dt;
        if(has_camera){
            for(int i=0;i<3;i++) innov[i] = pos_meas[i] - f->pos[i];
            /* Soft gate — clip innovation to gate radius, still apply (matches Python). */
            double innov_norm = _norm3d(innov);
            if(innov_norm > CFG_TERM_NAV_GATE_M){
                double scale = CFG_TERM_NAV_GATE_M / innov_norm;
                for(int i=0;i<3;i++) innov[i] *= scale;
            }
            for(int i=0;i<3;i++){
                f->pos[i] += CFG_TERM_NAV_ALPHA * innov[i];
                f->vel[i] += (CFG_TERM_NAV_BETA / dt) * innov[i];
            }
        }
    }

    double vnorm = _norm3d(f->vel);
    if(vnorm > CFG_TERM_NAV_VMAX_MS){
        double scale = CFG_TERM_NAV_VMAX_MS / vnorm;
        for(int i=0;i<3;i++) f->vel[i] *= scale;
    }

    for(int i=0;i<3;i++){
        pos_out[i] = f->pos[i];
        vel_out[i] = f->vel[i];
    }
}
