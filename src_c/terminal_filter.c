#include "terminal_filter.h"

#include <math.h>
#include <string.h>

#define TNF_ALPHA              0.35
#define TNF_BETA               0.06
#define TNF_INNOV_GATE_M       0.25
#define TNF_VEL_MAX_MS         0.10

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
            if(_norm3d(innov) <= TNF_INNOV_GATE_M){
                for(int i=0;i<3;i++){
                    f->pos[i] += TNF_ALPHA * innov[i];
                    f->vel[i] += (TNF_BETA / dt) * innov[i];
                }
            }
        }
    }

    double vnorm = _norm3d(f->vel);
    if(vnorm > TNF_VEL_MAX_MS){
        double scale = TNF_VEL_MAX_MS / vnorm;
        for(int i=0;i<3;i++) f->vel[i] *= scale;
    }

    for(int i=0;i<3;i++){
        pos_out[i] = f->pos[i];
        vel_out[i] = f->vel[i];
    }
}
