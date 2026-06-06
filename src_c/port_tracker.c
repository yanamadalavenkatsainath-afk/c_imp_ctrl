#include "port_tracker.h"
#include "sim_config.h"

#include <math.h>
#include <string.h>

static double _norm3d(const double v[3]){
    return sqrt(v[0]*v[0] + v[1]*v[1] + v[2]*v[2]);
}

void PT_reset(PortTracker *pt){
    if(!pt) return;
    memset(pt, 0, sizeof(*pt));
}

int PT_update(PortTracker *pt,
              const double port_meas[3],
              int has_port,
              double dt,
              double port_out[3]){
    double innov[3];

    if(!pt) return 0;
    if(dt <= 0.0) dt = 0.1;

    if(has_port){
        if(!pt->initialized){
            for(int i=0;i<3;i++) pt->pos[i] = port_meas[i];
            pt->initialized = 1;
        }else{
            for(int i=0;i<3;i++) innov[i] = port_meas[i] - pt->pos[i];
            /* Soft gate — clip innovation to gate radius, still apply (matches Python). */
            double innov_norm = _norm3d(innov);
            if(innov_norm > CFG_PORT_TRACK_GATE_M){
                double scale = CFG_PORT_TRACK_GATE_M / innov_norm;
                for(int i=0;i<3;i++) innov[i] *= scale;
            }
            for(int i=0;i<3;i++) pt->pos[i] += CFG_PORT_TRACK_ALPHA * innov[i];
        }
        pt->coast_s = 0.0;
    }else if(pt->initialized){
        pt->coast_s += dt;
        if(pt->coast_s > CFG_PORT_TRACK_MAX_COAST_S){
            PT_reset(pt);
        }
    }

    if(!pt->initialized) return 0;
    for(int i=0;i<3;i++) port_out[i] = pt->pos[i];
    return 1;
}
