#include "port_tracker.h"

#include <math.h>
#include <string.h>

#define PT_ALPHA             0.40
#define PT_INNOV_GATE_M      0.25
#define PT_MAX_COAST_S       5.0

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
            if(_norm3d(innov) <= PT_INNOV_GATE_M){
                for(int i=0;i<3;i++) pt->pos[i] += PT_ALPHA * innov[i];
            }
        }
        pt->coast_s = 0.0;
    }else if(pt->initialized){
        pt->coast_s += dt;
        if(pt->coast_s > PT_MAX_COAST_S){
            PT_reset(pt);
        }
    }

    if(!pt->initialized) return 0;
    for(int i=0;i<3;i++) port_out[i] = pt->pos[i];
    return 1;
}
