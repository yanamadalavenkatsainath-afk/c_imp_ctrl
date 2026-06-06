/*
 * keepout_planner.h - Moving appendage keepout avoidance layer.
 *
 * Mirrors control/keepout_planner.py.  Zones are spheres in chief body frame;
 * caller supplies chief body-to-LVLH rotation and deputy LVLH position.
 */

#ifndef KEEPOUT_PLANNER_H
#define KEEPOUT_PLANNER_H

#include "sim_config.h"

#define KEEPOUT_MAX_ZONES 8

typedef struct {
    double center_body[3];
    double radius_m;
} KeepoutZone;

typedef struct {
    KeepoutZone zones[KEEPOUT_MAX_ZONES];
    int zone_count;
    double warning_margin_m;
    double gain;
    double accel_max;
} KeepoutPlanner;

typedef struct {
    double accel[3];
    double min_clearance_m;
    int active_count;
} KeepoutResult;

void Keepout_default_appendage_zones(KeepoutPlanner *planner);
void Keepout_compute(const KeepoutPlanner *planner,
                     const double dep_lvlh[3],
                     const double R_body_to_lvlh[3][3],
                     KeepoutResult *out);

#endif /* KEEPOUT_PLANNER_H */
