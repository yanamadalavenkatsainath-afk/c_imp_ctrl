#include "keepout_planner.h"
#include <math.h>

static double ko_norm3(const double v[3]) {
    return sqrt(v[0]*v[0] + v[1]*v[1] + v[2]*v[2]);
}

static void ko_clamp(double v[3], double vmax) {
    double n = ko_norm3(v);
    if (n > vmax && n > 0.0) {
        double s = vmax / n;
        v[0] *= s; v[1] *= s; v[2] *= s;
    }
}

static void add_zone(KeepoutPlanner *planner,
                     double x, double y, double z,
                     double radius_m) {
    if (planner->zone_count >= KEEPOUT_MAX_ZONES) return;
    KeepoutZone *zone = &planner->zones[planner->zone_count++];
    zone->center_body[0] = x;
    zone->center_body[1] = y;
    zone->center_body[2] = z;
    zone->radius_m = radius_m;
}

void Keepout_default_appendage_zones(KeepoutPlanner *planner) {
    planner->zone_count = 0;
    planner->warning_margin_m = CFG_KEEPOUT_WARNING_MARGIN_M;
    planner->gain = CFG_KEEPOUT_GAIN;
    planner->accel_max = CFG_KEEPOUT_ACCEL_MAX_MS2;

    double span = CFG_KEEPOUT_SOLAR_HALF_SPAN_M;
    double fracs[3] = {1.0/3.0, 2.0/3.0, 1.0};
    for (int sign_i = 0; sign_i < 2; ++sign_i) {
        double sign = sign_i == 0 ? 1.0 : -1.0;
        for (int i = 0; i < 3; ++i) {
            add_zone(planner, 0.0, sign * fracs[i] * span, 0.0,
                     CFG_KEEPOUT_ARRAY_SPHERE_RADIUS_M);
        }
    }
    add_zone(planner, 0.0, 0.0, -0.70, CFG_KEEPOUT_ANTENNA_RADIUS_M);
}

void Keepout_compute(const KeepoutPlanner *planner,
                     const double dep_lvlh[3],
                     const double R_body_to_lvlh[3][3],
                     KeepoutResult *out) {
    out->accel[0] = 0.0;
    out->accel[1] = 0.0;
    out->accel[2] = 0.0;
    out->min_clearance_m = INFINITY;
    out->active_count = 0;

    for (int z = 0; z < planner->zone_count; ++z) {
        const KeepoutZone *zone = &planner->zones[z];
        double center[3];
        for (int r = 0; r < 3; ++r) {
            center[r] = R_body_to_lvlh[r][0] * zone->center_body[0]
                      + R_body_to_lvlh[r][1] * zone->center_body[1]
                      + R_body_to_lvlh[r][2] * zone->center_body[2];
        }

        double delta[3] = {
            dep_lvlh[0] - center[0],
            dep_lvlh[1] - center[1],
            dep_lvlh[2] - center[2]
        };
        double dist = ko_norm3(delta);
        double clearance = dist - zone->radius_m;
        if (clearance < out->min_clearance_m) out->min_clearance_m = clearance;

        double trigger = zone->radius_m + planner->warning_margin_m;
        if (dist < trigger) {
            double inv_dist = dist > 1e-9 ? 1.0 / dist : 0.0;
            double strength = planner->gain * (trigger - dist)
                            / (planner->warning_margin_m > 1e-9
                               ? planner->warning_margin_m : 1e-9);
            out->accel[0] += strength * delta[0] * inv_dist;
            out->accel[1] += strength * delta[1] * inv_dist;
            out->accel[2] += strength * delta[2] * inv_dist;
            out->active_count += 1;
        }
    }

    ko_clamp(out->accel, planner->accel_max);
}
