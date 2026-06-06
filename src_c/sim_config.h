/*
 * sim_config.h - C mirror of the Python sim_config.py RPOD baseline.
 *
 * Keep these values synchronized with:
 *   C:/Users/Venkat/OneDrive/Desktop/appex/flight sim/sim_config.py
 *
 * This header is intentionally plain constants only.  Guidance modules may
 * keep their legacy RPOD_* aliases, but new C ports should include this file
 * directly when they need mission/capture/sensor configuration.
 */

#ifndef SIM_CONFIG_H
#define SIM_CONFIG_H

/* Chief orbit / environment */
#define CFG_CHIEF_A_KM                         42164.0
#define CFG_CHIEF_E                            0.0003
#define CFG_MU_GEO                             3.986004418e14

/* Deputy hardware */
#define CFG_DEP_MASS_KG                        50.0
#define CFG_DEP_THRUST_N                       1.0
#define CFG_DEP_CR                             1.5
#define CFG_DEP_AM                             0.00720

/* Timing */
#define CFG_DT_OUTER_S                         0.1
#define CFG_DT_INNER_S                         0.01
#define CFG_T_SIM_MAX_S                        80000.0

/* Formation hold */
#define CFG_FORMATION_OFFSET_X_M               0.0
#define CFG_FORMATION_OFFSET_Y_M              -1000.0
#define CFG_FORMATION_OFFSET_Z_M               0.0
#define CFG_FORM_HOLD_SETTLE_S                 300.0

/* Natural motion circumnavigation */
#define CFG_NMC_RADIUS_M                       75.0
#define CFG_NMC_VERTICAL_AMP_M                 15.0
#define CFG_NMC_PERIOD_S                       1800.0
#define CFG_NMC_KP                             1.5e-5
#define CFG_NMC_KD                             8.0e-3
#define CFG_NMC_ACCEL_MAX_MS2                  2.0e-4

/* Appendage keepout planner */
#define CFG_KEEPOUT_SOLAR_HALF_SPAN_M          6.0
#define CFG_KEEPOUT_ARRAY_SPHERE_RADIUS_M      0.80
#define CFG_KEEPOUT_ANTENNA_RADIUS_M           0.50
#define CFG_KEEPOUT_WARNING_MARGIN_M           0.75
#define CFG_KEEPOUT_GAIN                       1.0e-4
#define CFG_KEEPOUT_ACCEL_MAX_MS2              2.0e-4

/* Docking geometry */
#define CFG_DOCK_PORT_BODY_X_M                 0.0
#define CFG_DOCK_PORT_BODY_Y_M                 0.0
#define CFG_DOCK_PORT_BODY_Z_M                 0.5
#define CFG_DOCK_AXIS_BODY_X                   0.0
#define CFG_DOCK_AXIS_BODY_Y                   0.0
#define CFG_DOCK_AXIS_BODY_Z                   1.0
#define CFG_DEP_DOCK_AXIS_BODY_X               0.0
#define CFG_DEP_DOCK_AXIS_BODY_Y               0.0
#define CFG_DEP_DOCK_AXIS_BODY_Z               1.0
#define CFG_CHIEF_BODY_HALF_X_M                0.80
#define CFG_CHIEF_BODY_HALF_Y_M                0.80
#define CFG_CHIEF_BODY_HALF_Z_M                0.50
#define CFG_DEPUTY_BODY_HALF_X_M               0.30
#define CFG_DEPUTY_BODY_HALF_Y_M               0.30
#define CFG_DEPUTY_BODY_HALF_Z_M               0.40
#define CFG_DOCK_PORT_APERTURE_M               0.15
#define CFG_DOCK_CONE_HALF_ANGLE_DEG           15.0
#define CFG_DOCK_CONE_MIN_RANGE_M              0.05
#define CFG_DOCK_FACE_TOL_M                    0.05
#define CFG_DOCK_ALIGN_MAX_DEG                 10.0

/* Soft / hard capture */
#define CFG_DOCK_RANGE_M                       0.30
#define CFG_DOCK_VREL_MS                       0.050
#define CFG_SOFT_CAPTURE_RANGE_M               CFG_DOCK_RANGE_M
#define CFG_SOFT_CAPTURE_VREL_MS               CFG_DOCK_VREL_MS
#define CFG_HARD_CAPTURE_RANGE_M               0.08
#define CFG_HARD_CAPTURE_VREL_MS               0.010
#define CFG_HARD_CAPTURE_HOLD_S                5.0
#define CFG_SOFT_CAPTURE_HOLD_S                5.0
#define CFG_SOFT_CAPTURE_LATCH_VREL_MS         0.030
#define CFG_SOFT_CAPTURE_MAX_HOLD_S            1200.0
#define CFG_SOFT_CAPTURE_CORE_ALIGN_MAX_DEG    20.0
#define CFG_SOFT_CAPTURE_ENTRY_ALIGN_MAX_DEG   30.0
#define CFG_SOFT_CAPTURE_ATT_TORQUE_SCALE      1.0
#define CFG_SOFT_CAPTURE_RESTITUTION           0.10
#define CFG_SOFT_CAPTURE_TANGENTIAL_DAMPING    0.30
#define CFG_HARD_CAPTURE_GRACE_S               1.0

/* Navigation / estimation */
#define CFG_SIGMA_V_DOPPLER_MS                 0.005
#define CFG_TERM_NAV_ALPHA                     0.25
#define CFG_TERM_NAV_BETA                      0.02
#define CFG_TERM_NAV_VMAX_MS                   0.05
#define CFG_TERM_NAV_GATE_M                    0.25
/* Active DockPortSensor tuning used by main.py / monte_carlo.py.  These
 * intentionally override the raw Python PortTracker class defaults. */
#define CFG_PORT_TRACK_ALPHA                   0.40
#define CFG_PORT_TRACK_GATE_M                  0.25
#define CFG_PORT_TRACK_MAX_COAST_S             5.0
#define CFG_PORT_SENSOR_RANGE_M                10.0
#define CFG_PORT_SENSOR_SIGMA_M                0.020
#define CFG_PORT_SENSOR_NOISE_BASE_M           0.010
#define CFG_PORT_SENSOR_NOISE_RANGE_FRAC       0.002
#define CFG_CLOSE_PROX_NAV_RANGE_M             20.0

/* RPOD guidance */
#define CFG_LAMBERT_DV_CAP_MS                  2.0
#define CFG_LAMBERT_ARRIVAL_TOL_M              750.0
#define CFG_LAMBERT_PLAN_COOLDOWN_S            60.0
#define CFG_LAMBERT_TOF_CANDIDATE_COUNT        7
#define CFG_MAIN_TERMINAL_M                    10.0
#define CFG_PROX_FAR_FIELD_M                   500.0
#define CFG_PROX_TAU_S                         5.0
#define CFG_PROX_V_CLOSE_MAX_MS                0.200
#define CFG_PROX_V_CLOSE_NEAR_MS               0.005
#define CFG_PROX_V_CLOSE_NEAR_RANGE_M          10.0
#define CFG_TERMINAL_V_MAX_MS                  0.025
#define CFG_TERMINAL_V_CAPTURE_MS              0.0015
#define CFG_TERMINAL_PORT_SANITY_M             2.0
#define CFG_TERMINAL_BRAKE_DONE_MS             0.010
#define CFG_TERMINAL_BRAKE_ENTRY_MS            0.050
#define CFG_TERMINAL_SQRT_NORM_RANGE_M         0.8
#define CFG_TERMINAL_TAU_CLOSE_S               10.0
#define CFG_TERMINAL_TAU_MID_S                 8.0
#define CFG_TERMINAL_TAU_FAR_S                 6.0

/* Phase timeouts */
#define CFG_PROX_OPS_MAX_S                     50000.0
#define CFG_TERMINAL_MAX_S                     20000.0

/* Spin sync health */
#define CFG_SPIN_SYNC_MAX_OMEGA_RAD_S          0.017453292519943295
#define CFG_SPIN_SYNC_RATE_BLEND               0.35
#define CFG_SPIN_SYNC_MAX_RATE_RAD_S           0.008726646259971648

#endif /* SIM_CONFIG_H */
