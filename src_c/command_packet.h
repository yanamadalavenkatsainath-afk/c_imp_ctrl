/**
 * command_packet.h — Command output packets for PC SIL flight loop
 * =================================================================
 * Written by C flight loop each guidance tick (10 Hz).
 * Read by Python logger / sim driver.
 */

#ifndef COMMAND_PACKET_H
#define COMMAND_PACKET_H

#include <stdint.h>

/* ── Navigation state (TH-EKF output) ──────────────────────── */
typedef struct {
    double pos_lvlh[3];     /* estimated relative position [m]   */
    double vel_lvlh[3];     /* estimated relative velocity [m/s] */
    double pos_std[3];      /* 1-sigma position uncertainty [m]  */
    double vel_std[3];      /* 1-sigma velocity uncertainty [m/s]*/
    double range_m;         /* EKF range estimate [m]            */
} NavState;

/* ── Attitude state (MEKF output) ───────────────────────────── */
typedef struct {
    double q_wxyz[4];       /* attitude quaternion [w,x,y,z]     */
    double bias_xyz[3];     /* estimated gyro bias [rad/s]       */
} AttState;

/* ── Guidance command ────────────────────────────────────────── */
typedef struct {
    double accel_lvlh[3];   /* commanded acceleration [m/s²]     */
    int32_t guidance_mode;  /* 0=PROX_OPS, 1=TERMINAL, 2=DOCKED,
                               3=LOST_TARGET, 4=FORMATION_HOLD   */
} GuidanceCmd;

/* ── Timing telemetry ────────────────────────────────────────── */
typedef struct {
    uint64_t tick;              /* flight loop tick counter              */
    double   loop_time_ms;      /* wall time for this tick [ms]          */
    double   max_loop_time_ms;  /* max loop time seen so far [ms]        */
    uint32_t missed_deadlines;  /* ticks where loop_time > deadline      */
    double   deadline_ms;       /* loop deadline [ms]                    */
} TimingTelemetry;

/* ── Full command frame written each guidance tick ───────────── */
typedef struct {
    NavState        nav;
    AttState        att;
    GuidanceCmd     cmd;
    TimingTelemetry timing;
    uint8_t         ekf_updated;    /* 1 if EKF accepted measurement this tick */
    uint8_t         _pad[7];
} CommandFrame;

#endif /* COMMAND_PACKET_H */