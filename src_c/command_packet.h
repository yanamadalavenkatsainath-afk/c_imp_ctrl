/**
 * command_packet.h — Command output packets for PC SIL flight loop v2
 */
#ifndef COMMAND_PACKET_H
#define COMMAND_PACKET_H
#include <stdint.h>
typedef struct {
    double pos_lvlh[3]; double vel_lvlh[3]; double pos_std[3];
    double vel_std[3];  double range_m;
} NavState;
typedef struct {
    double q_wxyz[4]; double bias_xyz[3];
    double quest_quality; double pointing_err_deg;
} AttState;
typedef struct {
    double  accel_lvlh[3];
    double  torque_rw[3];
    double  dipole_mtq[3];
    int32_t fsw_mode;
    int32_t rpod_mode;
} GuidanceCmd;
typedef struct {
    uint64_t tick; double loop_time_ms; double max_loop_time_ms;
    uint32_t missed_deadlines; double deadline_ms;
} TimingTelemetry;
typedef struct {
    NavState nav; AttState att; GuidanceCmd cmd;
    TimingTelemetry timing;
    uint8_t ekf_updated; uint8_t _pad[7];
} CommandFrame;
#endif