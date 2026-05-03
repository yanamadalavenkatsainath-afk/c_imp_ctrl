/**
 * sensor_packet.h — Sensor input packets for PC SIL flight loop
 * ==============================================================
 * Filled by Python sim (via ctypes) and read by C flight loop each tick.
 * All fields use plain C types so ctypes Structure mapping is trivial.
 *
 * Packet validity flags:
 *   valid = 1  → C code uses this measurement
 *   valid = 0  → sensor dropout; C code skips update, exercises guard logic
 */

#ifndef SENSOR_PACKET_H
#define SENSOR_PACKET_H

#include <stdint.h>

/* ── 100 Hz gyro packet (filled every tick) ─────────────────── */
typedef struct {
    double  omega_xyz[3];   /* body angular rate [rad/s] */
    uint8_t valid;          /* 1 = good, 0 = dropout     */
    uint8_t _pad[7];
} GyroPacket;

/* ── 10 Hz ranging packet (range + angles) ──────────────────── */
typedef struct {
    double  range_m;        /* range [m]          */
    double  azimuth_rad;    /* azimuth [rad]       */
    double  elevation_rad;  /* elevation [rad]     */
    uint8_t valid;
    uint8_t _pad[7];
} RangePacket;

/* ── 10 Hz camera packet (LVLH position) ────────────────────── */
typedef struct {
    double  pos_lvlh[3];    /* [dx, dy, dz] m      */
    double  R_diag[3];      /* diagonal of R (noise variances) */
    uint8_t valid;
    uint8_t _pad[7];
} CameraPacket;

/* ── 10 Hz magnetometer packet ──────────────────────────────── */
typedef struct {
    double  body[3];        /* measured unit vector in body frame */
    double  inertial[3];    /* reference unit vector in ECI       */
    uint8_t valid;
    uint8_t _pad[7];
} MagPacket;

/* ── Sun sensor packet ──────────────────────────────────────── */
typedef struct {
    double  body[3];        /* sun direction in body frame     */
    double  inertial[3];    /* sun direction in ECI            */
    uint8_t valid;
    uint8_t _pad[7];
} SunPacket;

/* ── Combined sensor frame (written atomically by Python) ────── */
typedef struct {
    GyroPacket   gyro;
    RangePacket  range;
    CameraPacket camera;
    MagPacket    mag;
    SunPacket    sun;
    uint64_t     sim_tick;       /* simulation tick counter       */
    double       sim_time_s;     /* simulation time [s]           */
} SensorFrame;

#endif /* SENSOR_PACKET_H */