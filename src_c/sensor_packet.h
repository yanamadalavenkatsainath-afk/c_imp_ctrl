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

#include <stddef.h>
#include <stdint.h>

#define SENSOR_PACKET_VERSION 2u

typedef struct {
    uint32_t version;
    uint32_t checksum;
    uint64_t timestamp_tick;
} PacketMeta;

static inline uint32_t SENSOR_fnv1a32(const void *data, size_t n) {
    const uint8_t *p = (const uint8_t *)data;
    uint32_t h = 2166136261u;
    for (size_t i = 0; i < n; ++i) {
        h ^= (uint32_t)p[i];
        h *= 16777619u;
    }
    return h;
}

/* ── 100 Hz gyro packet (filled every tick) ─────────────────── */
typedef struct {
    double  omega_xyz[3];   /* body angular rate [rad/s] */
    uint8_t valid;          /* 1 = good, 0 = dropout     */
    uint8_t _pad[7];
    PacketMeta meta;
} GyroPacket;

/* ── 10 Hz ranging packet (range + angles) ──────────────────── */
typedef struct {
    double  range_m;        /* range [m]          */
    double  azimuth_rad;    /* azimuth [rad]       */
    double  elevation_rad;  /* elevation [rad]     */
    uint8_t valid;
    uint8_t _pad[7];
    PacketMeta meta;
} RangePacket;

/* ── 10 Hz camera packet (LVLH position) ────────────────────── */
typedef struct {
    double  pos_lvlh[3];    /* [dx, dy, dz] m      */
    double  R_diag[3];      /* diagonal of R (noise variances) */
    uint8_t valid;
    uint8_t _pad[7];
    PacketMeta meta;
} CameraPacket;

/* 10 Hz close-range docking port packet */
typedef struct {
    double  port_lvlh[3];   /* docking port position in LVLH [m] */
    double  port_axis_lvlh[3]; /* docking port outward axis in LVLH */
    double  port_vel_lvlh[3];  /* docking port velocity in LVLH [m/s] */
    double  R_body_to_lvlh[3][3]; /* chief body-to-LVLH rotation */
    double  R_diag[3];      /* diagonal of R (noise variances)   */
    uint8_t valid;
    uint8_t _pad[7];
    PacketMeta meta;
} PortPacket;

/* ── 10 Hz magnetometer packet ──────────────────────────────── */
typedef struct {
    double  body[3];        /* measured unit vector in body frame */
    double  inertial[3];    /* reference unit vector in ECI       */
    uint8_t valid;
    uint8_t _pad[7];
    PacketMeta meta;
} MagPacket;

/* ── Sun sensor packet ──────────────────────────────────────── */
typedef struct {
    double  body[3];        /* sun direction in body frame     */
    double  inertial[3];    /* sun direction in ECI            */
    uint8_t valid;
    uint8_t _pad[7];
    PacketMeta meta;
} SunPacket;

/* ── Combined sensor frame (written atomically by Python) ────── */
typedef struct {
    GyroPacket   gyro;
    RangePacket  range;
    CameraPacket camera;
    PortPacket   port;
    MagPacket    mag;
    SunPacket    sun;
    uint64_t     sim_tick;       /* simulation tick counter       */
    double       sim_time_s;     /* simulation time [s]           */
} SensorFrame;

static inline uint32_t GyroPacket_checksum(const GyroPacket *p) {
    return SENSOR_fnv1a32(p, offsetof(GyroPacket, meta));
}
static inline uint32_t RangePacket_checksum(const RangePacket *p) {
    return SENSOR_fnv1a32(p, offsetof(RangePacket, meta));
}
static inline uint32_t CameraPacket_checksum(const CameraPacket *p) {
    return SENSOR_fnv1a32(p, offsetof(CameraPacket, meta));
}
static inline uint32_t PortPacket_checksum(const PortPacket *p) {
    return SENSOR_fnv1a32(p, offsetof(PortPacket, meta));
}
static inline uint32_t MagPacket_checksum(const MagPacket *p) {
    return SENSOR_fnv1a32(p, offsetof(MagPacket, meta));
}
static inline uint32_t SunPacket_checksum(const SunPacket *p) {
    return SENSOR_fnv1a32(p, offsetof(SunPacket, meta));
}

static inline int PacketMeta_ok(const PacketMeta *meta,
                                uint32_t expected_checksum,
                                uint64_t now_tick,
                                uint64_t max_age_ticks) {
    if (meta->version != SENSOR_PACKET_VERSION) return 0;
    if (meta->checksum != expected_checksum) return 0;
    if (meta->timestamp_tick > now_tick) return 0;
    if ((now_tick - meta->timestamp_tick) > max_age_ticks) return 0;
    return 1;
}

#endif /* SENSOR_PACKET_H */
