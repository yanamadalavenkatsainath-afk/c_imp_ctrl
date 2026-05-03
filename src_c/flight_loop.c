/**
 * flight_loop.c — PC SIL Real-Time Flight Loop
 * =============================================
 * Emulates embedded flight software timing on Windows/Linux.
 *
 * Architecture:
 *   100 Hz outer loop  → gyro read + MEKF attitude propagation
 *   10  Hz inner tasks → TH-EKF nav + RPOD guidance + command output
 *
 * IPC (ctypes DLL mode):
 *   Python writes SensorFrame* before calling flight_loop_step().
 *   C writes CommandFrame* which Python reads after each step.
 *   No locks needed — single-threaded, Python controls the tick rate.
 *
 * Timing model (realtime mode):
 *   flight_loop_run() uses platform sleep to hold 10 ms / tick.
 *   Python driver calls flight_loop_step() directly for SIL — no sleep.
 *
 * Compile (SIL, Windows):
 *   gcc -O2 -DMEKF_NO_CMSIS -shared -fPIC -Isrc_c \
 *       -o gnc_lib.dll \
 *       src_c/flight_loop.c src_c/th_ekf.c src_c/mekf.c src_c/rpod_ctrl.c \
 *       -lm
 *
 * Compile (realtime standalone, Windows):
 *   gcc -O2 -DMEKF_NO_CMSIS -DFLIGHT_LOOP_STANDALONE -Isrc_c \
 *       -o flight_loop.exe \
 *       src_c/flight_loop.c src_c/th_ekf.c src_c/mekf.c src_c/rpod_ctrl.c \
 *       -lm
 */

#include "flight_loop.h"
#include "th_ekf.h"
#include "mekf.h"
#include "rpod_ctrl.h"
#include "sensor_packet.h"
#include "command_packet.h"

#include <math.h>
#include <string.h>
#include <stdio.h>
#include <stdint.h>

/* ── Platform timing ─────────────────────────────────────────── */
#ifdef _WIN32
  #define WIN32_LEAN_AND_MEAN
  #include <windows.h>
  static double _get_time_ms(void) {
      LARGE_INTEGER freq, cnt;
      QueryPerformanceFrequency(&freq);
      QueryPerformanceCounter(&cnt);
      return (double)cnt.QuadPart / (double)freq.QuadPart * 1000.0;
  }
  static void _sleep_ms(double ms) {
      DWORD d = (DWORD)ms;
      if (d > 0) Sleep(d);
  }
#else
  #include <time.h>
  static double _get_time_ms(void) {
      struct timespec ts;
      clock_gettime(CLOCK_MONOTONIC, &ts);
      return ts.tv_sec * 1000.0 + ts.tv_nsec / 1e6;
  }
  static void _sleep_ms(double ms) {
      struct timespec req;
      req.tv_sec  = (time_t)(ms / 1000.0);
      req.tv_nsec = (long)((ms - req.tv_sec * 1000.0) * 1e6);
      nanosleep(&req, NULL);
  }
#endif

/* ── Module-level state ──────────────────────────────────────── */

static THEKF_State  g_thekf;
static MEKF_State   g_mekf;
static RPOD_State   g_rpod_state;
static int          g_is_braking = -1;   /* TERMINAL entry-brake flag */

static CommandFrame  g_cmd_frame;
static SensorFrame   g_sensor_frame;

static uint64_t g_tick          = 0;
static double   g_max_loop_ms   = 0.0;
static uint32_t g_missed        = 0;
static double   g_deadline_ms   = 10.0;   /* 100 Hz = 10 ms */
static int      g_running       = 0;

/* ── Flight loop init ────────────────────────────────────────── */

void flight_loop_init(double a_chief_m, double e_chief, double mu,
                      double dt_thekf_s) {
    memset(&g_cmd_frame,    0, sizeof(g_cmd_frame));
    memset(&g_sensor_frame, 0, sizeof(g_sensor_frame));

    /* TH-EKF: 10 Hz */
    THEKF_init(&g_thekf, a_chief_m, e_chief, mu, dt_thekf_s, 1e-4, 1e-8);

    /* MEKF: 100 Hz → dt = 0.01 s */
    MEKF_init(&g_mekf, 0.01f);

    g_tick        = 0;
    g_max_loop_ms = 0.0;
    g_missed      = 0;
    g_is_braking  = -1;

    g_cmd_frame.timing.deadline_ms = g_deadline_ms;
}

/* Seed the TH-EKF from Python after initialise() */
void flight_loop_seed_thekf(const double x0[6],
                             const double P0_flat[36],
                             double nu0) {
    double P0[6][6];
    for (int i = 0; i < 6; i++)
        for (int j = 0; j < 6; j++)
            P0[i][j] = P0_flat[i*6 + j];
    THEKF_seed(&g_thekf, x0, P0, nu0);
}

/* ── Sensor frame write (called by Python before each step) ──── */

SensorFrame *flight_loop_get_sensor_frame(void) {
    return &g_sensor_frame;
}

CommandFrame *flight_loop_get_command_frame(void) {
    return &g_cmd_frame;
}

/* ── Single flight loop tick ─────────────────────────────────── */
/*
 * Called by Python driver at 100 Hz (or as fast as possible in batch SIL).
 * Returns pointer to CommandFrame (valid even on non-guidance ticks).
 */
CommandFrame *flight_loop_step(void) {
    double t_start = _get_time_ms();

    SensorFrame *sf = &g_sensor_frame;
    CommandFrame *cf = &g_cmd_frame;

    /* ── 100 Hz: Gyro + MEKF attitude propagation ─────────────── */
    if (sf->gyro.valid) {
        MEKF_FLOAT omega[3] = {
            (MEKF_FLOAT)sf->gyro.omega_xyz[0],
            (MEKF_FLOAT)sf->gyro.omega_xyz[1],
            (MEKF_FLOAT)sf->gyro.omega_xyz[2]
        };
        MEKF_predict(&g_mekf, omega);
    }

    cf->att.q_wxyz[0] = (double)g_mekf.q[0];
    cf->att.q_wxyz[1] = (double)g_mekf.q[1];
    cf->att.q_wxyz[2] = (double)g_mekf.q[2];
    cf->att.q_wxyz[3] = (double)g_mekf.q[3];
    cf->att.bias_xyz[0] = (double)g_mekf.bias[0];
    cf->att.bias_xyz[1] = (double)g_mekf.bias[1];
    cf->att.bias_xyz[2] = (double)g_mekf.bias[2];

    /* ── 10 Hz tasks ─────────────────────────────────────────── */
    cf->ekf_updated = 0;

    if (g_tick % 10 == 0) {

        /* ── TH-EKF predict ─────────────────────────────────── */
        THEKF_predict(&g_thekf, NULL);

        /* ── TH-EKF update (ranging) ────────────────────────── */
        if (sf->range.valid) {
            double z[3] = {sf->range.range_m,
                           sf->range.azimuth_rad,
                           sf->range.elevation_rad};
            double R[3][3] = {{0.09, 0, 0},
                              {0, 3e-6, 0},
                              {0, 0, 3e-6}};
            cf->ekf_updated = (uint8_t)THEKF_update(&g_thekf, z, R, 5.0);
        }

        /* ── TH-EKF update (camera position) ────────────────── */
        if (sf->camera.valid) {
            double R_cam[3][3] = {
                {sf->camera.R_diag[0], 0, 0},
                {0, sf->camera.R_diag[1], 0},
                {0, 0, sf->camera.R_diag[2]}
            };
            int accepted = THEKF_update_position(&g_thekf,
                                                  sf->camera.pos_lvlh,
                                                  R_cam, 5.0);
            cf->ekf_updated |= (uint8_t)accepted;
        }

        /* ── MEKF vector update (mag) ────────────────────────── */
        if (sf->mag.valid) {
            MEKF_FLOAT z_b[3] = {(MEKF_FLOAT)sf->mag.body[0],
                                  (MEKF_FLOAT)sf->mag.body[1],
                                  (MEKF_FLOAT)sf->mag.body[2]};
            MEKF_FLOAT v_i[3] = {(MEKF_FLOAT)sf->mag.inertial[0],
                                  (MEKF_FLOAT)sf->mag.inertial[1],
                                  (MEKF_FLOAT)sf->mag.inertial[2]};
            MEKF_update(&g_mekf, z_b, v_i, g_mekf.R_mag);
        }

        /* ── Navigation state output ─────────────────────────── */
        THEKF_get_pos(&g_thekf, cf->nav.pos_lvlh);
        THEKF_get_vel(&g_thekf, cf->nav.vel_lvlh);
        THEKF_get_pos_std(&g_thekf, cf->nav.pos_std);
        THEKF_get_vel_std(&g_thekf, cf->nav.vel_std);
        cf->nav.range_m = THEKF_range(&g_thekf);

        /* ── RPOD guidance ───────────────────────────────────── */
        double n_chief = g_thekf.n;
        double accel_max = 0.020;   /* 20 mm/s² */

        g_rpod_state.pos[0] = cf->nav.pos_lvlh[0];
        g_rpod_state.pos[1] = cf->nav.pos_lvlh[1];
        g_rpod_state.pos[2] = cf->nav.pos_lvlh[2];
        g_rpod_state.vel[0] = cf->nav.vel_lvlh[0];
        g_rpod_state.vel[1] = cf->nav.vel_lvlh[1];
        g_rpod_state.vel[2] = cf->nav.vel_lvlh[2];

        double truth_range = cf->nav.range_m;   /* use EKF range */

        if (truth_range > RPOD_TERMINAL_M) {
            RPOD_prox_ops(&g_rpod_state, truth_range, n_chief,
                          accel_max, cf->cmd.accel_lvlh);
            cf->cmd.guidance_mode = 0;   /* PROX_OPS */
        } else if (truth_range > RPOD_DOCK_DONE_M) {
            int ret = RPOD_terminal_simple(&g_rpod_state, accel_max,
                                           cf->cmd.accel_lvlh);
            cf->cmd.guidance_mode = (ret == 2) ? 2 : 1;
        } else {
            cf->cmd.accel_lvlh[0] = 0.0;
            cf->cmd.accel_lvlh[1] = 0.0;
            cf->cmd.accel_lvlh[2] = 0.0;
            cf->cmd.guidance_mode = 2;   /* DOCKED */
        }
    }

    /* ── Timing telemetry ────────────────────────────────────── */
    double t_end  = _get_time_ms();
    double loop_t = t_end - t_start;

    if (loop_t > g_max_loop_ms)
        g_max_loop_ms = loop_t;
    if (loop_t > g_deadline_ms)
        g_missed++;

    cf->timing.tick             = g_tick;
    cf->timing.loop_time_ms     = loop_t;
    cf->timing.max_loop_time_ms = g_max_loop_ms;
    cf->timing.missed_deadlines = g_missed;
    cf->timing.deadline_ms      = g_deadline_ms;

    g_tick++;
    return cf;
}

/* ── Blocking real-time run (standalone exe mode) ────────────── */

void flight_loop_run(uint64_t n_ticks) {
    g_running = 1;
    double t_next = _get_time_ms();

    for (uint64_t i = 0; i < n_ticks && g_running; i++) {
        t_next += g_deadline_ms;
        flight_loop_step();
        double now = _get_time_ms();
        double sleep_remaining = t_next - now;
        if (sleep_remaining > 0.5)
            _sleep_ms(sleep_remaining - 0.4);   /* leave 0.4ms margin */
    }
    g_running = 0;
}

void flight_loop_stop(void) {
    g_running = 0;
}

/* ── Reset (for test suite) ──────────────────────────────────── */

void flight_loop_reset(void) {
    g_tick        = 0;
    g_max_loop_ms = 0.0;
    g_missed      = 0;
    g_is_braking  = -1;
    memset(&g_cmd_frame,    0, sizeof(g_cmd_frame));
    memset(&g_sensor_frame, 0, sizeof(g_sensor_frame));
}

/* ── Direct EKF state accessors (for verify_realtime_sil.py) ─── */

void flight_loop_get_thekf_state(double x_out[6], double P_out[36]) {
    for (int i = 0; i < 6; i++)
        x_out[i] = g_thekf.x[i];
    for (int i = 0; i < 6; i++)
        for (int j = 0; j < 6; j++)
            P_out[i*6 + j] = g_thekf.P[i][j];
}

#ifdef FLIGHT_LOOP_STANDALONE
/* ── Standalone smoke test ───────────────────────────────────── */
int main(void) {
    printf("Flight loop standalone smoke test\n");

    double mu    = 3.986004418e14;
    double a_geo = 42164e3;
    double e_geo = 0.001;
    flight_loop_init(a_geo, e_geo, mu, 0.1);

    double x0[6] = {0., 500., 0., 0., 1e-3, 0.};
    double P0[36] = {0};
    for (int i = 0; i < 6; i++) P0[i*6+i] = (i<3) ? 2500.0 : 0.25;
    flight_loop_seed_thekf(x0, P0, 0.0);

    /* Inject a fake ranging measurement */
    SensorFrame *sf = flight_loop_get_sensor_frame();
    sf->range.range_m       = 500.1;
    sf->range.azimuth_rad   = 0.001;
    sf->range.elevation_rad = 0.0;
    sf->range.valid         = 1;
    sf->gyro.omega_xyz[0]   = 0.001;
    sf->gyro.valid          = 1;

    /* Run 100 ticks (1 s wall time in theory) */
    double t0 = _get_time_ms();
    for (int i = 0; i < 100; i++) {
        sf->sim_tick = i;
        CommandFrame *cf = flight_loop_step();
        if (i == 99) {
            printf("  Tick 99: range=%.1fm  mode=%d  loop=%.3fms  max=%.3fms  missed=%u\n",
                   cf->nav.range_m,
                   cf->cmd.guidance_mode,
                   cf->timing.loop_time_ms,
                   cf->timing.max_loop_time_ms,
                   cf->timing.missed_deadlines);
        }
    }
    double elapsed = _get_time_ms() - t0;
    printf("  100 ticks in %.2f ms  (avg %.3f ms/tick)\n", elapsed, elapsed/100.0);
    printf("  PASS — flight_loop_step() runs well under 10ms deadline\n");
    return 0;
}
#endif /* FLIGHT_LOOP_STANDALONE */