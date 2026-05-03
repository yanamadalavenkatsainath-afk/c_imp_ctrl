/**
 * flight_loop.h — PC SIL Real-Time Flight Loop API
 * =================================================
 * Exposed via gnc_lib.dll for Python ctypes driver.
 */

#ifndef FLIGHT_LOOP_H
#define FLIGHT_LOOP_H

#include <stdint.h>
#include "sensor_packet.h"
#include "command_packet.h"

#ifdef __cplusplus
extern "C" {
#endif

/**
 * flight_loop_init — configure all subsystems.
 * Call once before any step.
 */
void flight_loop_init(double a_chief_m, double e_chief, double mu,
                      double dt_thekf_s);

/**
 * flight_loop_seed_thekf — inject initial EKF state from Python.
 * P0_flat: row-major 6×6 covariance (36 doubles).
 */
void flight_loop_seed_thekf(const double x0[6],
                             const double P0_flat[36],
                             double nu0);

/**
 * flight_loop_get_sensor_frame — returns pointer to internal SensorFrame.
 * Python fills this before each call to flight_loop_step().
 */
SensorFrame  *flight_loop_get_sensor_frame(void);

/**
 * flight_loop_get_command_frame — returns pointer to internal CommandFrame.
 * Python reads this after each call to flight_loop_step().
 */
CommandFrame *flight_loop_get_command_frame(void);

/**
 * flight_loop_step — execute one 10ms tick.
 * Returns CommandFrame* (same as flight_loop_get_command_frame()).
 */
CommandFrame *flight_loop_step(void);

/**
 * flight_loop_run — blocking real-time loop (standalone mode).
 * Runs n_ticks at 100 Hz using platform sleep.
 */
void flight_loop_run(uint64_t n_ticks);

/**
 * flight_loop_stop — signal flight_loop_run() to exit.
 */
void flight_loop_stop(void);

/**
 * flight_loop_reset — zero all state (for test sequences).
 */
void flight_loop_reset(void);

/**
 * flight_loop_get_thekf_state — copy EKF state for Python comparison.
 * x_out[6], P_out[36] (row-major).
 */
void flight_loop_get_thekf_state(double x_out[6], double P_out[36]);

#ifdef __cplusplus
}
#endif

#endif /* FLIGHT_LOOP_H */