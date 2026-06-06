/*
 * rpod_sequencer.h - High-level RPOD phase sequencer.
 *
 * This ports the hardware-relevant state machine from
 * control/lambert_controller.py: FORMATION_HOLD -> LAMBERT -> PROX_OPS.
 * PROX_OPS/TERMINAL/SOFT_CAPTURE inner loops remain in rpod_ctrl.c.
 */

#ifndef RPOD_SEQUENCER_H
#define RPOD_SEQUENCER_H

#include "rpod_ctrl.h"

typedef enum {
    RPOD_SEQ_FORMATION_HOLD = 4,
    RPOD_SEQ_LAMBERT        = 5,
    RPOD_SEQ_PROX_OPS       = 10
} RPODSeqMode;

typedef struct {
    double nav_lvlh[6];
    double chief_pos_eci[3];
    double chief_vel_eci[3];
    double t_s;
    double n_chief;
    double accel_max;
    double mu;
    int chief_eci_valid;
} RPODSeqInput;

typedef struct {
    double accel_lvlh[3];
    double impulse_dv_lvlh[3];
    int has_impulse;
    RPODSeqMode mode;
    int lambert_active;
    int plan_ok;
} RPODSeqOutput;

typedef struct {
    RPODSeqMode mode;
    double mode_entry_t_s;
    double lambert_burn2_t_s;
    double lambert_dv2_lvlh[3];
    double last_plan_t_s;
    int lambert_active;
    int replan_count;
} RPODSequencer;

void RPODSeq_init(RPODSequencer *seq);
void RPODSeq_start_rendezvous(RPODSequencer *seq, double t_s, double range_m);
void RPODSeq_step(RPODSequencer *seq,
                  const RPODSeqInput *in,
                  RPODSeqOutput *out);

#endif /* RPOD_SEQUENCER_H */
