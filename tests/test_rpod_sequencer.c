#include <math.h>
#include <stdio.h>
#include <string.h>
#include "rpod_sequencer.h"
#include "sim_config.h"

#define CHECK(cond, msg) do { \
    printf("  %s - %s\n", (cond) ? "PASS" : "FAIL", msg); \
    if (!(cond)) failures++; \
} while (0)

static int failures = 0;

static double norm3(const double v[3]) {
    return sqrt(v[0]*v[0] + v[1]*v[1] + v[2]*v[2]);
}

static void default_input(RPODSeqInput *in) {
    memset(in, 0, sizeof(*in));
    in->n_chief = sqrt(CFG_MU_GEO / pow(CFG_CHIEF_A_KM * 1000.0, 3.0));
    in->accel_max = CFG_DEP_THRUST_N / CFG_DEP_MASS_KG;
    in->mu = CFG_MU_GEO;
    in->chief_pos_eci[0] = CFG_CHIEF_A_KM * 1000.0;
    in->chief_vel_eci[1] = sqrt(CFG_MU_GEO / (CFG_CHIEF_A_KM * 1000.0));
    in->chief_eci_valid = 1;
}

int main(void) {
    printf("=== RPOD Sequencer C Verification ===\n");
    RPODSequencer seq;
    RPODSeqInput in;
    RPODSeqOutput out;

    RPODSeq_init(&seq);
    CHECK(seq.mode == RPOD_SEQ_FORMATION_HOLD, "init starts in FORMATION_HOLD");

    default_input(&in);
    in.nav_lvlh[1] = -400.0;
    RPODSeq_start_rendezvous(&seq, 100.0, 400.0);
    CHECK(seq.mode == RPOD_SEQ_PROX_OPS, "start inside far field goes direct to PROX_OPS");
    RPODSeq_step(&seq, &in, &out);
    CHECK(out.mode == RPOD_SEQ_PROX_OPS, "step remains in PROX_OPS");
    CHECK(norm3(out.accel_lvlh) > 0.0, "PROX_OPS outputs continuous acceleration");

    RPODSeq_init(&seq);
    default_input(&in);
    in.t_s = 200.0;
    in.nav_lvlh[1] = -1000.0;
    in.chief_eci_valid = 0;
    RPODSeq_start_rendezvous(&seq, in.t_s, 1000.0);
    RPODSeq_step(&seq, &in, &out);
    CHECK(seq.mode == RPOD_SEQ_LAMBERT, "start outside far field enters LAMBERT");
    CHECK(!out.has_impulse, "without chief ECI state, Lambert planner does not fire");
    CHECK(norm3(out.accel_lvlh) < 1e-12,
          "missing chief ECI falls back to formation hold and is quiet at standoff");

    RPODSeq_init(&seq);
    default_input(&in);
    in.t_s = 300.0;
    in.nav_lvlh[1] = -1000.0;
    RPODSeq_start_rendezvous(&seq, in.t_s, 1000.0);
    RPODSeq_step(&seq, &in, &out);
    CHECK(out.mode == RPOD_SEQ_LAMBERT, "Lambert planner remains in LAMBERT after burn-1");
    CHECK(out.has_impulse || norm3(out.accel_lvlh) > 0.0,
          "Lambert step either plans a burn or safely holds formation");

    printf("=== %s (%d failures) ===\n", failures ? "FAIL" : "ALL PASS", failures);
    return failures ? 1 : 0;
}
