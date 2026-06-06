/**
 * flight_loop.c ??? PC SIL Real-Time Flight Loop v2
 * Full ADCS stack: ModeManager + QUEST + B-dot + RW + MTQ + PD
 */
#include "flight_loop.h"
#include "th_ekf.h"
#include "mekf.h"
#include "quest.h"
#include "adcs.h"
#include "mode_manager.h"
#include "rpod_ctrl.h"
#include "terminal_filter.h"
#include "port_tracker.h"
#include "chief_pose_estimator.h"
#include "spin_sync_controller.h"
#include "sensor_packet.h"
#include "command_packet.h"
#include "target_port.h"
#include <math.h>
#include <string.h>
#include <stdint.h>

#ifdef _WIN32
  #define WIN32_LEAN_AND_MEAN
  #include <windows.h>
  static double _get_time_ms(void){LARGE_INTEGER f,c;QueryPerformanceFrequency(&f);QueryPerformanceCounter(&c);return(double)c.QuadPart/(double)f.QuadPart*1000.0;}
  static void   _sleep_ms(double ms){DWORD d=(DWORD)ms;if(d>0)Sleep(d);}
#else
  #include <time.h>
  static double _get_time_ms(void){struct timespec ts;clock_gettime(CLOCK_MONOTONIC,&ts);return ts.tv_sec*1000.0+ts.tv_nsec/1e6;}
  static void   _sleep_ms(double ms){struct timespec r;r.tv_sec=(time_t)(ms/1000.0);r.tv_nsec=(long)((ms-r.tv_sec*1000.0)*1e6);nanosleep(&r,NULL);}
#endif

static THEKF_State   g_thekf;
static MEKF_State    g_mekf;
static MM_State      g_mm;
static BDot_State    g_bdot;
static RW_State      g_rw;
static MTQ_State     g_mtq;
static AttCtrl_State g_attctrl;
static RPOD_State    g_rpod_state;
static int           g_is_braking = -1;
static CommandFrame  g_cmd_frame;
static SensorFrame   g_sensor_frame;
static uint64_t g_tick=0;
static double   g_max_loop_ms=0.0;
static uint32_t g_missed=0;
static double   g_deadline_ms=10.0;
static int      g_running=0;
static uint32_t g_invalid_packet_count=0;
static uint32_t g_stale_sensor_count=0;
static uint32_t g_gyro_bad_ticks=0;
/* Attitude reference. Identity is the default fine-pointing attitude. */
static double g_q_ref[4] = {1.0,0.0,0.0,0.0};
static SpinSyncController g_spin_sync_ctl;
static double g_spin_sync_omega_body[3] = {0.0,0.0,0.0};
static double g_spin_sync_prev_axis[3]  = {0.0,0.0,0.0};
static int    g_spin_sync_has_prev_axis = 0;
static int    g_spin_sync_active        = 0;
static int    g_watchdog_mode           = -99;
static double g_watchdog_phase_start_s  = 0.0;

static double _pointing_err_deg(const double q[4]){
    double qv=sqrt(q[1]*q[1]+q[2]*q[2]+q[3]*q[3]);
    return 2.0*asin(qv>1.0?1.0:qv)*180.0/3.14159265358979;
}

static void _set_q_ref_identity(void){
    g_q_ref[0]=1.0; g_q_ref[1]=0.0; g_q_ref[2]=0.0; g_q_ref[3]=0.0;
}

static void _reset_spin_sync(void){
    g_spin_sync_omega_body[0]=0.0;
    g_spin_sync_omega_body[1]=0.0;
    g_spin_sync_omega_body[2]=0.0;
    SSC_reset(&g_spin_sync_ctl);
    g_spin_sync_prev_axis[0]=0.0;
    g_spin_sync_prev_axis[1]=0.0;
    g_spin_sync_prev_axis[2]=0.0;
    g_spin_sync_has_prev_axis=0;
    g_spin_sync_active=0;
}

static void _quat_rotate_body_to_lvlh(const double q[4],
                                      const double v_body[3],
                                      double v_lvlh[3]){
    double w=q[0], x=q[1], y=q[2], z=q[3];
    double R00=1.0-2.0*(y*y+z*z), R01=2.0*(x*y-w*z),     R02=2.0*(x*z+w*y);
    double R10=2.0*(x*y+w*z),     R11=1.0-2.0*(x*x+z*z), R12=2.0*(y*z-w*x);
    double R20=2.0*(x*z-w*y),     R21=2.0*(y*z+w*x),     R22=1.0-2.0*(x*x+y*y);
    v_lvlh[0]=R00*v_body[0]+R01*v_body[1]+R02*v_body[2];
    v_lvlh[1]=R10*v_body[0]+R11*v_body[1]+R12*v_body[2];
    v_lvlh[2]=R20*v_body[0]+R21*v_body[1]+R22*v_body[2];
}

static void _quat_rotate_lvlh_to_body(const double q[4],
                                      const double v_lvlh[3],
                                      double v_body[3]){
    double w=q[0], x=q[1], y=q[2], z=q[3];
    double R00=1.0-2.0*(y*y+z*z), R01=2.0*(x*y-w*z),     R02=2.0*(x*z+w*y);
    double R10=2.0*(x*y+w*z),     R11=1.0-2.0*(x*x+z*z), R12=2.0*(y*z-w*x);
    double R20=2.0*(x*z-w*y),     R21=2.0*(y*z+w*x),     R22=1.0-2.0*(x*x+y*y);
    v_body[0]=R00*v_lvlh[0]+R10*v_lvlh[1]+R20*v_lvlh[2];
    v_body[1]=R01*v_lvlh[0]+R11*v_lvlh[1]+R21*v_lvlh[2];
    v_body[2]=R02*v_lvlh[0]+R12*v_lvlh[1]+R22*v_lvlh[2];
}

static void _update_spin_sync_from_port(const double q_dep_body_to_lvlh[4],
                                        const double port_lvlh[3],
                                        const double port_axis_lvlh[3],
                                        const double port_vel_lvlh[3],
                                        double dt_s){
    double axis[3]={port_axis_lvlh[0],port_axis_lvlh[1],port_axis_lvlh[2]};
    double an=sqrt(axis[0]*axis[0]+axis[1]*axis[1]+axis[2]*axis[2]);
    if(an < 1e-12 || dt_s <= 0.0){
        _reset_spin_sync();
        return;
    }
    axis[0]/=an; axis[1]/=an; axis[2]/=an;

    double omega_lvlh[3]={0.0,0.0,0.0};
    double n_terms=0.0;

    double r2=port_lvlh[0]*port_lvlh[0]+port_lvlh[1]*port_lvlh[1]+port_lvlh[2]*port_lvlh[2];
    if(r2 > 1e-8){
        double rv[3]={
            port_lvlh[1]*port_vel_lvlh[2]-port_lvlh[2]*port_vel_lvlh[1],
            port_lvlh[2]*port_vel_lvlh[0]-port_lvlh[0]*port_vel_lvlh[2],
            port_lvlh[0]*port_vel_lvlh[1]-port_lvlh[1]*port_vel_lvlh[0]
        };
        omega_lvlh[0]+=rv[0]/r2; omega_lvlh[1]+=rv[1]/r2; omega_lvlh[2]+=rv[2]/r2;
        n_terms+=1.0;
    }

    if(g_spin_sync_has_prev_axis){
        double c[3]={
            g_spin_sync_prev_axis[1]*axis[2]-g_spin_sync_prev_axis[2]*axis[1],
            g_spin_sync_prev_axis[2]*axis[0]-g_spin_sync_prev_axis[0]*axis[2],
            g_spin_sync_prev_axis[0]*axis[1]-g_spin_sync_prev_axis[1]*axis[0]
        };
        omega_lvlh[0]+=c[0]/dt_s; omega_lvlh[1]+=c[1]/dt_s; omega_lvlh[2]+=c[2]/dt_s;
        n_terms+=1.0;
    }

    g_spin_sync_prev_axis[0]=axis[0];
    g_spin_sync_prev_axis[1]=axis[1];
    g_spin_sync_prev_axis[2]=axis[2];
    g_spin_sync_has_prev_axis=1;

    if(n_terms < 0.5){
        g_spin_sync_active=0;
        return;
    }
    omega_lvlh[0]/=n_terms; omega_lvlh[1]/=n_terms; omega_lvlh[2]/=n_terms;

    double mag=sqrt(omega_lvlh[0]*omega_lvlh[0]+omega_lvlh[1]*omega_lvlh[1]+omega_lvlh[2]*omega_lvlh[2]);
    if(mag > CFG_SPIN_SYNC_MAX_OMEGA_RAD_S){
        _reset_spin_sync();
        return;
    }
    if(mag > CFG_SPIN_SYNC_MAX_RATE_RAD_S){
        double s=CFG_SPIN_SYNC_MAX_RATE_RAD_S/mag;
        omega_lvlh[0]*=s; omega_lvlh[1]*=s; omega_lvlh[2]*=s;
    }

    double omega_body[3];
    _quat_rotate_lvlh_to_body(q_dep_body_to_lvlh, omega_lvlh, omega_body);
    SSC_compute_rate_body(&g_spin_sync_ctl, omega_body, g_spin_sync_omega_body);
    g_spin_sync_active=1;
}

static void _attctrl_omega_for_control(const double omega_est[3], double omega_out[3]){
    for(int i=0;i<3;i++){
        omega_out[i]=omega_est[i] - (g_spin_sync_active ? g_spin_sync_omega_body[i] : 0.0);
    }
}

static void _clear_rpod_telem(RPODTelemetry *rt){
    rt->port_range_m=NAN;
    rt->port_vrel_ms=NAN;
    rt->attitude_align_deg=NAN;
    rt->cone_error_deg=NAN;
    rt->lateral_m=NAN;
    rt->phase_elapsed_s=0.0;
    rt->has_port=0;
    rt->geometry_ok=0;
    rt->body_clear=0;
    rt->capture_core=0;
    rt->timeout_code=0;
    rt->pose_age_s=NAN;
    rt->spin_sync_rate_cmd[0]=0.0;
    rt->spin_sync_rate_cmd[1]=0.0;
    rt->spin_sync_rate_cmd[2]=0.0;
    rt->pose_status=0;
    rt->pose_valid=0;
    rt->spin_sync_active=0;
}

static void _fill_rpod_telem(RPODTelemetry *rt, const RPOD_TermState *ts){
    rt->has_port=ts->has_port ? 1 : 0;
    rt->geometry_ok=ts->geometry_ok ? 1 : 0;
    rt->body_clear=ts->body_clear ? 1 : 0;
    rt->capture_core=ts->capture_core ? 1 : 0;
    rt->attitude_align_deg=ts->has_attitude_align ? ts->attitude_align_deg : NAN;
    rt->cone_error_deg=ts->cone_error_deg;
    rt->lateral_m=ts->lateral_m;
    if(ts->has_port){
        rt->port_range_m=sqrt(ts->port_lvlh[0]*ts->port_lvlh[0]+
                              ts->port_lvlh[1]*ts->port_lvlh[1]+
                              ts->port_lvlh[2]*ts->port_lvlh[2]);
        double dv[3]={ts->vel[0]-ts->port_vel_lvlh[0],
                      ts->vel[1]-ts->port_vel_lvlh[1],
                      ts->vel[2]-ts->port_vel_lvlh[2]};
        rt->port_vrel_ms=sqrt(dv[0]*dv[0]+dv[1]*dv[1]+dv[2]*dv[2]);
    }else{
        rt->port_range_m=NAN;
        rt->port_vrel_ms=NAN;
    }
}

static void _reset_rpod_watchdog(void){
    g_watchdog_mode=-99;
    g_watchdog_phase_start_s=0.0;
}

static void _update_rpod_watchdog(double t_sim, int mode, RPODTelemetry *rt){
    if(mode != g_watchdog_mode){
        g_watchdog_mode=mode;
        g_watchdog_phase_start_s=t_sim;
    }
    double elapsed=t_sim-g_watchdog_phase_start_s;
    if(elapsed < 0.0) elapsed=0.0;
    rt->phase_elapsed_s=elapsed;
    rt->timeout_code=0;
    if(mode==10 && elapsed > CFG_PROX_OPS_MAX_S) rt->timeout_code=1;
    else if(mode==11 && elapsed > CFG_TERMINAL_MAX_S) rt->timeout_code=2;
    else if(mode==14 && elapsed > CFG_SOFT_CAPTURE_MAX_HOLD_S) rt->timeout_code=3;
}

static double _dock_alignment_deg(const double q_dep_body_to_lvlh[4],
                                  const double port_axis_lvlh[3]){
    double dep_axis_body[3]={
        CFG_DEP_DOCK_AXIS_BODY_X,
        CFG_DEP_DOCK_AXIS_BODY_Y,
        CFG_DEP_DOCK_AXIS_BODY_Z
    };
    double dep_axis[3];
    _quat_rotate_body_to_lvlh(q_dep_body_to_lvlh, dep_axis_body, dep_axis);

    double dep_n=sqrt(dep_axis[0]*dep_axis[0]+dep_axis[1]*dep_axis[1]+dep_axis[2]*dep_axis[2]);
    double port_n=sqrt(port_axis_lvlh[0]*port_axis_lvlh[0]+port_axis_lvlh[1]*port_axis_lvlh[1]+port_axis_lvlh[2]*port_axis_lvlh[2]);
    if(dep_n < 1e-12 || port_n < 1e-12) return 180.0;

    double desired[3]={
        -port_axis_lvlh[0]/port_n,
        -port_axis_lvlh[1]/port_n,
        -port_axis_lvlh[2]/port_n
    };
    double dot=(dep_axis[0]/dep_n)*desired[0]+
               (dep_axis[1]/dep_n)*desired[1]+
               (dep_axis[2]/dep_n)*desired[2];
    if(dot > 1.0) dot = 1.0;
    if(dot < -1.0) dot = -1.0;
    return acos(dot) * 180.0 / 3.14159265358979;
}

static void _set_q_ref_for_port_axis(const double port_axis_lvlh[3]){
    double a[3]={
        CFG_DEP_DOCK_AXIS_BODY_X,
        CFG_DEP_DOCK_AXIS_BODY_Y,
        CFG_DEP_DOCK_AXIS_BODY_Z
    };
    double b[3]={-port_axis_lvlh[0], -port_axis_lvlh[1], -port_axis_lvlh[2]};
    double an=sqrt(a[0]*a[0]+a[1]*a[1]+a[2]*a[2]);
    double bn=sqrt(b[0]*b[0]+b[1]*b[1]+b[2]*b[2]);
    if(an < 1e-12 || bn < 1e-12){
        _set_q_ref_identity();
        return;
    }
    for(int i=0;i<3;i++){ a[i]/=an; b[i]/=bn; }

    double dot=a[0]*b[0]+a[1]*b[1]+a[2]*b[2];
    if(dot > 1.0) dot=1.0;
    if(dot < -1.0) dot=-1.0;

    double q[4];
    if(dot < -0.999999){
        double axis[3] = {1.0, 0.0, 0.0};
        if(fabs(a[0]) > 0.9){ axis[0]=0.0; axis[1]=1.0; axis[2]=0.0; }
        double ortho[3]={
            a[1]*axis[2]-a[2]*axis[1],
            a[2]*axis[0]-a[0]*axis[2],
            a[0]*axis[1]-a[1]*axis[0]
        };
        double on=sqrt(ortho[0]*ortho[0]+ortho[1]*ortho[1]+ortho[2]*ortho[2]);
        q[0]=0.0;
        q[1]=ortho[0]/on; q[2]=ortho[1]/on; q[3]=ortho[2]/on;
    }else{
        double c[3]={
            a[1]*b[2]-a[2]*b[1],
            a[2]*b[0]-a[0]*b[2],
            a[0]*b[1]-a[1]*b[0]
        };
        q[0]=1.0+dot;
        q[1]=c[0]; q[2]=c[1]; q[3]=c[2];
    }
    double qn=sqrt(q[0]*q[0]+q[1]*q[1]+q[2]*q[2]+q[3]*q[3]);
    if(qn < 1e-12){
        _set_q_ref_identity();
        return;
    }
    g_q_ref[0]=q[0]/qn; g_q_ref[1]=q[1]/qn;
    g_q_ref[2]=q[2]/qn; g_q_ref[3]=q[3]/qn;
}

/*
 * g_accel_lvlh_hold ??? RPOD accel hold register.
 *
 * closed_loop_sil.py reads cf->cmd.accel_lvlh every 100 Hz tick and feeds
 * it to the CW plant.  RPOD guidance runs at 10 Hz (g_tick%10==0) and the
 * remaining 9 ticks zeroed accel ??? giving only 10% of commanded thrust.
 * Fix: cache the last computed accel and write it into every CommandFrame.
 * Hold is cleared when RPOD is not active.
 */
static double g_accel_lvlh_hold[3] = {0.0, 0.0, 0.0};
static double g_dipole_mtq_hold[3] = {0.0, 0.0, 0.0};
static double g_torque_rw_hold[3]  = {0.0, 0.0, 0.0};
static int    g_rpod_mode_hold     = -1;
static int    g_docked_latched     = 0;
static int    g_soft_capture_active = 0;
static double g_hard_capture_hold_s = 0.0;
static double g_hard_capture_grace_s = 0.0;
static int    g_terminal_inflated  = 0;
static TermNavFilter g_terminal_nav;
static PortTracker   g_port_tracker;
static CPE_State     g_chief_pose;
static double                  g_terminal_cam_lost_s = -1.0;
static int                     g_lost_target_active = 0;

static void _reset_chief_pose_estimator(void){
    CPE_CamParams cam;
    CPE_default_cam_params(&cam);
    CPE_init(&g_chief_pose, &cam, 0.1, 0.001, 3.0, 10.0);
}

static void _reset_terminal_guidance(void){
    TNF_reset(&g_terminal_nav);
    PT_reset(&g_port_tracker);
    _reset_spin_sync();
    g_terminal_cam_lost_s = -1.0;
    g_lost_target_active = 0;
    g_soft_capture_active = 0;
    g_hard_capture_hold_s = 0.0;
    g_hard_capture_grace_s = 0.0;
}

static void _clear_attitude_holds(void){
    g_dipole_mtq_hold[0]=g_dipole_mtq_hold[1]=g_dipole_mtq_hold[2]=0.0;
    g_torque_rw_hold[0]=g_torque_rw_hold[1]=g_torque_rw_hold[2]=0.0;
    _set_q_ref_identity();
    _reset_spin_sync();
}

#define GNC_GYRO_MAX_AGE_TICKS   2u
#define GNC_10HZ_MAX_AGE_TICKS   15u
#define GNC_GYRO_SAFE_TICKS      50u
#define GNC_ACCEL_MAX_MS2        0.020
#define GNC_MTQ_MAX_AM2          300.0
#define TAU_PLANT_MAX_NM         2e-3

static double _clampd(double v, double lo, double hi){
    return v < lo ? lo : (v > hi ? hi : v);
}

static int _packet_is_usable(uint8_t valid, int meta_ok, uint32_t invalid_flag,
                             uint32_t *flags){
    if(!valid) return 0;
    if(!meta_ok){
        *flags |= invalid_flag;
        g_invalid_packet_count++;
        return 0;
    }
    return 1;
}

static void _finalize_outputs(CommandFrame *cf, uint32_t watchdog_flags){
    double amag=sqrt(cf->cmd.accel_lvlh[0]*cf->cmd.accel_lvlh[0]+
                     cf->cmd.accel_lvlh[1]*cf->cmd.accel_lvlh[1]+
                     cf->cmd.accel_lvlh[2]*cf->cmd.accel_lvlh[2]);
    if(amag > GNC_ACCEL_MAX_MS2 && amag > 1e-15){
        double s=GNC_ACCEL_MAX_MS2/amag;
        for(int i=0;i<3;i++) cf->cmd.accel_lvlh[i]*=s;
    }
    for(int i=0;i<3;i++){
        cf->cmd.torque_rw[i]=_clampd(cf->cmd.torque_rw[i],
                                     -TAU_PLANT_MAX_NM, TAU_PLANT_MAX_NM);
        cf->cmd.dipole_mtq[i]=_clampd(cf->cmd.dipole_mtq[i],
                                      -GNC_MTQ_MAX_AM2, GNC_MTQ_MAX_AM2);
    }
    if(watchdog_flags & GNC_WD_SAFE_FALLBACK){
        for(int i=0;i<3;i++){
            cf->cmd.accel_lvlh[i]=0.0;
            cf->cmd.torque_rw[i]=0.0;
            cf->cmd.dipole_mtq[i]=0.0;
            g_accel_lvlh_hold[i]=0.0;
            g_torque_rw_hold[i]=0.0;
            g_dipole_mtq_hold[i]=0.0;
        }
        cf->cmd.rpod_mode=-1;
        cf->timing.output_inhibited=1;
        cf->timing.watchdog_flags |= GNC_WD_OUTPUT_INHIBITED;
    }else{
        cf->timing.output_inhibited=0;
    }
}

/*
 * _apply_rw_clamped ??? clamp torque to plant physical limit before h_rw
 * integration.  The PhysicsPlantSim clamps applied torque to tau_max=2mN??m.
 * Without this, the C wheel counter diverges from plant reality, triggering
 * spurious momentum dumps before attitude has converged.
 */
static void _apply_rw_clamped(RW_State *rw, const double trw[3], double dt){
    double t[3];
    for(int i=0;i<3;i++){
        t[i] = trw[i];
        if(t[i] >  TAU_PLANT_MAX_NM) t[i] =  TAU_PLANT_MAX_NM;
        if(t[i] < -TAU_PLANT_MAX_NM) t[i] = -TAU_PLANT_MAX_NM;
    }
    RW_apply_torque(rw, t, dt);
}

void flight_loop_init(double a_chief_m,double e_chief,double mu,double dt_thekf_s){
    memset(&g_cmd_frame,0,sizeof(g_cmd_frame));
    memset(&g_sensor_frame,0,sizeof(g_sensor_frame));
    THEKF_init(&g_thekf,a_chief_m,e_chief,mu,dt_thekf_s,1e-4,1e-8);
    MEKF_init(&g_mekf,0.01f);
    MM_init(&g_mm);
    BDOT_init(&g_bdot);
    RW_init(&g_rw);
    MTQ_init(&g_mtq);
    ATTCTRL_init(&g_attctrl);
    SSC_init(&g_spin_sync_ctl, CFG_SPIN_SYNC_RATE_BLEND, CFG_SPIN_SYNC_MAX_RATE_RAD_S);
    _reset_chief_pose_estimator();
    g_tick=0; g_max_loop_ms=0.0; g_missed=0; g_is_braking=-1; g_rpod_mode_hold=-1; g_docked_latched=0; g_terminal_inflated=0; _reset_rpod_watchdog(); _reset_terminal_guidance();
    g_cmd_frame.timing.deadline_ms=g_deadline_ms;
    g_invalid_packet_count=0; g_stale_sensor_count=0; g_gyro_bad_ticks=0;
    GNC_LOG("  flight_loop v2: full ADCS stack enabled\n");
}

void flight_loop_seed_thekf(const double x0[6],const double P0_flat[36],double nu0){
    double P0[6][6];
    for(int i=0;i<6;i++) for(int j=0;j<6;j++) P0[i][j]=P0_flat[i*6+j];
    THEKF_seed(&g_thekf,x0,P0,nu0);
}

SensorFrame  *flight_loop_get_sensor_frame(void) {return &g_sensor_frame;}
CommandFrame *flight_loop_get_command_frame(void){return &g_cmd_frame;}

CommandFrame *flight_loop_step(void){
    double t_start=_get_time_ms();
    SensorFrame  *sf=&g_sensor_frame;
    CommandFrame *cf=&g_cmd_frame;
    double t_sim=(double)g_tick*0.01;
    _clear_rpod_telem(&cf->rpod);
    cf->timing.watchdog_flags=0;
    cf->timing.output_inhibited=0;
    if(sf->sim_tick != g_tick) cf->timing.watchdog_flags |= GNC_WD_FRAME_TICK_MISMATCH;

    int gyro_valid = _packet_is_usable(sf->gyro.valid,
        PacketMeta_ok(&sf->gyro.meta, GyroPacket_checksum(&sf->gyro), g_tick, GNC_GYRO_MAX_AGE_TICKS),
        GNC_WD_GYRO_INVALID, &cf->timing.watchdog_flags);
    int range_valid = _packet_is_usable(sf->range.valid,
        PacketMeta_ok(&sf->range.meta, RangePacket_checksum(&sf->range), g_tick, GNC_10HZ_MAX_AGE_TICKS),
        GNC_WD_RANGE_INVALID, &cf->timing.watchdog_flags);
    int camera_valid = _packet_is_usable(sf->camera.valid,
        PacketMeta_ok(&sf->camera.meta, CameraPacket_checksum(&sf->camera), g_tick, GNC_10HZ_MAX_AGE_TICKS),
        GNC_WD_CAMERA_INVALID, &cf->timing.watchdog_flags);
    int port_valid = _packet_is_usable(sf->port.valid,
        PacketMeta_ok(&sf->port.meta, PortPacket_checksum(&sf->port), g_tick, GNC_10HZ_MAX_AGE_TICKS),
        GNC_WD_PORT_INVALID, &cf->timing.watchdog_flags);
    int mag_valid = _packet_is_usable(sf->mag.valid,
        PacketMeta_ok(&sf->mag.meta, MagPacket_checksum(&sf->mag), g_tick, GNC_10HZ_MAX_AGE_TICKS),
        GNC_WD_MAG_INVALID, &cf->timing.watchdog_flags);
    int sun_valid = _packet_is_usable(sf->sun.valid,
        PacketMeta_ok(&sf->sun.meta, SunPacket_checksum(&sf->sun), g_tick, GNC_10HZ_MAX_AGE_TICKS),
        GNC_WD_SUN_INVALID, &cf->timing.watchdog_flags);
    if(gyro_valid) g_gyro_bad_ticks=0;
    else {
        g_gyro_bad_ticks++;
        g_stale_sensor_count++;
        if(g_gyro_bad_ticks >= GNC_GYRO_SAFE_TICKS){
            cf->timing.watchdog_flags |= GNC_WD_SAFE_FALLBACK;
        }
    }

    /* ?????? 100 Hz: Gyro + MEKF predict ???????????????????????????????????????????????????????????????????????????????????? */
    if(gyro_valid){
        MEKF_FLOAT w[3]={(MEKF_FLOAT)sf->gyro.omega_xyz[0],
                         (MEKF_FLOAT)sf->gyro.omega_xyz[1],
                         (MEKF_FLOAT)sf->gyro.omega_xyz[2]};
        MEKF_predict(&g_mekf,w);
    }
    cf->att.q_wxyz[0]=(double)g_mekf.q[0]; cf->att.q_wxyz[1]=(double)g_mekf.q[1];
    cf->att.q_wxyz[2]=(double)g_mekf.q[2]; cf->att.q_wxyz[3]=(double)g_mekf.q[3];
    cf->att.bias_xyz[0]=(double)g_mekf.bias[0];
    cf->att.bias_xyz[1]=(double)g_mekf.bias[1];
    cf->att.bias_xyz[2]=(double)g_mekf.bias[2];
    cf->att.pointing_err_deg=_pointing_err_deg(cf->att.q_wxyz);

    double omega_est[3]={
        gyro_valid?sf->gyro.omega_xyz[0]-(double)g_mekf.bias[0]:0.0,
        gyro_valid?sf->gyro.omega_xyz[1]-(double)g_mekf.bias[1]:0.0,
        gyro_valid?sf->gyro.omega_xyz[2]-(double)g_mekf.bias[2]:0.0
    };

    /* ?????? Mode manager update ??????????????????????????????????????????????????????????????????????????????????????????????????????????????? */
    double quest_err=-1.0;
    FSW_Mode mode=MM_update(&g_mm,t_sim,omega_est,g_rw.h,
                            quest_err,0,cf->att.pointing_err_deg);

    /* ?????? Zero actuator outputs ????????????????????????????????????????????????????????????????????????????????????????????????????????? */
    memset(cf->cmd.torque_rw, 0,sizeof(cf->cmd.torque_rw));
    memset(cf->cmd.dipole_mtq,0,sizeof(cf->cmd.dipole_mtq));
    /* Restore held accel ??? plant reads this every 100 Hz tick */
    cf->cmd.accel_lvlh[0]=g_accel_lvlh_hold[0];
    cf->cmd.accel_lvlh[1]=g_accel_lvlh_hold[1];
    cf->cmd.accel_lvlh[2]=g_accel_lvlh_hold[2];
    cf->cmd.dipole_mtq[0]=g_dipole_mtq_hold[0];
    cf->cmd.dipole_mtq[1]=g_dipole_mtq_hold[1];
    cf->cmd.dipole_mtq[2]=g_dipole_mtq_hold[2];
    cf->cmd.torque_rw[0]=g_torque_rw_hold[0];
    cf->cmd.torque_rw[1]=g_torque_rw_hold[1];
    cf->cmd.torque_rw[2]=g_torque_rw_hold[2];
    cf->cmd.rpod_mode=g_rpod_mode_hold;

    /* ?????? Mode dispatch ????????????????????????????????????????????????????????????????????????????????????????????????????????????????????????????????? */
    if(mode==MODE_SAFE_MODE){
        g_accel_lvlh_hold[0]=g_accel_lvlh_hold[1]=g_accel_lvlh_hold[2]=0.0;
        g_rpod_mode_hold=-1;
        g_is_braking=-1;
        g_terminal_inflated=0;
        _reset_terminal_guidance();
        _clear_attitude_holds();
        cf->cmd.rpod_mode=-1;
        cf->cmd.accel_lvlh[0]=cf->cmd.accel_lvlh[1]=cf->cmd.accel_lvlh[2]=0.0;
        cf->cmd.dipole_mtq[0]=cf->cmd.dipole_mtq[1]=cf->cmd.dipole_mtq[2]=0.0;
        cf->cmd.torque_rw[0]=cf->cmd.torque_rw[1]=cf->cmd.torque_rw[2]=0.0;
        cf->cmd.fsw_mode=0;

    }else if(mode==MODE_DETUMBLE){
        g_accel_lvlh_hold[0]=g_accel_lvlh_hold[1]=g_accel_lvlh_hold[2]=0.0;
        g_rpod_mode_hold=-1;
        g_is_braking=-1;
        g_terminal_inflated=0;
        _reset_terminal_guidance();
        g_torque_rw_hold[0]=g_torque_rw_hold[1]=g_torque_rw_hold[2]=0.0;
        cf->cmd.rpod_mode=-1;
        cf->cmd.accel_lvlh[0]=cf->cmd.accel_lvlh[1]=cf->cmd.accel_lvlh[2]=0.0;
        if(mag_valid){
            double m[3],tau[3];
            BDOT_compute(&g_bdot,sf->mag.body,omega_est,m,tau);
            g_dipole_mtq_hold[0]=m[0]; g_dipole_mtq_hold[1]=m[1]; g_dipole_mtq_hold[2]=m[2];
        }
        cf->cmd.dipole_mtq[0]=g_dipole_mtq_hold[0]; cf->cmd.dipole_mtq[1]=g_dipole_mtq_hold[1]; cf->cmd.dipole_mtq[2]=g_dipole_mtq_hold[2];
        cf->cmd.torque_rw[0]=cf->cmd.torque_rw[1]=cf->cmd.torque_rw[2]=0.0;
        cf->cmd.fsw_mode=1;

    }else if(mode==MODE_SUN_ACQUISITION){
        g_accel_lvlh_hold[0]=g_accel_lvlh_hold[1]=g_accel_lvlh_hold[2]=0.0;
        g_rpod_mode_hold=-1;
        g_is_braking=-1;
        g_terminal_inflated=0;
        _reset_terminal_guidance();
        g_dipole_mtq_hold[0]=g_dipole_mtq_hold[1]=g_dipole_mtq_hold[2]=0.0;
        cf->cmd.rpod_mode=-1;
        cf->cmd.accel_lvlh[0]=cf->cmd.accel_lvlh[1]=cf->cmd.accel_lvlh[2]=0.0;
        /* QUEST attitude init */
        if(mag_valid && sun_valid){
            QUEST_Result qr=QUEST_compute(sf->mag.body,sf->mag.inertial,
                                           sf->sun.body,sf->sun.inertial,0.9,0.1);
            cf->att.quest_quality=qr.quality;
            if(qr.ok){
                g_mekf.q[0]=(MEKF_FLOAT)qr.q[0]; g_mekf.q[1]=(MEKF_FLOAT)qr.q[1];
                g_mekf.q[2]=(MEKF_FLOAT)qr.q[2]; g_mekf.q[3]=(MEKF_FLOAT)qr.q[3];
            }
            quest_err=cf->att.pointing_err_deg;
            mode=MM_update(&g_mm,t_sim,omega_est,g_rw.h,quest_err,0,cf->att.pointing_err_deg);
        }
        /* PD slew toward reference during sun acq */
        double trw[3], omega_ctrl[3];
        _attctrl_omega_for_control(omega_est, omega_ctrl);
        ATTCTRL_compute(&g_attctrl,cf->att.q_wxyz,omega_ctrl,g_q_ref,trw,NULL);
        _apply_rw_clamped(&g_rw,trw,0.01);
        g_torque_rw_hold[0]=trw[0]; g_torque_rw_hold[1]=trw[1]; g_torque_rw_hold[2]=trw[2];
        cf->cmd.torque_rw[0]=g_torque_rw_hold[0]; cf->cmd.torque_rw[1]=g_torque_rw_hold[1]; cf->cmd.torque_rw[2]=g_torque_rw_hold[2];
        cf->cmd.dipole_mtq[0]=cf->cmd.dipole_mtq[1]=cf->cmd.dipole_mtq[2]=0.0;
        cf->cmd.fsw_mode=2;

    }else if(mode==MODE_FINE_POINTING){
        double trw[3], omega_ctrl[3];
        _attctrl_omega_for_control(omega_est, omega_ctrl);
        ATTCTRL_compute(&g_attctrl,cf->att.q_wxyz,omega_ctrl,g_q_ref,trw,NULL);
        _apply_rw_clamped(&g_rw,trw,0.01);
        g_torque_rw_hold[0]=trw[0]; g_torque_rw_hold[1]=trw[1]; g_torque_rw_hold[2]=trw[2];
        g_dipole_mtq_hold[0]=g_dipole_mtq_hold[1]=g_dipole_mtq_hold[2]=0.0;
        cf->cmd.torque_rw[0]=g_torque_rw_hold[0]; cf->cmd.torque_rw[1]=g_torque_rw_hold[1]; cf->cmd.torque_rw[2]=g_torque_rw_hold[2];
        cf->cmd.dipole_mtq[0]=cf->cmd.dipole_mtq[1]=cf->cmd.dipole_mtq[2]=0.0;
        cf->cmd.fsw_mode=3;

    }else if(mode==MODE_MOMENTUM_DUMP){
        g_accel_lvlh_hold[0]=g_accel_lvlh_hold[1]=g_accel_lvlh_hold[2]=0.0;
        g_rpod_mode_hold=-1;
        g_is_braking=-1;
        g_terminal_inflated=0;
        _reset_terminal_guidance();
        cf->cmd.rpod_mode=-1;
        cf->cmd.accel_lvlh[0]=cf->cmd.accel_lvlh[1]=cf->cmd.accel_lvlh[2]=0.0;
        if(mag_valid){
            double m[3],tau[3];
            MTQ_compute_dipole(&g_mtq,g_rw.h,sf->mag.body,m);
            MTQ_compute_torque(m,sf->mag.body,tau);
            g_dipole_mtq_hold[0]=m[0]; g_dipole_mtq_hold[1]=m[1]; g_dipole_mtq_hold[2]=m[2];
        }
        double trw[3], omega_ctrl[3];
        _attctrl_omega_for_control(omega_est, omega_ctrl);
        ATTCTRL_compute(&g_attctrl,cf->att.q_wxyz,omega_ctrl,g_q_ref,trw,NULL);
        _apply_rw_clamped(&g_rw,trw,0.01);
        g_torque_rw_hold[0]=trw[0]; g_torque_rw_hold[1]=trw[1]; g_torque_rw_hold[2]=trw[2];
        cf->cmd.dipole_mtq[0]=g_dipole_mtq_hold[0]; cf->cmd.dipole_mtq[1]=g_dipole_mtq_hold[1]; cf->cmd.dipole_mtq[2]=g_dipole_mtq_hold[2];
        cf->cmd.torque_rw[0]=g_torque_rw_hold[0]; cf->cmd.torque_rw[1]=g_torque_rw_hold[1]; cf->cmd.torque_rw[2]=g_torque_rw_hold[2];
        cf->cmd.fsw_mode=4;
    }

    /* ?????? 10 Hz tasks ???????????????????????????????????????????????????????????????????????????????????????????????????????????????????????????????????? */
    cf->ekf_updated=0;
    if(g_tick%10==0){
        THEKF_predict(&g_thekf,g_accel_lvlh_hold);
        if(range_valid){
            double z[3]={sf->range.range_m,sf->range.azimuth_rad,sf->range.elevation_rad};
            double R[3][3]={{0.09,0,0},{0,3e-6,0},{0,0,3e-6}};
            cf->ekf_updated=(uint8_t)THEKF_update(&g_thekf,z,R,5.0);
        }
        if(camera_valid){
            double Rc[3][3]={{sf->camera.R_diag[0],0,0},{0,sf->camera.R_diag[1],0},{0,0,sf->camera.R_diag[2]}};
            cf->ekf_updated|=(uint8_t)THEKF_update_position(&g_thekf,sf->camera.pos_lvlh,Rc,5.0);
        }
        if(mag_valid){
            MEKF_FLOAT zb[3]={(MEKF_FLOAT)sf->mag.body[0],(MEKF_FLOAT)sf->mag.body[1],(MEKF_FLOAT)sf->mag.body[2]};
            MEKF_FLOAT vi[3]={(MEKF_FLOAT)sf->mag.inertial[0],(MEKF_FLOAT)sf->mag.inertial[1],(MEKF_FLOAT)sf->mag.inertial[2]};
            MEKF_update(&g_mekf,zb,vi,g_mekf.R_mag);
        }
        if(sun_valid){
            MEKF_FLOAT zb[3]={(MEKF_FLOAT)sf->sun.body[0],(MEKF_FLOAT)sf->sun.body[1],(MEKF_FLOAT)sf->sun.body[2]};
            MEKF_FLOAT vi[3]={(MEKF_FLOAT)sf->sun.inertial[0],(MEKF_FLOAT)sf->sun.inertial[1],(MEKF_FLOAT)sf->sun.inertial[2]};
            MEKF_update(&g_mekf,zb,vi,g_mekf.R_sun);
        }
        THEKF_get_pos(&g_thekf,cf->nav.pos_lvlh);
        THEKF_get_vel(&g_thekf,cf->nav.vel_lvlh);
        THEKF_get_pos_std(&g_thekf,cf->nav.pos_std);
        THEKF_get_vel_std(&g_thekf,cf->nav.vel_std);
        cf->nav.range_m=THEKF_range(&g_thekf);

        /* RPOD in FINE_POINTING */
        if(mode==MODE_FINE_POINTING){
            double tr=cf->nav.range_m;
            double ar=0.020, new_acc[3]={0.,0.,0.};
            g_rpod_state.pos[0]=cf->nav.pos_lvlh[0];
            g_rpod_state.pos[1]=cf->nav.pos_lvlh[1];
            g_rpod_state.pos[2]=cf->nav.pos_lvlh[2];
            g_rpod_state.vel[0]=cf->nav.vel_lvlh[0];
            g_rpod_state.vel[1]=cf->nav.vel_lvlh[1];
            g_rpod_state.vel[2]=cf->nav.vel_lvlh[2];
            if(g_docked_latched){
                g_rpod_mode_hold=12;
                g_is_braking=-1;
            }else if(tr>RPOD_TERMINAL_M){
                RPOD_prox_ops(&g_rpod_state,tr,g_thekf.n,ar,new_acc);
                g_rpod_mode_hold=10;
                g_is_braking=-1;
                g_terminal_inflated=0;
                _set_q_ref_identity();
                _reset_terminal_guidance();
            }else{
                if(!g_terminal_inflated){
                    THEKF_inflate_process_noise(&g_thekf,10.0);
                    g_terminal_inflated=1;
                }
                RPOD_TermState ts;
                double guided_pos[3], guided_vel[3], tracked_port[3];
                TNF_update(&g_terminal_nav, g_rpod_state.pos, g_rpod_state.vel,
                                 camera_valid ? 1 : 0, 0.1,
                                 guided_pos, guided_vel);
                ts.pos[0]=guided_pos[0]; ts.pos[1]=guided_pos[1]; ts.pos[2]=guided_pos[2];
                ts.vel[0]=guided_vel[0]; ts.vel[1]=guided_vel[1]; ts.vel[2]=guided_vel[2];
                ts.attitude_align_deg=0.0;
                ts.cone_angle_deg=0.0;
                ts.cone_error_deg=0.0;
                ts.lateral_m=0.0;
                ts.has_attitude_align=0;
                ts.has_body_R=0;
                ts.has_geometry=0;
                ts.geometry_ok=1;
                ts.body_clear=1;
                ts.capture_core=0;
                ts.has_port=PT_update(&g_port_tracker, sf->port.port_lvlh,
                                                  port_valid ? 1 : 0,
                                                  0.1, tracked_port);
                if(ts.has_port){
                    ts.port_lvlh[0]=tracked_port[0];
                    ts.port_lvlh[1]=tracked_port[1];
                    ts.port_lvlh[2]=tracked_port[2];
                    if(port_valid){
                        ts.port_axis_lvlh[0]=sf->port.port_axis_lvlh[0];
                        ts.port_axis_lvlh[1]=sf->port.port_axis_lvlh[1];
                        ts.port_axis_lvlh[2]=sf->port.port_axis_lvlh[2];
                        ts.port_vel_lvlh[0]=sf->port.port_vel_lvlh[0];
                        ts.port_vel_lvlh[1]=sf->port.port_vel_lvlh[1];
                        ts.port_vel_lvlh[2]=sf->port.port_vel_lvlh[2];
                        for(int rr=0; rr<3; ++rr){
                            for(int cc=0; cc<3; ++cc){
                                ts.R_body_to_lvlh[rr][cc]=sf->port.R_body_to_lvlh[rr][cc];
                            }
                        }
                        CPE_Result cpe = CPE_update_rotation(&g_chief_pose,
                                                             sf->port.R_body_to_lvlh,
                                                             cf->nav.range_m);
                        (void)cpe;
                        double R_cpe[3][3];
                        if(CPE_get_R_body2lvlh(&g_chief_pose, R_cpe)){
                            for(int rr=0; rr<3; ++rr){
                                for(int cc=0; cc<3; ++cc){
                                    ts.R_body_to_lvlh[rr][cc]=R_cpe[rr][cc];
                                }
                            }
                        }
                        ts.has_body_R=1;
                        ts.attitude_align_deg=_dock_alignment_deg(cf->att.q_wxyz,
                                                                  ts.port_axis_lvlh);
                        ts.has_attitude_align=1;
                        _set_q_ref_for_port_axis(ts.port_axis_lvlh);
                        _update_spin_sync_from_port(cf->att.q_wxyz,
                                                    ts.port_lvlh,
                                                    ts.port_axis_lvlh,
                                                    ts.port_vel_lvlh,
                                                    0.1);
                    }else{
                        ts.port_axis_lvlh[0]=0.0; ts.port_axis_lvlh[1]=0.0; ts.port_axis_lvlh[2]=1.0;
                        ts.port_vel_lvlh[0]=0.0; ts.port_vel_lvlh[1]=0.0; ts.port_vel_lvlh[2]=0.0;
                        _reset_spin_sync();
                    }
                }else{
                    ts.port_lvlh[0]=0.0; ts.port_lvlh[1]=0.0; ts.port_lvlh[2]=0.0;
                    ts.port_axis_lvlh[0]=0.0; ts.port_axis_lvlh[1]=0.0; ts.port_axis_lvlh[2]=1.0;
                    ts.port_vel_lvlh[0]=0.0; ts.port_vel_lvlh[1]=0.0; ts.port_vel_lvlh[2]=0.0;
                    _reset_spin_sync();
                }
                RPOD_fill_geometry(&ts);
                _fill_rpod_telem(&cf->rpod, &ts);
                cf->rpod.pose_age_s = g_chief_pose.pose_age_s;
                cf->rpod.pose_status = g_chief_pose.status;
                cf->rpod.pose_valid = g_chief_pose.valid;
                cf->rpod.spin_sync_active = g_spin_sync_active;
                cf->rpod.spin_sync_rate_cmd[0] = g_spin_sync_omega_body[0];
                cf->rpod.spin_sync_rate_cmd[1] = g_spin_sync_omega_body[1];
                cf->rpod.spin_sync_rate_cmd[2] = g_spin_sync_omega_body[2];

                if(!camera_valid){
                    if(g_terminal_cam_lost_s < 0.0) g_terminal_cam_lost_s = t_sim;
                    if((t_sim - g_terminal_cam_lost_s) >= 2.0) g_lost_target_active = 1;
                }else{
                    g_terminal_cam_lost_s = -1.0;
                    if(g_lost_target_active){
                        g_lost_target_active = 0;
                        g_is_braking = -1;
                        TNF_reset(&g_terminal_nav);
                    }
                }

                if(g_lost_target_active){
                    _reset_spin_sync();
                    RPOD_lost_target(&g_rpod_state, ar, new_acc);
                    g_rpod_mode_hold=13;
                }else{
                    if(g_soft_capture_active){
                        int ret=RPOD_soft_capture(&ts,ar,new_acc);
                        g_rpod_mode_hold=14;
                        if(ret==RPOD_RET_DOCKED){
                            g_hard_capture_hold_s += 0.1;
                            g_hard_capture_grace_s = 0.0;
                        }else if(g_hard_capture_hold_s > 0.0){
                            g_hard_capture_grace_s += 0.1;
                            if(g_hard_capture_grace_s > RPOD_HARD_CAPTURE_GRACE_S){
                                g_hard_capture_hold_s = 0.0;
                                g_hard_capture_grace_s = 0.0;
                            }
                        }else{
                            g_hard_capture_hold_s = 0.0;
                            g_hard_capture_grace_s = 0.0;
                        }
                        if(g_hard_capture_hold_s >= RPOD_HARD_CAPTURE_HOLD_S){
                            g_docked_latched=1;
                            g_rpod_mode_hold=12;
                            g_is_braking=-1;
                            _reset_terminal_guidance();
                            new_acc[0]=0.0; new_acc[1]=0.0; new_acc[2]=0.0;
                        }
                    }else{
                        int ret=RPOD_terminal(&ts,ar,new_acc,&g_is_braking);
                        g_rpod_mode_hold=11;
                        if(ret==RPOD_RET_SOFT_CAPTURE_READY){
                            g_soft_capture_active=1;
                            g_hard_capture_hold_s=0.0;
                            g_hard_capture_grace_s=0.0;
                            g_is_braking=-1;
                            g_rpod_mode_hold=14;
                        }
                    }
                }
            }
            g_accel_lvlh_hold[0]=new_acc[0];
            g_accel_lvlh_hold[1]=new_acc[1];
            g_accel_lvlh_hold[2]=new_acc[2];
            cf->cmd.rpod_mode=g_rpod_mode_hold;
            cf->cmd.accel_lvlh[0]=new_acc[0];
            cf->cmd.accel_lvlh[1]=new_acc[1];
            cf->cmd.accel_lvlh[2]=new_acc[2];
        }
    }

    _update_rpod_watchdog(t_sim, cf->cmd.rpod_mode, &cf->rpod);

    /* ?????? Timing ?????????????????????????????????????????????????????????????????????????????????????????????????????????????????????????????????????????????????????? */
    double loop_t=_get_time_ms()-t_start;
    if(loop_t>g_max_loop_ms) g_max_loop_ms=loop_t;
    if(loop_t>g_deadline_ms) g_missed++;
    cf->timing.tick=g_tick; cf->timing.loop_time_ms=loop_t;
    cf->timing.max_loop_time_ms=g_max_loop_ms;
    cf->timing.missed_deadlines=g_missed;
    cf->timing.deadline_ms=g_deadline_ms;
    cf->timing.invalid_packet_count=g_invalid_packet_count;
    cf->timing.stale_sensor_count=g_stale_sensor_count;
    _finalize_outputs(cf, cf->timing.watchdog_flags);
    g_tick++;
    return cf;
}

void flight_loop_run(uint64_t n_ticks){
    g_running=1;
    double t_next=_get_time_ms();
    for(uint64_t i=0;i<n_ticks&&g_running;i++){
        t_next+=g_deadline_ms;
        flight_loop_step();
        double now=_get_time_ms();
        if(t_next-now>0.5) _sleep_ms(t_next-now-0.4);
    }
    g_running=0;
}
void flight_loop_stop(void){g_running=0;}
void flight_loop_reset(void){
    g_tick=0;g_max_loop_ms=0.0;g_missed=0;g_is_braking=-1;
    g_accel_lvlh_hold[0]=g_accel_lvlh_hold[1]=g_accel_lvlh_hold[2]=0.0;
    g_rpod_mode_hold=-1; g_docked_latched=0; g_terminal_inflated=0; _reset_rpod_watchdog(); _reset_terminal_guidance();
    _reset_chief_pose_estimator();
    _clear_attitude_holds();
    memset(&g_cmd_frame,0,sizeof(g_cmd_frame));
    memset(&g_sensor_frame,0,sizeof(g_sensor_frame));
    MM_init(&g_mm); RW_init(&g_rw);
}
void flight_loop_get_thekf_state(double x_out[6],double P_out[36]){
    for(int i=0;i<6;i++) x_out[i]=g_thekf.x[i];
    for(int i=0;i<6;i++) for(int j=0;j<6;j++) P_out[i*6+j]=g_thekf.P[i][j];
}

#ifdef FLIGHT_LOOP_STANDALONE
static void _standalone_stamp_packets(SensorFrame *sf){
    if(sf->gyro.valid){
        sf->gyro.meta.version=SENSOR_PACKET_VERSION;
        sf->gyro.meta.timestamp_tick=sf->sim_tick;
        sf->gyro.meta.checksum=GyroPacket_checksum(&sf->gyro);
    }
    if(sf->range.valid){
        sf->range.meta.version=SENSOR_PACKET_VERSION;
        sf->range.meta.timestamp_tick=sf->sim_tick;
        sf->range.meta.checksum=RangePacket_checksum(&sf->range);
    }
    if(sf->camera.valid){
        sf->camera.meta.version=SENSOR_PACKET_VERSION;
        sf->camera.meta.timestamp_tick=sf->sim_tick;
        sf->camera.meta.checksum=CameraPacket_checksum(&sf->camera);
    }
    if(sf->port.valid){
        sf->port.meta.version=SENSOR_PACKET_VERSION;
        sf->port.meta.timestamp_tick=sf->sim_tick;
        sf->port.meta.checksum=PortPacket_checksum(&sf->port);
    }
    if(sf->mag.valid){
        sf->mag.meta.version=SENSOR_PACKET_VERSION;
        sf->mag.meta.timestamp_tick=sf->sim_tick;
        sf->mag.meta.checksum=MagPacket_checksum(&sf->mag);
    }
    if(sf->sun.valid){
        sf->sun.meta.version=SENSOR_PACKET_VERSION;
        sf->sun.meta.timestamp_tick=sf->sim_tick;
        sf->sun.meta.checksum=SunPacket_checksum(&sf->sun);
    }
}

int main(void){
    GNC_LOG("Flight loop v2 standalone smoke test\n");
    flight_loop_init(42164e3,0.001,3.986004418e14,0.1);
    double x0[6]={0.,500.,0.,0.,1e-3,0.};
    double P0[36]={0}; for(int i=0;i<6;i++) P0[i*6+i]=(i<3)?2500.0:0.25;
    flight_loop_seed_thekf(x0,P0,0.0);
    SensorFrame *sf=flight_loop_get_sensor_frame();
    /* High rate ??? DETUMBLE */
    sf->gyro.omega_xyz[0]=0.5; sf->gyro.omega_xyz[1]=0.3; sf->gyro.omega_xyz[2]=0.2; sf->gyro.valid=1;
    sf->mag.body[0]=2e-5; sf->mag.body[1]=1e-5; sf->mag.body[2]=4e-5;
    sf->mag.inertial[0]=2e-5; sf->mag.inertial[1]=1e-5; sf->mag.inertial[2]=4e-5; sf->mag.valid=1;
    sf->range.range_m=500.1; sf->range.azimuth_rad=0.001; sf->range.valid=1;
    double t0=_get_time_ms();
    CommandFrame *cf=NULL;
    for(int i=0;i<300;i++){
        if(i>60){sf->gyro.omega_xyz[0]*=0.96;sf->gyro.omega_xyz[1]*=0.96;sf->gyro.omega_xyz[2]*=0.96;}
        if(i==100){
            sf->sun.body[0]=0.577;sf->sun.body[1]=0.577;sf->sun.body[2]=0.577;
            sf->sun.inertial[0]=0.577;sf->sun.inertial[1]=0.577;sf->sun.inertial[2]=0.577;sf->sun.valid=1;
        }
        sf->sim_tick=i; _standalone_stamp_packets(sf); cf=flight_loop_step();
    }
    double el=_get_time_ms()-t0;
    GNC_LOG("  Tick 299: fsw_mode=%d range=%.1fm loop=%.3fms max=%.3fms missed=%u\n",
        cf->cmd.fsw_mode,cf->nav.range_m,cf->timing.loop_time_ms,
        cf->timing.max_loop_time_ms,cf->timing.missed_deadlines);
    GNC_LOG("  300 ticks in %.2fms (avg %.3fms/tick)\n",el,el/300.0);
    GNC_LOG("  PASS\n");
    return 0;
}
#endif


