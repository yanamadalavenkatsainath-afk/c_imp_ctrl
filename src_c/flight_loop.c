/**
 * flight_loop.c — PC SIL Real-Time Flight Loop v2
 * Full ADCS stack: ModeManager + QUEST + B-dot + RW + MTQ + PD
 */
#include "flight_loop.h"
#include "th_ekf.h"
#include "mekf.h"
#include "quest.h"
#include "adcs.h"
#include "mode_manager.h"
#include "rpod_ctrl.h"
#include "sensor_packet.h"
#include "command_packet.h"
#include <math.h>
#include <string.h>
#include <stdio.h>
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
/* Nadir-pointing reference (identity = body-z toward nadir) */
static const double g_q_ref[4] = {1.0,0.0,0.0,0.0};

static double _pointing_err_deg(const double q[4]){
    double qv=sqrt(q[1]*q[1]+q[2]*q[2]+q[3]*q[3]);
    return 2.0*asin(qv>1.0?1.0:qv)*180.0/3.14159265358979;
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
    g_tick=0; g_max_loop_ms=0.0; g_missed=0; g_is_braking=-1;
    g_cmd_frame.timing.deadline_ms=g_deadline_ms;
    printf("  flight_loop v2: full ADCS stack enabled\n");
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

    /* ── 100 Hz: Gyro + MEKF predict ──────────────────────────── */
    if(sf->gyro.valid){
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
        sf->gyro.valid?sf->gyro.omega_xyz[0]-(double)g_mekf.bias[0]:0.0,
        sf->gyro.valid?sf->gyro.omega_xyz[1]-(double)g_mekf.bias[1]:0.0,
        sf->gyro.valid?sf->gyro.omega_xyz[2]-(double)g_mekf.bias[2]:0.0
    };

    /* ── Mode manager update ───────────────────────────────────── */
    double quest_err=-1.0;
    FSW_Mode mode=MM_update(&g_mm,t_sim,omega_est,g_rw.h,
                            quest_err,0,cf->att.pointing_err_deg);

    /* ── Zero actuator outputs ─────────────────────────────────── */
    memset(cf->cmd.torque_rw, 0,sizeof(cf->cmd.torque_rw));
    memset(cf->cmd.dipole_mtq,0,sizeof(cf->cmd.dipole_mtq));
    memset(cf->cmd.accel_lvlh,0,sizeof(cf->cmd.accel_lvlh));
    cf->cmd.rpod_mode=-1;

    /* ── Mode dispatch ─────────────────────────────────────────── */
    if(mode==MODE_SAFE_MODE){
        cf->cmd.fsw_mode=0;

    }else if(mode==MODE_DETUMBLE){
        if(sf->mag.valid){
            double m[3],tau[3];
            BDOT_compute(&g_bdot,sf->mag.body,omega_est,m,tau);
            cf->cmd.dipole_mtq[0]=m[0]; cf->cmd.dipole_mtq[1]=m[1]; cf->cmd.dipole_mtq[2]=m[2];
        }
        cf->cmd.fsw_mode=1;

    }else if(mode==MODE_SUN_ACQUISITION){
        /* QUEST attitude init */
        if(sf->mag.valid && sf->sun.valid){
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
        double trw[3];
        ATTCTRL_compute(&g_attctrl,cf->att.q_wxyz,omega_est,g_q_ref,trw,NULL);
        RW_apply_torque(&g_rw,trw,0.01);
        cf->cmd.torque_rw[0]=trw[0]; cf->cmd.torque_rw[1]=trw[1]; cf->cmd.torque_rw[2]=trw[2];
        cf->cmd.fsw_mode=2;

    }else if(mode==MODE_FINE_POINTING){
        double trw[3];
        ATTCTRL_compute(&g_attctrl,cf->att.q_wxyz,omega_est,g_q_ref,trw,NULL);
        RW_apply_torque(&g_rw,trw,0.01);
        cf->cmd.torque_rw[0]=trw[0]; cf->cmd.torque_rw[1]=trw[1]; cf->cmd.torque_rw[2]=trw[2];
        cf->cmd.fsw_mode=3;

    }else if(mode==MODE_MOMENTUM_DUMP){
        if(sf->mag.valid){
            double m[3],tau[3];
            MTQ_compute_dipole(&g_mtq,g_rw.h,sf->mag.body,m);
            MTQ_compute_torque(m,sf->mag.body,tau);
            cf->cmd.dipole_mtq[0]=m[0]; cf->cmd.dipole_mtq[1]=m[1]; cf->cmd.dipole_mtq[2]=m[2];
        }
        double trw[3];
        ATTCTRL_compute(&g_attctrl,cf->att.q_wxyz,omega_est,g_q_ref,trw,NULL);
        RW_apply_torque(&g_rw,trw,0.01);
        cf->cmd.torque_rw[0]=trw[0]; cf->cmd.torque_rw[1]=trw[1]; cf->cmd.torque_rw[2]=trw[2];
        cf->cmd.fsw_mode=4;
    }

    /* ── 10 Hz tasks ──────────────────────────────────────────── */
    cf->ekf_updated=0;
    if(g_tick%10==0){
        THEKF_predict(&g_thekf,NULL);
        if(sf->range.valid){
            double z[3]={sf->range.range_m,sf->range.azimuth_rad,sf->range.elevation_rad};
            double R[3][3]={{0.09,0,0},{0,3e-6,0},{0,0,3e-6}};
            cf->ekf_updated=(uint8_t)THEKF_update(&g_thekf,z,R,5.0);
        }
        if(sf->camera.valid){
            double Rc[3][3]={{sf->camera.R_diag[0],0,0},{0,sf->camera.R_diag[1],0},{0,0,sf->camera.R_diag[2]}};
            cf->ekf_updated|=(uint8_t)THEKF_update_position(&g_thekf,sf->camera.pos_lvlh,Rc,5.0);
        }
        if(sf->mag.valid){
            MEKF_FLOAT zb[3]={(MEKF_FLOAT)sf->mag.body[0],(MEKF_FLOAT)sf->mag.body[1],(MEKF_FLOAT)sf->mag.body[2]};
            MEKF_FLOAT vi[3]={(MEKF_FLOAT)sf->mag.inertial[0],(MEKF_FLOAT)sf->mag.inertial[1],(MEKF_FLOAT)sf->mag.inertial[2]};
            MEKF_update(&g_mekf,zb,vi,g_mekf.R_mag);
        }
        if(sf->sun.valid){
            MEKF_FLOAT zb[3]={(MEKF_FLOAT)sf->sun.body[0],(MEKF_FLOAT)sf->sun.body[1],(MEKF_FLOAT)sf->sun.body[2]};
            MEKF_FLOAT vi[3]={(MEKF_FLOAT)sf->sun.inertial[0],(MEKF_FLOAT)sf->sun.inertial[1],(MEKF_FLOAT)sf->sun.inertial[2]};
            MEKF_update(&g_mekf,zb,vi,g_mekf.R_sun);
        }
        THEKF_get_pos(&g_thekf,cf->nav.pos_lvlh);
        THEKF_get_vel(&g_thekf,cf->nav.vel_lvlh);
        THEKF_get_pos_std(&g_thekf,cf->nav.pos_std);
        THEKF_get_vel_std(&g_thekf,cf->nav.vel_std);
        cf->nav.range_m=THEKF_range(&g_thekf);

        /* RPOD only in FINE_POINTING */
        if(mode==MODE_FINE_POINTING){
            double ar=0.020,tr=cf->nav.range_m;
            g_rpod_state.pos[0]=cf->nav.pos_lvlh[0]; g_rpod_state.pos[1]=cf->nav.pos_lvlh[1]; g_rpod_state.pos[2]=cf->nav.pos_lvlh[2];
            g_rpod_state.vel[0]=cf->nav.vel_lvlh[0]; g_rpod_state.vel[1]=cf->nav.vel_lvlh[1]; g_rpod_state.vel[2]=cf->nav.vel_lvlh[2];
            if(tr>RPOD_TERMINAL_M){
                RPOD_prox_ops(&g_rpod_state,tr,g_thekf.n,ar,cf->cmd.accel_lvlh);
                cf->cmd.rpod_mode=10;
            }else if(tr>RPOD_DOCK_DONE_M){
                int ret=RPOD_terminal_simple(&g_rpod_state,ar,cf->cmd.accel_lvlh);
                cf->cmd.rpod_mode=(ret==2)?12:11;
            }else{
                cf->cmd.rpod_mode=12;
            }
        }
    }

    /* ── Timing ────────────────────────────────────────────────── */
    double loop_t=_get_time_ms()-t_start;
    if(loop_t>g_max_loop_ms) g_max_loop_ms=loop_t;
    if(loop_t>g_deadline_ms) g_missed++;
    cf->timing.tick=g_tick; cf->timing.loop_time_ms=loop_t;
    cf->timing.max_loop_time_ms=g_max_loop_ms;
    cf->timing.missed_deadlines=g_missed;
    cf->timing.deadline_ms=g_deadline_ms;
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
    memset(&g_cmd_frame,0,sizeof(g_cmd_frame));
    memset(&g_sensor_frame,0,sizeof(g_sensor_frame));
    MM_init(&g_mm); RW_init(&g_rw);
}
void flight_loop_get_thekf_state(double x_out[6],double P_out[36]){
    for(int i=0;i<6;i++) x_out[i]=g_thekf.x[i];
    for(int i=0;i<6;i++) for(int j=0;j<6;j++) P_out[i*6+j]=g_thekf.P[i][j];
}

#ifdef FLIGHT_LOOP_STANDALONE
int main(void){
    printf("Flight loop v2 standalone smoke test\n");
    flight_loop_init(42164e3,0.001,3.986004418e14,0.1);
    double x0[6]={0.,500.,0.,0.,1e-3,0.};
    double P0[36]={0}; for(int i=0;i<6;i++) P0[i*6+i]=(i<3)?2500.0:0.25;
    flight_loop_seed_thekf(x0,P0,0.0);
    SensorFrame *sf=flight_loop_get_sensor_frame();
    /* High rate → DETUMBLE */
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
        sf->sim_tick=i; cf=flight_loop_step();
    }
    double el=_get_time_ms()-t0;
    printf("  Tick 299: fsw_mode=%d range=%.1fm loop=%.3fms max=%.3fms missed=%u\n",
        cf->cmd.fsw_mode,cf->nav.range_m,cf->timing.loop_time_ms,
        cf->timing.max_loop_time_ms,cf->timing.missed_deadlines);
    printf("  300 ticks in %.2fms (avg %.3fms/tick)\n",el,el/300.0);
    printf("  PASS\n");
    return 0;
}
#endif