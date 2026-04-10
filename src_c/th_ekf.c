/**
 * th_ekf.c — TH-EKF implementation
 * Direct C port of th_ekf.py for SIL.
 * Every function maps 1:1 to a Python method.
 */

#include "th_ekf.h"
#include "linalg.h"
#include <math.h>
#include <string.h>

/* ── Internal helpers (static — not exposed in header) ────────── */

/* dnu/dt = h * k^2 / p^2  where k = 1 + e*cos(nu)  */
static double _dnu_dt(const THEKF_State *ekf, double nu) {
    double k = 1.0 + ekf->e * cos(nu);
    return ekf->h_orb * k * k / (ekf->p * ekf->p);
}

/* Advance true anomaly by dt seconds using RK4.
   Maps to Python _advance_nu(). */
static double _advance_nu(const THEKF_State *ekf, double nu0, double dt) {
    double k1 = _dnu_dt(ekf, nu0);
    double k2 = _dnu_dt(ekf, nu0 + 0.5*dt*k1);
    double k3 = _dnu_dt(ekf, nu0 + 0.5*dt*k2);
    double k4 = _dnu_dt(ekf, nu0 +     dt*k3);
    return nu0 + (dt/6.0)*(k1 + 2.0*k2 + 2.0*k3 + k4);
}

/* Convert nu interval [nu0,nu1] to equivalent dt using trapezoidal rule.
   Maps to Python _nu_to_dt(). */
static double _nu_to_dt(const THEKF_State *ekf, double nu0, double nu1) {
    /* integrate dt/dnu = p^2 / (h * k^2) */
    double dnu = nu1 - nu0;
    double f0  = 1.0 / _dnu_dt(ekf, nu0);
    double f1  = 1.0 / _dnu_dt(ekf, nu1);
    return 0.5 * dnu * (f0 + f1);
}

/* CW State Transition Matrix (6×6).
   Maps to Python _cw_stm(). */
static void _cw_stm(const THEKF_State *ekf, double dt_m,
                    double Phi[6][6]) {
    double n  = ekf->n;
    double nt = n * dt_m;
    double s  = sin(nt);
    double c  = cos(nt);
    /* row 0 */ Phi[0][0]=4-3*c;     Phi[0][1]=0; Phi[0][2]=0; Phi[0][3]=s/n;           Phi[0][4]=2*(1-c)/n;      Phi[0][5]=0;
    /* row 1 */ Phi[1][0]=6*(s-nt);  Phi[1][1]=1; Phi[1][2]=0; Phi[1][3]=-2*(1-c)/n;    Phi[1][4]=(4*s-3*nt)/n;   Phi[1][5]=0;
    /* row 2 */ Phi[2][0]=0;          Phi[2][1]=0; Phi[2][2]=c; Phi[2][3]=0;             Phi[2][4]=0;              Phi[2][5]=s/n;
    /* row 3 */ Phi[3][0]=3*n*s;      Phi[3][1]=0; Phi[3][2]=0; Phi[3][3]=c;             Phi[3][4]=2*s;            Phi[3][5]=0;
    /* row 4 */ Phi[4][0]=6*n*(c-1); Phi[4][1]=0; Phi[4][2]=0; Phi[4][3]=-2*s;          Phi[4][4]=4*c-3;          Phi[4][5]=0;
    /* row 5 */ Phi[5][0]=0;          Phi[5][1]=0; Phi[5][2]=-n*s; Phi[5][3]=0;         Phi[5][4]=0;              Phi[5][5]=c;
}

/* CW control input integral (6-vector).
   Maps to Python _cw_control_input(). */
static void _cw_control_input(const THEKF_State *ekf,
                               const double accel[3], double dt_m,
                               double Bu[6]) {
    double n  = ekf->n;
    double n2 = n * n;
    double nt = n * dt_m;
    double s  = sin(nt);
    double c  = cos(nt);
    double ax = accel[0], ay = accel[1], az = accel[2];
    Bu[0] = (ax*(4*s - 3*nt) + 2*ay*(1 - c)) / n2;
    Bu[1] = (-2*ax*(1 - c) + ay*(4*s/n - 3*dt_m)) / n;
    Bu[2] = az*(1 - c) / n2;
    Bu[3] = (ax*s + 2*ay*(1-c)) / n;
    Bu[4] = (-2*ax*(1-c) + ay*(4*s - 3*nt)) / n;
    Bu[5] = az*s / n;
}

/* Measurement function z = h(dr): [range, az, el].
   Maps to Python _h(). */
static void _h_meas(const double dr[3], double z_pred[3]) {
    double r  = sqrt(dr[0]*dr[0] + dr[1]*dr[1] + dr[2]*dr[2]);
    double az = atan2(dr[1], dr[0]);     /* azimuth in xy plane */
    double el = atan2(dr[2], sqrt(dr[0]*dr[0] + dr[1]*dr[1]));
    z_pred[0] = r;
    z_pred[1] = az;
    z_pred[2] = el;
}

/* Measurement Jacobian H (3×6).
   Maps to Python _H_jac(). */
static void _H_jac(const double dr[3], double H[3][6]) {
    double x = dr[0], y = dr[1], z = dr[2];
    double r     = sqrt(x*x + y*y + z*z);
    double r_xy2 = x*x + y*y;
    double r_xy  = sqrt(r_xy2);
    memset(H, 0, 3*6*sizeof(double));
    if (r < 1e-6 || r_xy < 1e-6) return;
    /* range row */
    H[0][0] = x/r;  H[0][1] = y/r;  H[0][2] = z/r;
    /* azimuth row */
    H[1][0] = -y/r_xy2;  H[1][1] = x/r_xy2;
    /* elevation row */
    H[2][0] = -x*z/(r*r*r_xy);
    H[2][1] = -y*z/(r*r*r_xy);
    H[2][2] =  r_xy/(r*r);
}

/* ── Public API ───────────────────────────────────────────────── */

void THEKF_init(THEKF_State *ekf,
                double a_chief_m, double e_chief,
                double mu, double dt_s,
                double q_pos, double q_vel) {
    memset(ekf, 0, sizeof(THEKF_State));
    ekf->a    = a_chief_m;
    ekf->e    = e_chief;
    ekf->mu   = mu;
    ekf->dt   = dt_s;
    ekf->n    = sqrt(mu / (a_chief_m*a_chief_m*a_chief_m));
    ekf->p    = a_chief_m * (1.0 - e_chief*e_chief);
    ekf->h_orb= sqrt(mu * ekf->p);
    ekf->eta  = sqrt(1.0 - e_chief*e_chief);

    /* P_init: 1m pos std, 0.01m/s vel std — matches Python */
    for (int i = 0; i < 3; i++) ekf->P[i][i]   = 1.0;
    for (int i = 3; i < 6; i++) ekf->P[i][i]   = 1e-4;

    /* Q: diagonal, scaled by dt — matches Python */
    for (int i = 0; i < 3; i++) ekf->Q[i][i]   = q_pos * dt_s;
    for (int i = 3; i < 6; i++) ekf->Q[i][i]   = q_vel * dt_s;
}

void THEKF_seed(THEKF_State *ekf,
                const double x0[THEKF_NX],
                const double P0[THEKF_NX][THEKF_NX],
                double nu0) {
    memcpy(ekf->x, x0, THEKF_NX * sizeof(double));
    ekf->nu = nu0;
    if (P0 != NULL)
        memcpy(ekf->P, P0, THEKF_NX*THEKF_NX*sizeof(double));
}

void THEKF_predict(THEKF_State *ekf, const double accel_lvlh[3]) {
    static const double zero3[3] = {0,0,0};
    if (accel_lvlh == NULL) accel_lvlh = zero3;

    /* Advance true anomaly — matches Python _advance_nu() */
    double nu0  = ekf->nu;
    double nu1  = _advance_nu(ekf, nu0, ekf->dt);
    double dt_m = _nu_to_dt(ekf, nu0, nu1);

    /* STM */
    double Phi[6][6];
    _cw_stm(ekf, dt_m, Phi);

    /* x = Phi @ x */
    double x_new[6];
    MAT_VEC(x_new, Phi, ekf->x, 6, 6);

    /* x += Bu if accel non-zero */
    if (accel_lvlh[0] != 0.0 || accel_lvlh[1] != 0.0 || accel_lvlh[2] != 0.0) {
        double Bu[6];
        _cw_control_input(ekf, accel_lvlh, dt_m, Bu);
        for (int i = 0; i < 6; i++) x_new[i] += Bu[i];
    }
    memcpy(ekf->x, x_new, 6*sizeof(double));

    /* P = Phi @ P @ Phi^T + Q */
    double PhiP[6][6], PhiT[6][6], PhiPPhiT[6][6];
    MAT_MUL(PhiP, Phi, ekf->P, 6, 6, 6);
    MAT_T(PhiT, Phi, 6, 6);
    MAT_MUL(PhiPPhiT, PhiP, PhiT, 6, 6, 6);
    MAT_ADD(ekf->P, PhiPPhiT, ekf->Q, 6, 6);
    MAT_SYM(ekf->P, 6);

    ekf->nu   = nu1;
    ekf->t_ekf += ekf->dt;
}

int THEKF_update(THEKF_State *ekf,
                 const double z_meas[THEKF_NZ],
                 const double R_meas[THEKF_NZ][THEKF_NZ],
                 double gate_k) {
    double *dr = ekf->x;   /* position part of state */
    double r   = sqrt(dr[0]*dr[0] + dr[1]*dr[1] + dr[2]*dr[2]);
    if (r < 1.0) return 0;

    /* Predicted measurement */
    double z_pred[3];
    _h_meas(dr, z_pred);

    /* Innovation */
    double innov[3];
    innov[0] = z_meas[0] - z_pred[0];
    innov[1] = wrap_pi(z_meas[1] - z_pred[1]);
    innov[2] = wrap_pi(z_meas[2] - z_pred[2]);

    /* H Jacobian (3×6) */
    double H[3][6];
    _H_jac(dr, H);

    /* S = H @ P @ H^T + R  (3×3) */
    double HP[3][6], HT[6][3], HPHT[3][3], S[3][3];
    MAT_MUL(HP, H, ekf->P, 3, 6, 6);
    MAT_T(HT, H, 3, 6);
    MAT_MUL(HPHT, HP, HT, 3, 6, 3);
    MAT_ADD(S, HPHT, R_meas, 3, 3);

    /* S^-1 */
    double S_inv[3][3];
    if (mat3_inv(S_inv, S) != 0) return 0;

    /* Mahalanobis gate */
    double Si_innov[3];
    MAT_VEC(Si_innov, S_inv, innov, 3, 3);
    double mahal = 0.0;
    for (int i = 0; i < 3; i++) mahal += innov[i] * Si_innov[i];
    if (mahal > gate_k*gate_k) return 0;

    /* K = P @ H^T @ S^-1  (6×3) */
    double PHT[6][3], K[6][3];
    MAT_MUL(PHT, ekf->P, HT, 6, 6, 3);
    MAT_MUL(K, PHT, S_inv, 6, 3, 3);

    /* x += K @ innov */
    for (int i = 0; i < 6; i++) {
        double ki = 0.0;
        for (int j = 0; j < 3; j++) ki += K[i][j]*innov[j];
        ekf->x[i] += ki;
    }

    /* P = (I - K@H) @ P @ (I-K@H)^T + K @ R @ K^T  (Joseph form) */
    double KH[6][6], IKH[6][6], I6[6][6];
    MAT_EYE(I6, 6);
    MAT_MUL(KH, K, H, 6, 3, 6);
    MAT_SUB(IKH, I6, KH, 6, 6);

    double IKHP[6][6], IKHT[6][6], IKHPIKHT[6][6];
    MAT_MUL(IKHP, IKH, ekf->P, 6, 6, 6);
    MAT_T(IKHT, IKH, 6, 6);
    MAT_MUL(IKHPIKHT, IKHP, IKHT, 6, 6, 6);

    /* KRK^T term */
    double KT[3][6], KR[6][3], KRKT[6][6];
    MAT_T(KT, K, 6, 3);
    MAT_MUL(KR, K, R_meas, 6, 3, 3);
    MAT_MUL(KRKT, KR, KT, 6, 3, 6);

    MAT_ADD(ekf->P, IKHPIKHT, KRKT, 6, 6);
    MAT_SYM(ekf->P, 6);

    return 1;
}