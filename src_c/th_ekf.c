/**
 * th_ekf.c — TH-EKF implementation (updated to match th_ekf.py)
 * ==============================================================
 * Every public function maps 1:1 to a Python method.
 *
 * New vs old C:
 *   + THEKF_update_position()        — linear camera position update
 *   + THEKF_update_velocity_doppler()— scalar Doppler radial update
 *   + THEKF_inflate_process_noise()  — P inflation at TERMINAL entry
 *   + P ceiling in THEKF_predict()   — sigma_pos ≤ 50m, sigma_vel ≤ 1m/s
 *   ~ _nu_to_dt() now uses Kepler's equation (matches Python exactly)
 */

#include "th_ekf.h"
#include "linalg.h"
#include <math.h>
#include <string.h>

/* ── Internal helpers ─────────────────────────────────────────── */

static double _dnu_dt(const THEKF_State *ekf, double nu) {
    double k = 1.0 + ekf->e * cos(nu);
    return ekf->h_orb * k * k / (ekf->p * ekf->p);
}

/* Advance true anomaly by dt seconds — RK4. Maps to Python _advance_nu(). */
static double _advance_nu(const THEKF_State *ekf, double nu0, double dt) {
    double k1 = _dnu_dt(ekf, nu0);
    double k2 = _dnu_dt(ekf, nu0 + 0.5*dt*k1);
    double k3 = _dnu_dt(ekf, nu0 + 0.5*dt*k2);
    double k4 = _dnu_dt(ekf, nu0 +     dt*k3);
    return nu0 + (dt/6.0)*(k1 + 2.0*k2 + 2.0*k3 + k4);
}

/**
 * _nu_to_M — convert true anomaly to mean anomaly.
 * File-scope helper (avoids GNU nested-function extension).
 * Matches Python: tan(E/2) = sqrt((1-e)/(1+e)) * tan(nu/2)
 * Uses atan2 of sin/cos to handle all quadrants without ambiguity.
 */
static double _nu_to_M(double nu, double e) {
    /* Use atan2(sin, cos) form to avoid quadrant ambiguity */
    double sq  = sqrt((1.0 - e) / (1.0 + e));
    double hnu = 0.5 * nu;
    double E   = 2.0 * atan2(sq * sin(hnu), cos(hnu));
    return E - e * sin(E);
}

/**
 * _nu_to_dt — convert true anomaly interval [nu0, nu1] to elapsed time.
 * Uses Kepler's equation (matches Python _nu_to_dt exactly).
 * dM wrapped to [0, 2π] so dt is always positive — mirrors Python % (2*pi).
 */
static double _nu_to_dt(const THEKF_State *ekf, double nu0, double nu1) {
    double M0 = _nu_to_M(nu0, ekf->e);
    double M1 = _nu_to_M(nu1, ekf->e);
    double dM = fmod(M1 - M0 + 2.0*M_PI, 2.0*M_PI);   /* wrap to [0, 2π] */
    return dM / ekf->n;
}

/* CW State Transition Matrix (6×6). Maps to Python _cw_stm(). */
static void _cw_stm(const THEKF_State *ekf, double dt_m, double Phi[6][6]) {
    double n  = ekf->n;
    double nt = n * dt_m;
    double s  = sin(nt);
    double c  = cos(nt);

    Phi[0][0]=4-3*c;       Phi[0][1]=0; Phi[0][2]=0;
    Phi[0][3]=s/n;          Phi[0][4]=2*(1-c)/n;     Phi[0][5]=0;

    Phi[1][0]=6*(s-nt);    Phi[1][1]=1; Phi[1][2]=0;
    Phi[1][3]=-2*(1-c)/n;  Phi[1][4]=(4*s-3*nt)/n;  Phi[1][5]=0;

    Phi[2][0]=0;            Phi[2][1]=0; Phi[2][2]=c;
    Phi[2][3]=0;            Phi[2][4]=0;              Phi[2][5]=s/n;

    Phi[3][0]=3*n*s;        Phi[3][1]=0; Phi[3][2]=0;
    Phi[3][3]=c;            Phi[3][4]=2*s;            Phi[3][5]=0;

    Phi[4][0]=6*n*(c-1);   Phi[4][1]=0; Phi[4][2]=0;
    Phi[4][3]=-2*s;         Phi[4][4]=4*c-3;          Phi[4][5]=0;

    Phi[5][0]=0;            Phi[5][1]=0; Phi[5][2]=-n*s;
    Phi[5][3]=0;            Phi[5][4]=0;              Phi[5][5]=c;
}

/* CW control input integral. Maps to Python _cw_control_input(). */
static void _cw_control_input(const THEKF_State *ekf,
                               const double accel[3], double dt_m,
                               double Bu[6]) {
    double n  = ekf->n;
    double n2 = n * n;
    double nt = n * dt_m;
    double s  = sin(nt);
    double c  = cos(nt);
    double ax = accel[0], ay = accel[1], az = accel[2];

    Bu[0] = (ax*(1 - c) + 2*ay*(nt - s)) / n2;
    Bu[1] = (2*ax*(s - nt)) / n2 + ay*(4*(1 - c)/n2 - 1.5*dt_m*dt_m);
    Bu[2] = az*(1 - c) / n2;
    Bu[3] = (ax*s + 2*ay*(1-c)) / n;
    Bu[4] = (-2*ax*(1-c) + ay*(4*s - 3*nt)) / n;
    Bu[5] = az*s / n;
}

/* Measurement function h(dr) = [range, az, el]. Maps to Python _h(). */
static void _h_meas(const double dr[3], double z_pred[3]) {
    double r  = sqrt(dr[0]*dr[0] + dr[1]*dr[1] + dr[2]*dr[2]);
    double az = atan2(dr[1], dr[0]);
    double el = atan2(dr[2], sqrt(dr[0]*dr[0] + dr[1]*dr[1]));
    z_pred[0] = r;
    z_pred[1] = az;
    z_pred[2] = el;
}

/* Measurement Jacobian H (3×6). Maps to Python _H_jac(). */
static void _H_jac(const double dr[3], double H[3][6]) {
    double x = dr[0], y = dr[1], z = dr[2];
    double r     = sqrt(x*x + y*y + z*z);
    double r_xy2 = x*x + y*y;
    double r_xy  = sqrt(r_xy2);

    memset(H, 0, 3*6*sizeof(double));
    if (r < 1e-6 || r_xy < 1e-6) return;

    H[0][0] = x/r;          H[0][1] = y/r;          H[0][2] = z/r;
    H[1][0] = -y/r_xy2;     H[1][1] = x/r_xy2;
    H[2][0] = -x*z/(r*r*r_xy);
    H[2][1] = -y*z/(r*r*r_xy);
    H[2][2] =  r_xy/(r*r);
}

/* ── Joseph-form covariance update (shared utility) ──────────── */
/*
 * P = (I - K@H) @ P @ (I - K@H)^T + K @ R @ K^T
 * K  : 6×nz
 * H  : nz×6
 * R  : nz×nz
 * nz : measurement dimension (1 or 3)
 */
static void _joseph_update_3(THEKF_State *ekf,
                              const double K[6][3],
                              const double H[3][6],
                              const double R[3][3]) {
    double KH[6][6], IKH[6][6], I6[6][6];
    MAT_EYE(I6, 6);
    MAT_MUL(KH, K, H, 6, 3, 6);
    MAT_SUB(IKH, I6, KH, 6, 6);

    double IKHP[6][6], IKHT[6][6], IKHPIKHT[6][6];
    MAT_MUL(IKHP, IKH, ekf->P, 6, 6, 6);
    MAT_T(IKHT, IKH, 6, 6);
    MAT_MUL(IKHPIKHT, IKHP, IKHT, 6, 6, 6);

    double KT[3][6], KR[6][3], KRKT[6][6];
    MAT_T(KT, K, 6, 3);
    MAT_MUL(KR, K, R, 6, 3, 3);
    MAT_MUL(KRKT, KR, KT, 6, 3, 6);

    MAT_ADD(ekf->P, IKHPIKHT, KRKT, 6, 6);
    MAT_SYM(ekf->P, 6);
}

/* ── P ceiling helper — mirrors Python predict() ceiling ─────── */
/*
 * Only clamps diagonal elements that EXCEED the ceiling.
 * Must not restore P that has legitimately shrunk via EKF updates.
 * Row/column scaling is applied only when the diagonal grows past
 * the ceiling (divergence guard), not as a blanket reset.
 */
static void _apply_p_ceiling(THEKF_State *ekf) {
    static const double ceil_diag[6] = {
        THEKF_P_CEIL_POS, THEKF_P_CEIL_POS, THEKF_P_CEIL_POS,
        THEKF_P_CEIL_VEL, THEKF_P_CEIL_VEL, THEKF_P_CEIL_VEL
    };
    for (int i = 0; i < 6; i++) {
        double pd = ekf->P[i][i];
        if (pd > ceil_diag[i] * 1.0001) {   /* only trim actual growth */
            double scale = sqrt(ceil_diag[i] / pd);
            for (int j = 0; j < 6; j++) {
                ekf->P[i][j] *= scale;
                ekf->P[j][i] *= scale;
            }
            ekf->P[i][i] = ceil_diag[i];
        }
    }
}

/* ── Public API ───────────────────────────────────────────────── */

void THEKF_init(THEKF_State *ekf,
                double a_chief_m, double e_chief,
                double mu, double dt_s,
                double q_pos, double q_vel) {
    memset(ekf, 0, sizeof(THEKF_State));
    ekf->a     = a_chief_m;
    ekf->e     = e_chief;
    ekf->mu    = mu;
    ekf->dt    = dt_s;
    ekf->n     = sqrt(mu / (a_chief_m * a_chief_m * a_chief_m));
    ekf->p     = a_chief_m * (1.0 - e_chief * e_chief);
    ekf->h_orb = sqrt(mu * ekf->p);
    ekf->eta   = sqrt(1.0 - e_chief * e_chief);

    /* P_init: 1m position std, 0.01m/s velocity std */
    for (int i = 0; i < 3; i++) ekf->P[i][i]   = 1.0;
    for (int i = 3; i < 6; i++) ekf->P[i][i]   = 1e-4;

    /* Q: diagonal, scaled by dt */
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
        memcpy(ekf->P, P0, THEKF_NX * THEKF_NX * sizeof(double));
}

void THEKF_predict(THEKF_State *ekf, const double accel_lvlh[3]) {
    static const double zero3[3] = {0, 0, 0};
    if (accel_lvlh == NULL) accel_lvlh = zero3;

    double nu0  = ekf->nu;
    double nu1  = _advance_nu(ekf, nu0, ekf->dt);
    double dt_m = _nu_to_dt(ekf, nu0, nu1);

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
    memcpy(ekf->x, x_new, 6 * sizeof(double));

    /* P = Phi @ P @ Phi^T + Q */
    double PhiP[6][6], PhiT[6][6], PhiPPhiT[6][6];
    MAT_MUL(PhiP, Phi, ekf->P, 6, 6, 6);
    MAT_T(PhiT, Phi, 6, 6);
    MAT_MUL(PhiPPhiT, PhiP, PhiT, 6, 6, 6);
    MAT_ADD(ekf->P, PhiPPhiT, ekf->Q, 6, 6);
    MAT_SYM(ekf->P, 6);

    /* Apply P ceiling — prevents Mahalanobis gate becoming trivial */
    _apply_p_ceiling(ekf);

    ekf->nu    = nu1;
    ekf->t_ekf += ekf->dt;
}

int THEKF_update(THEKF_State *ekf,
                 const double z_meas[THEKF_NZ],
                 const double R_meas[THEKF_NZ][THEKF_NZ],
                 double gate_k) {
    double *dr = ekf->x;
    double r   = sqrt(dr[0]*dr[0] + dr[1]*dr[1] + dr[2]*dr[2]);
    if (r < 1.0) return 0;

    double z_pred[3];
    _h_meas(dr, z_pred);

    double innov[3];
    innov[0] = z_meas[0] - z_pred[0];
    innov[1] = wrap_pi(z_meas[1] - z_pred[1]);
    innov[2] = wrap_pi(z_meas[2] - z_pred[2]);

    double H[3][6];
    _H_jac(dr, H);

    double HP[3][6], HT[6][3], HPHT[3][3], S[3][3];
    MAT_MUL(HP, H, ekf->P, 3, 6, 6);
    MAT_T(HT, H, 3, 6);
    MAT_MUL(HPHT, HP, HT, 3, 6, 3);
    MAT_ADD(S, HPHT, R_meas, 3, 3);

    double S_inv[3][3];
    if (mat3_inv(S_inv, S) != 0) return 0;

    double Si_innov[3];
    MAT_VEC(Si_innov, S_inv, innov, 3, 3);
    double mahal = 0.0;
    for (int i = 0; i < 3; i++) mahal += innov[i] * Si_innov[i];
    if (mahal > gate_k * gate_k) return 0;

    double PHT[6][3], K[6][3];
    MAT_MUL(PHT, ekf->P, HT, 6, 6, 3);
    MAT_MUL(K, PHT, S_inv, 6, 3, 3);

    for (int i = 0; i < 6; i++) {
        double ki = 0.0;
        for (int j = 0; j < 3; j++) ki += K[i][j] * innov[j];
        ekf->x[i] += ki;
    }

    _joseph_update_3(ekf, K, H, R_meas);
    return 1;
}

/* ── THEKF_update_position — linear camera position update ───── */
int THEKF_update_position(THEKF_State *ekf,
                           const double z_pos[3],
                           const double R_pos[3][3],
                           double gate_k) {
    /*
     * H = [I₃ | 0₃]  (3×6)
     * z_pred = x[0:3]
     * innov  = z_pos - x[0:3]
     */
    double innov[3];
    for (int i = 0; i < 3; i++)
        innov[i] = z_pos[i] - ekf->x[i];

    /* H (3×6): position rows only — linear, exact */
    double H[3][6];
    memset(H, 0, sizeof(H));
    H[0][0] = 1.0;
    H[1][1] = 1.0;
    H[2][2] = 1.0;

    /* S = H @ P @ H^T + R  =  P[0:3,0:3] + R */
    double S[3][3];
    for (int i = 0; i < 3; i++)
        for (int j = 0; j < 3; j++)
            S[i][j] = ekf->P[i][j] + R_pos[i][j];

    double S_inv[3][3];
    if (mat3_inv(S_inv, S) != 0) return 0;

    /* Mahalanobis gate */
    double Si_innov[3];
    MAT_VEC(Si_innov, S_inv, innov, 3, 3);
    double mahal = 0.0;
    for (int i = 0; i < 3; i++) mahal += innov[i] * Si_innov[i];
    if (mahal > gate_k * gate_k) return 0;

    /* Absolute innovation gate — rejects camera spikes that pass a large-P gate.
     * Mirrors Python: if innov_m > 10.0: return False  (checked AFTER Mahalanobis) */
    double innov_m = sqrt(innov[0]*innov[0] + innov[1]*innov[1] + innov[2]*innov[2]);
    if (innov_m > THEKF_INNOV_POS_MAX_M) return 0;

    /* K = P @ H^T @ S^-1  = P[:,0:3] @ S^-1  (6×3) */
    double PHT[6][3];
    for (int i = 0; i < 6; i++)
        for (int j = 0; j < 3; j++)
            PHT[i][j] = ekf->P[i][j];   /* P @ H^T = P[:,0:3] since H^T = [I | 0]^T */

    double K[6][3];
    MAT_MUL(K, PHT, S_inv, 6, 3, 3);

    /* x += K @ innov */
    for (int i = 0; i < 6; i++) {
        double ki = 0.0;
        for (int j = 0; j < 3; j++) ki += K[i][j] * innov[j];
        ekf->x[i] += ki;
    }

    /* Joseph-form P update */
    _joseph_update_3(ekf, K, H, R_pos);

    /* Ensure P stays PSD — add small diagonal floor if eigenvalue negative */
    /* Simple check: if any P[i][i] < 0, add offset */
    for (int i = 0; i < 6; i++) {
        if (ekf->P[i][i] < 0.0) {
            double offset = -ekf->P[i][i] + 1e-12;
            for (int k = 0; k < 6; k++)
                ekf->P[k][k] += offset;
            break;
        }
    }

    return 1;
}

/* ── THEKF_update_velocity_doppler — scalar Doppler update ───── */
int THEKF_update_velocity_doppler(THEKF_State *ekf,
                                   double v_radial_meas,
                                   const double r_hat[3],
                                   double sigma_radial) {
    /*
     * H = [0, 0, 0, r̂ₓ, r̂ᵧ, r̂_z]   (1×6)
     * z_pred = H @ x = dot(r_hat, x[3:6])
     * innov  = v_radial_meas - z_pred
     */
    double z_pred = r_hat[0]*ekf->x[3] + r_hat[1]*ekf->x[4] + r_hat[2]*ekf->x[5];
    double innov  = v_radial_meas - z_pred;

    double R_dop = sigma_radial * sigma_radial;

    /*
     * S = H @ P @ H^T + R  (scalar)
     * H @ P = sum over j: r_hat[j-3] * P[3:6, :]  for j in {3,4,5}
     * H @ P @ H^T = sum_{i,j in {3,4,5}} r_hat[i-3] * P[i][j] * r_hat[j-3]
     */
    double S_scalar = R_dop;
    for (int i = 3; i < 6; i++)
        for (int j = 3; j < 6; j++)
            S_scalar += r_hat[i-3] * ekf->P[i][j] * r_hat[j-3];

    if (S_scalar < 1e-30) return 0;

    /* Mahalanobis gate */
    double mahal = innov * innov / S_scalar;
    if (mahal > THEKF_DOPPLER_GATE_SIGMA2) return 0;

    /*
     * K = P @ H^T / S  (6×1)
     * P @ H^T = sum over j in {3,4,5}: P[:,j] * r_hat[j-3]
     */
    double K[6];
    for (int i = 0; i < 6; i++) {
        K[i] = 0.0;
        for (int j = 3; j < 6; j++)
            K[i] += ekf->P[i][j] * r_hat[j-3];
        K[i] /= S_scalar;
    }

    /* K_vel: zero the position rows (rows 0-2) — velocity-only correction.
     * This prevents cross-term explosion at close range (see Python docstring).
     */
    double K_vel[6];
    for (int i = 0; i < 6; i++)
        K_vel[i] = (i >= 3) ? K[i] : 0.0;

    /* x += K_vel * innov */
    for (int i = 0; i < 6; i++)
        ekf->x[i] += K_vel[i] * innov;

    /*
     * Joseph-form covariance update using K_vel and H (1×6).
     * IKH = I - K_vel @ H   (6×6)
     * P = IKH @ P @ IKH^T + K_vel * R_dop * K_vel^T
     */
    /* Build H (1×6) then pad to 3×6 so we can reuse _joseph_update_3.
     * Instead, do it inline for the scalar case. */
    double IKH[6][6], I6[6][6];
    MAT_EYE(I6, 6);
    for (int i = 0; i < 6; i++)
        for (int j = 0; j < 6; j++) {
            /* K_vel @ H: (K_vel[i]) * H[j]  where H[j] = r_hat[j-3] for j>=3, else 0 */
            double kh = K_vel[i] * ((j >= 3) ? r_hat[j-3] : 0.0);
            IKH[i][j] = I6[i][j] - kh;
        }

    double IKHP[6][6], IKHT[6][6], IKHPIKHT[6][6];
    MAT_MUL(IKHP, IKH, ekf->P, 6, 6, 6);
    MAT_T(IKHT, IKH, 6, 6);
    MAT_MUL(IKHPIKHT, IKHP, IKHT, 6, 6, 6);

    /* K_vel * R_dop * K_vel^T (outer product scaled by R_dop) */
    double KRKT[6][6];
    for (int i = 0; i < 6; i++)
        for (int j = 0; j < 6; j++)
            KRKT[i][j] = K_vel[i] * R_dop * K_vel[j];

    MAT_ADD(ekf->P, IKHPIKHT, KRKT, 6, 6);
    MAT_SYM(ekf->P, 6);

    /* PSD floor */
    for (int i = 0; i < 6; i++) {
        if (ekf->P[i][i] < 0.0) {
            double offset = -ekf->P[i][i] + 1e-12;
            for (int k = 0; k < 6; k++)
                ekf->P[k][k] += offset;
            break;
        }
    }

    return 1;
}

/* ── THEKF_inflate_process_noise ─────────────────────────────── */
void THEKF_inflate_process_noise(THEKF_State *ekf, double scale) {
    /*
     * Adds scale * Q[0][0] to P[0:3, 0:3] diagonal.
     * Mirrors Python: self.P[0:3, 0:3] += np.eye(3) * (self.Q[0, 0] * scale)
     */
    double add = ekf->Q[0][0] * scale;
    for (int i = 0; i < 3; i++)
        ekf->P[i][i] += add;
}
