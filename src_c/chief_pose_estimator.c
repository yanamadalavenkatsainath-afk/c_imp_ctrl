/**
 * chief_pose_estimator.c — Chief Pose EKF (6-state attitude + omega)
 * ===================================================================
 * Port of chief_pose_estimator.py.
 *
 * SVD is required for EPnP.  We use a Jacobi one-sided SVD for 3x3
 * and a Golub-Reinsch bidiagonal SVD for the (2M x 12) EPnP system.
 * Both are stack-allocated with fixed upper bounds.
 * No LAPACK / BLAS dependency.
 *
 * EPnP reference: Lepetit, Moreno-Noguer, Fua, IJCV 2009.
 */

#include "chief_pose_estimator.h"
#include <math.h>
#include <string.h>
#include <stdlib.h>   /* rand(), RAND_MAX */
#include <stdio.h>

/* ── Compile-time limits ──────────────────────────────────────── */
#define CPE_MAX_PTS      CPE_MAX_MODEL_PTS
#define CPE_MAX_VIS      CPE_MAX_MODEL_PTS
/* EPnP linear system: 2*MAX_VIS rows × 12 cols */
#define CPE_L_ROWS      (2 * CPE_MAX_VIS)
#define CPE_L_COLS      12

/* ── Gaussian noise via Box-Muller (no libm randn) ──────────── */
static double _randn(void) {
    /* Box-Muller — uses two uniform samples */
    double u1 = ((double)rand() + 1.0) / ((double)RAND_MAX + 2.0);
    double u2 = ((double)rand() + 1.0) / ((double)RAND_MAX + 2.0);
    return sqrt(-2.0 * log(u1)) * cos(2.0 * M_PI * u2);
}

/* ── Quaternion helpers ───────────────────────────────────────── */

static void _qnorm(double q[4]) {
    double n = sqrt(q[0]*q[0]+q[1]*q[1]+q[2]*q[2]+q[3]*q[3]);
    if (n < 1e-12) { q[0]=1.0; q[1]=q[2]=q[3]=0.0; return; }
    q[0]/=n; q[1]/=n; q[2]/=n; q[3]/=n;
}

/* Hamilton product: out = a * b  (w-first) */
static void _qmul(const double a[4], const double b[4], double out[4]) {
    out[0] = a[0]*b[0] - a[1]*b[1] - a[2]*b[2] - a[3]*b[3];
    out[1] = a[0]*b[1] + a[1]*b[0] + a[2]*b[3] - a[3]*b[2];
    out[2] = a[0]*b[2] - a[1]*b[3] + a[2]*b[0] + a[3]*b[1];
    out[3] = a[0]*b[3] + a[1]*b[2] - a[2]*b[1] + a[3]*b[0];
}

/* Rotation matrix from quaternion [w,x,y,z] → 3x3 (body→frame) */
static void _q2R(const double q[4], double R[3][3]) {
    double w=q[0],x=q[1],y=q[2],z=q[3];
    R[0][0]=1-2*(y*y+z*z); R[0][1]=2*(x*y-w*z);   R[0][2]=2*(x*z+w*y);
    R[1][0]=2*(x*y+w*z);   R[1][1]=1-2*(x*x+z*z); R[1][2]=2*(y*z-w*x);
    R[2][0]=2*(x*z-w*y);   R[2][1]=2*(y*z+w*x);   R[2][2]=1-2*(x*x+y*y);
}

/* Rotation matrix → quaternion [w,x,y,z] — Shepperd method */
static void _R2q(const double R[3][3], double q[4]) {
    double trace = R[0][0] + R[1][1] + R[2][2];
    if (trace > 0.0) {
        double s = 0.5 / sqrt(trace + 1.0);
        q[0] = 0.25 / s;
        q[1] = (R[2][1] - R[1][2]) * s;
        q[2] = (R[0][2] - R[2][0]) * s;
        q[3] = (R[1][0] - R[0][1]) * s;
    } else if (R[0][0] > R[1][1] && R[0][0] > R[2][2]) {
        double s = 2.0 * sqrt(1.0 + R[0][0] - R[1][1] - R[2][2]);
        q[0] = (R[2][1] - R[1][2]) / s;
        q[1] = 0.25 * s;
        q[2] = (R[0][1] + R[1][0]) / s;
        q[3] = (R[0][2] + R[2][0]) / s;
    } else if (R[1][1] > R[2][2]) {
        double s = 2.0 * sqrt(1.0 + R[1][1] - R[0][0] - R[2][2]);
        q[0] = (R[0][2] - R[2][0]) / s;
        q[1] = (R[0][1] + R[1][0]) / s;
        q[2] = 0.25 * s;
        q[3] = (R[1][2] + R[2][1]) / s;
    } else {
        double s = 2.0 * sqrt(1.0 + R[2][2] - R[0][0] - R[1][1]);
        q[0] = (R[1][0] - R[0][1]) / s;
        q[1] = (R[0][2] + R[2][0]) / s;
        q[2] = (R[1][2] + R[2][1]) / s;
        q[3] = 0.25 * s;
    }
    _qnorm(q);
}

static void _reset_pose_covariance(CPE_State *s, double att_deg, double omega_deg_s) {
    memset(s->P, 0, sizeof(s->P));
    double sig_att = att_deg * M_PI / 180.0;
    double sig_w = omega_deg_s * M_PI / 180.0;
    for (int i=0;i<3;i++) s->P[i][i] = sig_att * sig_att;
    for (int i=3;i<6;i++) s->P[i][i] = sig_w * sig_w;
}

static void _reset_debug(CPE_State *s) {
    s->status = 0;
    s->visible_count = 0;
    s->visible_mask = 0ULL;
    s->stub_visible = 0;
    s->reproj_rms_px = NAN;
    s->pca_cond = NAN;
}

/* ── 3x3 matrix helpers ───────────────────────────────────────── */

static void _mat3_mul(const double A[3][3], const double B[3][3], double C[3][3]) {
    for (int i=0;i<3;i++) for (int j=0;j<3;j++) {
        C[i][j]=0.0;
        for (int k=0;k<3;k++) C[i][j]+=A[i][k]*B[k][j];
    }
}

static void _mat3_T(const double A[3][3], double B[3][3]) {
    for (int i=0;i<3;i++) for (int j=0;j<3;j++) B[j][i]=A[i][j];
}

static void _mat3_vec(const double A[3][3], const double v[3], double out[3]) {
    for (int i=0;i<3;i++) { out[i]=0.0; for(int j=0;j<3;j++) out[i]+=A[i][j]*v[j]; }
}

static double _norm3(const double v[3]) {
    return sqrt(v[0]*v[0]+v[1]*v[1]+v[2]*v[2]);
}

static double _dot3(const double a[3], const double b[3]) {
    return a[0]*b[0]+a[1]*b[1]+a[2]*b[2];
}

static void _cross3(const double a[3], const double b[3], double out[3]) {
    out[0]=a[1]*b[2]-a[2]*b[1];
    out[1]=a[2]*b[0]-a[0]*b[2];
    out[2]=a[0]*b[1]-a[1]*b[0];
}

/* ── 3x3 Jacobi SVD (for Procrustes step) ────────────────────── */
/*
 * Computes A = U * diag(S) * Vt for a 3x3 matrix A.
 * S values may be negative — caller handles sign for det correction.
 * We only need V^T and U for the Procrustes step so we compute both.
 */

static void _svd3(double A[3][3], double U[3][3], double S[3], double Vt[3][3]) {
    /* One-sided Jacobi on A^T A to get V, then U = A V S^{-1} */
    double AtA[3][3], V[3][3];
    /* AtA = A^T A */
    for (int i=0;i<3;i++) for (int j=0;j<3;j++) {
        AtA[i][j]=0.0;
        for (int k=0;k<3;k++) AtA[i][j]+=A[k][i]*A[k][j];
    }
    /* V = I */
    for (int i=0;i<3;i++) for (int j=0;j<3;j++) V[i][j]=(i==j)?1.0:0.0;

    for (int iter=0;iter<50;iter++) {
        double off=0.0;
        for (int i=0;i<3;i++) for (int j=i+1;j<3;j++) off+=AtA[i][j]*AtA[i][j];
        if (off < 1e-28) break;
        for (int p=0;p<3;p++) for (int q_=p+1;q_<3;q_++) {
            double theta=0.5*(AtA[q_][q_]-AtA[p][p])/(AtA[p][q_]+1e-300);
            double t=1.0/(fabs(theta)+sqrt(1.0+theta*theta));
            if (theta<0.0) t=-t;
            double c=1.0/sqrt(1.0+t*t), s=t*c;
            /* Apply rotation to AtA and V */
            for (int i=0;i<3;i++) {
                double aip=AtA[i][p], aiq=AtA[i][q_];
                AtA[i][p]=c*aip-s*aiq; AtA[i][q_]=s*aip+c*aiq;
            }
            for (int j=0;j<3;j++) {
                double apj=AtA[p][j], aqj=AtA[q_][j];
                AtA[p][j]=c*apj-s*aqj; AtA[q_][j]=s*apj+c*aqj;
            }
            for (int i=0;i<3;i++) {
                double vip=V[i][p], viq=V[i][q_];
                V[i][p]=c*vip-s*viq; V[i][q_]=s*vip+c*viq;
            }
        }
    }
    /* Singular values = sqrt of eigenvalues of A^T A */
    for (int i=0;i<3;i++) S[i]=sqrt(fabs(AtA[i][i]));

    /* U = A V S^{-1} */
    for (int i=0;i<3;i++) for (int j=0;j<3;j++) {
        U[i][j]=0.0;
        if (S[j]>1e-12) { for (int k=0;k<3;k++) U[i][j]+=A[i][k]*V[k][j]; U[i][j]/=S[j]; }
    }

    /* Vt = V^T */
    for (int i=0;i<3;i++) for (int j=0;j<3;j++) Vt[i][j]=V[j][i];
}

/* ── Thin SVD of (2M x 12) EPnP matrix — last right sing. vec ── */
/*
 * We only need the last right singular vector (null-space of L).
 * Use power iteration on L^T L restricted to 12x12.
 * This is sufficient because we want the smallest singular value's
 * right vector, which is the eigenvector of L^T L with smallest eigenvalue.
 * We compute ALL eigenvectors via Jacobi on LtL, then pick smallest.
 */

#define SVD12_N 12

static void _epnp_nullspace(const double L[CPE_L_ROWS][CPE_L_COLS],
                             int rows,
                             double null_vec[CPE_L_COLS]) {
    /* Build L^T L (12x12) */
    double LtL[SVD12_N][SVD12_N];
    for (int i=0;i<SVD12_N;i++) for (int j=0;j<SVD12_N;j++) {
        LtL[i][j]=0.0;
        for (int k=0;k<rows;k++) LtL[i][j]+=L[k][i]*L[k][j];
    }

    /* Jacobi eigendecomposition on LtL */
    double V12[SVD12_N][SVD12_N];
    for (int i=0;i<SVD12_N;i++) for (int j=0;j<SVD12_N;j++)
        V12[i][j]=(i==j)?1.0:0.0;

    for (int iter=0;iter<200;iter++) {
        double off=0.0;
        for (int i=0;i<SVD12_N;i++) for (int j=i+1;j<SVD12_N;j++)
            off+=LtL[i][j]*LtL[i][j];
        if (off<1e-20) break;
        for (int p=0;p<SVD12_N;p++) for (int q_=p+1;q_<SVD12_N;q_++) {
            if (fabs(LtL[p][q_])<1e-14) continue;
            double theta=0.5*(LtL[q_][q_]-LtL[p][p])/(LtL[p][q_]+1e-300);
            double t=1.0/(fabs(theta)+sqrt(1.0+theta*theta));
            if (theta<0.0) t=-t;
            double c=1.0/sqrt(1.0+t*t), s=t*c;
            for (int i=0;i<SVD12_N;i++) {
                double aip=LtL[i][p], aiq=LtL[i][q_];
                LtL[i][p]=c*aip-s*aiq; LtL[i][q_]=s*aip+c*aiq;
            }
            for (int j=0;j<SVD12_N;j++) {
                double apj=LtL[p][j], aqj=LtL[q_][j];
                LtL[p][j]=c*apj-s*aqj; LtL[q_][j]=s*apj+c*aqj;
            }
            for (int i=0;i<SVD12_N;i++) {
                double vip=V12[i][p], viq=V12[i][q_];
                V12[i][p]=c*vip-s*viq; V12[i][q_]=s*vip+c*viq;
            }
        }
    }
    /* Find index of smallest eigenvalue */
    int min_idx=0;
    double min_val=LtL[0][0];
    for (int i=1;i<SVD12_N;i++) if (LtL[i][i]<min_val){ min_val=LtL[i][i]; min_idx=i; }

    for (int i=0;i<SVD12_N;i++) null_vec[i]=V12[i][min_idx];
}

/* ── 4x4 linear system solver (for barycentric coords) ─────── */
/* Gauss elimination with partial pivoting */
static int _solve4x4(double A[4][4], double b[4], double x[4]) {
    /* Augmented [A|b] */
    double M[4][5];
    for (int i=0;i<4;i++) { for(int j=0;j<4;j++) M[i][j]=A[i][j]; M[i][4]=b[i]; }
    for (int col=0;col<4;col++) {
        /* Pivot */
        int piv=col;
        for (int r=col+1;r<4;r++) if(fabs(M[r][col])>fabs(M[piv][col])) piv=r;
        if (fabs(M[piv][col])<1e-14) return -1;
        double tmp[5]; memcpy(tmp,M[col],40); memcpy(M[col],M[piv],40); memcpy(M[piv],tmp,40);
        double inv=1.0/M[col][col];
        for (int j=col;j<5;j++) M[col][j]*=inv;
        for (int r=0;r<4;r++) if(r!=col){
            double f=M[r][col];
            for(int j=col;j<5;j++) M[r][j]-=f*M[col][j];
        }
    }
    for (int i=0;i<4;i++) x[i]=M[i][4];
    return 0;
}

/* ── EPnP orientation estimator — matches Python _estimate_orientation ── */

static int _estimate_orientation(CPE_State *s,
                                  const double dr_lvlh[3],
                                  const double q_chief[4],
                                  double R_out[3][3]) {
    double true_range = _norm3(dr_lvlh);
    if (true_range < s->cam.min_range || true_range > s->cam.max_range)
        return 0;

    /* ── Build camera frame (same as Python) ─────────────────── */
    double r_hat[3];
    for (int i=0;i<3;i++) r_hat[i]=dr_lvlh[i]/true_range;
    double world_up[3]={0.,0.,1.};
    if (fabs(_dot3(r_hat,world_up))>0.99){ world_up[0]=0.; world_up[1]=1.; world_up[2]=0.; }
    double cam_X[3], cam_Y[3];
    _cross3(world_up, r_hat, cam_X);
    double nX=_norm3(cam_X); if(nX<1e-12) return 0;
    for(int i=0;i<3;i++) cam_X[i]/=nX;
    _cross3(r_hat, cam_X, cam_Y);
    /* R_l2c rows = [cam_X, cam_Y, r_hat] */
    double R_l2c[3][3];
    for(int j=0;j<3;j++){ R_l2c[0][j]=cam_X[j]; R_l2c[1][j]=cam_Y[j]; R_l2c[2][j]=r_hat[j]; }

    /* ── Project model points ─────────────────────────────────── */
    double R_b2l[3][3]; _q2R(q_chief, R_b2l);
    double dr_cam[3]; _mat3_vec(R_l2c, dr_lvlh, dr_cam);

    /* Visible points */
    double px_obs[CPE_MAX_VIS][2];
    double pts_body[CPE_MAX_VIS][3];
    int M=0;

    for (int i=0; i<s->cam.n_model_pts && M<CPE_MAX_VIS; i++) {
        /* pt_body → pt_cam = R_l2c @ R_b2l @ pt_body + dr_cam */
        double tmp[3], tmp2[3];
        _mat3_vec(R_b2l, s->cam.model_pts[i], tmp);
        _mat3_vec(R_l2c, tmp, tmp2);
        double Pc[3];
        for(int k=0;k<3;k++) Pc[k]=tmp2[k]+dr_cam[k];
        double Z=Pc[2];
        if (Z<=0.01) continue;
        double u=s->cam.f*Pc[0]/Z+s->cam.cx;
        double v=s->cam.f*Pc[1]/Z+s->cam.cy;
        if (u<0||u>=s->cam.W||v<0||v>=s->cam.H) continue;
        /* Add pixel noise */
        px_obs[M][0]=u+_randn()*s->cam.sigma_px;
        px_obs[M][1]=v+_randn()*s->cam.sigma_px;
        for(int k=0;k<3;k++) pts_body[M][k]=s->cam.model_pts[i][k];
        if (i < 63) s->visible_mask |= (1ULL << (unsigned)i);
        if (i == 8 || i == 9) s->stub_visible = 1;
        M++;
    }
    s->visible_count = M;
    if (M<4) { s->status = 4; return 0; }

    /* ── EPnP step 1: centroid + PCA (simplified — use centroid + axis-aligned) */
    double c0[3]={0.,0.,0.};
    for(int i=0;i<M;i++) for(int k=0;k<3;k++) c0[k]+=pts_body[i][k];
    for(int k=0;k<3;k++) c0[k]/=(double)M;

    /* Build 3xM de-meaned matrix for PCA via 3x3 scatter */
    double Sc[3][3]={{0}};
    for(int i=0;i<M;i++){
        double d[3]; for(int k=0;k<3;k++) d[k]=pts_body[i][k]-c0[k];
        for(int a=0;a<3;a++) for(int b=0;b<3;b++) Sc[a][b]+=d[a]*d[b];
    }
    /* Eigendecomposition of Sc (3x3 Jacobi) to get principal axes */
    double Vsc[3][3];
    for(int i=0;i<3;i++) for(int j=0;j<3;j++) Vsc[i][j]=(i==j)?1.0:0.0;
    for(int iter=0;iter<30;iter++){
        for(int p=0;p<3;p++) for(int q_=p+1;q_<3;q_++){
            if(fabs(Sc[p][q_])<1e-16) continue;
            double theta=0.5*(Sc[q_][q_]-Sc[p][p])/(Sc[p][q_]+1e-300);
            double t=1.0/(fabs(theta)+sqrt(1.0+theta*theta)); if(theta<0.)t=-t;
            double c=1.0/sqrt(1.0+t*t),s2=t*c;
            for(int i=0;i<3;i++){double a=Sc[i][p],b=Sc[i][q_];Sc[i][p]=c*a-s2*b;Sc[i][q_]=s2*a+c*b;}
            for(int j=0;j<3;j++){double a=Sc[p][j],b=Sc[q_][j];Sc[p][j]=c*a-s2*b;Sc[q_][j]=s2*a+c*b;}
            for(int i=0;i<3;i++){double a=Vsc[i][p],b=Vsc[i][q_];Vsc[i][p]=c*a-s2*b;Vsc[i][q_]=s2*a+c*b;}
        }
    }
    double sv[3]={Sc[0][0],Sc[1][1],Sc[2][2]};
    double sv_max=fabs(sv[0]), sv_min=fabs(sv[0]);
    for(int i=1;i<3;i++){
        double a=fabs(sv[i]);
        if(a>sv_max) sv_max=a;
        if(a<sv_min) sv_min=a;
    }
    s->pca_cond = sv_max / (sv_min > 1e-12 ? sv_min : 1e-12);
    /* 4 control points: c0, c0 + v0*sqrt(sv0/M), ... */
    double ctrl_body[4][3];
    for(int k=0;k<3;k++) ctrl_body[0][k]=c0[k];
    for(int ax=0;ax<3;ax++){
        double scale=sqrt(fabs(sv[ax])/(double)M);
        for(int k=0;k<3;k++) ctrl_body[ax+1][k]=c0[k]+Vsc[k][ax]*scale;
    }

    /* ── EPnP step 2: barycentric coords of each point ─────── */
    /* ctrl_body is 4x3; we need 4x4 augmented: [ctrl_body | 1] */
    double ctrl_aug[4][4];
    for(int i=0;i<4;i++){for(int j=0;j<3;j++) ctrl_aug[i][j]=ctrl_body[i][j]; ctrl_aug[i][3]=1.0;}
    /* alphas[i] = ctrl_aug^{-T} @ [pts_body[i], 1]
       i.e. solve ctrl_aug^T @ alpha = [p; 1] */
    double alphas[CPE_MAX_VIS][4];
    for(int i=0;i<M;i++){
        double At[4][4], rhs[4], sol[4];
        for(int r=0;r<4;r++) for(int c=0;c<4;c++) At[r][c]=ctrl_aug[c][r];
        for(int k=0;k<3;k++) rhs[k]=pts_body[i][k]; rhs[3]=1.0;
        if(_solve4x4(At,rhs,sol)!=0) return 0;
        for(int k=0;k<4;k++) alphas[i][k]=sol[k];
    }

    /* ── EPnP step 3: build 2M x 12 linear system ──────────── */
    double L[CPE_L_ROWS][CPE_L_COLS];
    memset(L,0,sizeof(L));
    int rows=2*M; if(rows>CPE_L_ROWS) rows=CPE_L_ROWS;
    double f=s->cam.f, cx=s->cam.cx, cy=s->cam.cy;
    for(int i=0;i<M && 2*i+1<CPE_L_ROWS;i++){
        double ui=px_obs[i][0], vi=px_obs[i][1];
        for(int j=0;j<4;j++){
            double a=alphas[i][j];
            L[2*i][   3*j    ]= f*a;
            L[2*i][   3*j+2  ]=(cx-ui)*a;
            L[2*i+1][ 3*j+1  ]= f*a;
            L[2*i+1][ 3*j+2  ]=(cy-vi)*a;
        }
    }

    /* ── EPnP step 4: null-space of L → x_est (12-vector) ─── */
    double x_est[CPE_L_COLS];
    _epnp_nullspace(L, rows, x_est);

    /* Reshape x_est → ctrl_cam (4x3) */
    double ctrl_cam[4][3];
    for(int i=0;i<4;i++) for(int j=0;j<3;j++) ctrl_cam[i][j]=x_est[3*i+j];

    /* Enforce positive depth */
    if(ctrl_cam[0][2]<0.0) for(int i=0;i<4;i++) for(int j=0;j<3;j++) ctrl_cam[i][j]=-ctrl_cam[i][j];

    /* ── EPnP step 5: Procrustes — ctrl_body → ctrl_cam ────── */
    double c_body[3]={0.}, c_cam[3]={0.};
    for(int i=0;i<4;i++) for(int k=0;k<3;k++){ c_body[k]+=ctrl_body[i][k]; c_cam[k]+=ctrl_cam[i][k]; }
    for(int k=0;k<3;k++){ c_body[k]/=4.0; c_cam[k]/=4.0; }

    /* A = (ctrl_cam - c_cam)^T @ (ctrl_body - c_body)  (3x3) */
    double A_proc[3][3]={{0}};
    for(int i=0;i<4;i++){
        double db[3], dc[3];
        for(int k=0;k<3;k++){ db[k]=ctrl_body[i][k]-c_body[k]; dc[k]=ctrl_cam[i][k]-c_cam[k]; }
        for(int r=0;r<3;r++) for(int c2=0;c2<3;c2++) A_proc[r][c2]+=dc[r]*db[c2];
    }

    double U_p[3][3], S_p[3], Vt_p[3][3];
    _svd3(A_proc, U_p, S_p, Vt_p);

    /* det correction */
    double R_b2c_tmp[3][3], VU[3][3];
    double Vt_T[3][3];
    _mat3_T(Vt_p, Vt_T);
    _mat3_mul(U_p, Vt_T, VU);
    double det_VU = VU[0][0]*(VU[1][1]*VU[2][2]-VU[1][2]*VU[2][1])
                  - VU[0][1]*(VU[1][0]*VU[2][2]-VU[1][2]*VU[2][0])
                  + VU[0][2]*(VU[1][0]*VU[2][1]-VU[1][1]*VU[2][0]);
    double diag[3][3]={{1.,0.,0.},{0.,1.,0.},{0.,0.,det_VU>0?1.:-1.}};
    double tmp_m[3][3];
    _mat3_mul(U_p, diag, tmp_m);
    _mat3_mul(tmp_m, Vt_p, R_b2c_tmp);   /* R_body2cam */

    /* R_body2lvlh = R_l2c^T @ R_body2cam */
    double R_l2c_T[3][3]; _mat3_T(R_l2c, R_l2c_T);
    _mat3_mul(R_l2c_T, R_b2c_tmp, R_out);

    double err2_sum=0.0;
    int err_n=0;
    for(int i=0;i<M;i++){
        double pc_rot[3];
        _mat3_vec(R_b2c_tmp, pts_body[i], pc_rot);
        for(int k=0;k<3;k++) pc_rot[k] += dr_cam[k];
        if(pc_rot[2] <= 0.01) continue;
        double u = s->cam.f * pc_rot[0] / pc_rot[2] + s->cam.cx;
        double v = s->cam.f * pc_rot[1] / pc_rot[2] + s->cam.cy;
        double du = u - px_obs[i][0];
        double dv = v - px_obs[i][1];
        err2_sum += du*du + dv*dv;
        err_n++;
    }
    s->reproj_rms_px = (err_n > 0) ? sqrt(err2_sum / (double)err_n) : NAN;
    return 1;
}

/* ── EKF predict ─────────────────────────────────────────────── */

static void _predict(CPE_State *s) {
    double dt=s->dt;
    double wx=s->omega[0], wy=s->omega[1], wz=s->omega[2];

    /* Quaternion kinematics: q += 0.5*dt*Omega(w)*q
     * Omega = 0.5*[[0,-wx,-wy,-wz],[wx,0,wz,-wy],[wy,-wz,0,wx],[wz,wy,-wx,0]]
     * (q_dot = 0.5 * Omega @ q) — matches Python exactly. */
    double Omega_q2[4] = {
        0.5*( -wx*s->q[1] - wy*s->q[2] - wz*s->q[3]),
        0.5*(  wx*s->q[0] + wz*s->q[2] - wy*s->q[3]),
        0.5*(  wy*s->q[0] - wz*s->q[1] + wx*s->q[3]),
        0.5*(  wz*s->q[0] + wy*s->q[1] - wx*s->q[2])
    };
    for(int i=0;i<4;i++) s->q[i]+=dt*Omega_q2[i];
    _qnorm(s->q);

    /* Skew-symmetric of omega */
    double skew_w[3][3]={{0,-wz,wy},{wz,0,-wx},{-wy,wx,0}};

    /* F (6x6): F[0:3,0:3]=-skew_w, F[0:3,3:6]=I, rest=0 */
    /* Phi = I + F*dt */
    double Phi[6][6];
    memset(Phi,0,sizeof(Phi));
    for(int i=0;i<6;i++) Phi[i][i]=1.0;
    for(int i=0;i<3;i++) for(int j=0;j<3;j++) Phi[i][j]+=-skew_w[i][j]*dt;
    for(int i=0;i<3;i++) Phi[i][i+3]+=dt;

    /* P = Phi @ P @ Phi^T + Q */
    double PhiP[6][6], PhiT[6][6], PhiPPhiT[6][6];
    for(int i=0;i<6;i++) for(int j=0;j<6;j++){
        PhiP[i][j]=0.0; for(int k=0;k<6;k++) PhiP[i][j]+=Phi[i][k]*s->P[k][j];
    }
    for(int i=0;i<6;i++) for(int j=0;j<6;j++) PhiT[j][i]=Phi[i][j];
    for(int i=0;i<6;i++) for(int j=0;j<6;j++){
        PhiPPhiT[i][j]=0.0; for(int k=0;k<6;k++) PhiPPhiT[i][j]+=PhiP[i][k]*PhiT[k][j];
    }
    double sum_diag=0.0;
    for(int i=0;i<6;i++) sum_diag+=PhiPPhiT[i][i]+s->Q[i][i];
    if(sum_diag>1e6 || !isfinite(sum_diag)){
        /* Divergence guard — reset P and omega */
        memset(s->P,0,sizeof(s->P));
        for(int i=0;i<3;i++) s->P[i][i]=(30.0*M_PI/180.0)*(30.0*M_PI/180.0);
        for(int i=3;i<6;i++) s->P[i][i]=(5.0*M_PI/180.0)*(5.0*M_PI/180.0);
        s->omega[0]=s->omega[1]=s->omega[2]=0.0;
        s->valid=0; s->update_count=0;
        return;
    }
    for(int i=0;i<6;i++) for(int j=0;j<6;j++)
        s->P[i][j]=PhiPPhiT[i][j]+s->Q[i][j];
    /* Symmetrise */
    for(int i=0;i<6;i++) for(int j=i+1;j<6;j++){
        double avg=0.5*(s->P[i][j]+s->P[j][i]);
        s->P[i][j]=avg; s->P[j][i]=avg;
    }
}

/* ── EKF update from rotation matrix ─────────────────────────── */

static int _update(CPE_State *s, const double R_meas[3][3], const double R_noise[3][3]) {
    double R_est[3][3]; _q2R(s->q, R_est);

    /* Innovation: R_err = R_meas @ R_est^T */
    double R_est_T[3][3]; _mat3_T(R_est, R_est_T);
    double R_err[3][3]; _mat3_mul(R_meas, R_est_T, R_err);

    /* Convert to rotation vector (axis-angle) */
    double cos_a=(R_err[0][0]+R_err[1][1]+R_err[2][2]-1.0)/2.0;
    if(cos_a>1.) cos_a=1.; if(cos_a<-1.) cos_a=-1.;
    double angle=acos(cos_a);
    double z_err[3]={0.,0.,0.};
    if(angle>1e-8){
        double s2=2.0*sin(angle)+1e-12;
        z_err[0]=(R_err[2][1]-R_err[1][2])/s2*angle;
        z_err[1]=(R_err[0][2]-R_err[2][0])/s2*angle;
        z_err[2]=(R_err[1][0]-R_err[0][1])/s2*angle;
    }

    /* H = [I3 | 0] (3x6) */
    /* S = H P H^T + R = P[0:3,0:3] + R_noise */
    double S[3][3];
    for(int i=0;i<3;i++) for(int j=0;j<3;j++) S[i][j]=s->P[i][j]+R_noise[i][j];

    /* S inverse (3x3) */
    double det=S[0][0]*(S[1][1]*S[2][2]-S[1][2]*S[2][1])
              -S[0][1]*(S[1][0]*S[2][2]-S[1][2]*S[2][0])
              +S[0][2]*(S[1][0]*S[2][1]-S[1][1]*S[2][0]);
    if(fabs(det)<1e-30) return 0;
    double inv=1.0/det;
    double S_inv[3][3];
    S_inv[0][0]= inv*(S[1][1]*S[2][2]-S[1][2]*S[2][1]);
    S_inv[0][1]=-inv*(S[0][1]*S[2][2]-S[0][2]*S[2][1]);
    S_inv[0][2]= inv*(S[0][1]*S[1][2]-S[0][2]*S[1][1]);
    S_inv[1][0]=-inv*(S[1][0]*S[2][2]-S[1][2]*S[2][0]);
    S_inv[1][1]= inv*(S[0][0]*S[2][2]-S[0][2]*S[2][0]);
    S_inv[1][2]=-inv*(S[0][0]*S[1][2]-S[0][2]*S[1][0]);
    S_inv[2][0]= inv*(S[1][0]*S[2][1]-S[1][1]*S[2][0]);
    S_inv[2][1]=-inv*(S[0][0]*S[2][1]-S[0][1]*S[2][0]);
    S_inv[2][2]= inv*(S[0][0]*S[1][1]-S[0][1]*S[1][0]);

    /* Mahalanobis gate */
    double Si_z[3];
    for(int i=0;i<3;i++){Si_z[i]=0.; for(int j=0;j<3;j++) Si_z[i]+=S_inv[i][j]*z_err[j];}
    double mah2=_dot3(z_err,Si_z);
    if(mah2>s->gate_k*s->gate_k) return 0;

    /* K = P H^T S^{-1} — H^T = [I;0] so P H^T = P[:,0:3] */
    double K[6][3];
    for(int i=0;i<6;i++) for(int j=0;j<3;j++){
        K[i][j]=0.0;
        for(int k=0;k<3;k++) K[i][j]+=s->P[i][k]*S_inv[k][j];
    }

    /* dx = K @ z_err */
    double dx[6];
    for(int i=0;i<6;i++){dx[i]=0.; for(int j=0;j<3;j++) dx[i]+=K[i][j]*z_err[j];}

    /* Quaternion multiplicative correction */
    double dq[4]={1.0, 0.5*dx[0], 0.5*dx[1], 0.5*dx[2]};
    double q_new[4]; _qmul(dq, s->q, q_new); _qnorm(q_new);
    for(int i=0;i<4;i++) s->q[i]=q_new[i];

    /* Omega correction */
    for(int i=0;i<3;i++) s->omega[i]+=dx[3+i];

    /* Joseph-form covariance: P = (I-KH)P(I-KH)^T + K R K^T */
    /* IKH[i][j] = delta(i,j) - K[i][j] for j<3, else delta(i,j) */
    double IKH[6][6]; memset(IKH,0,sizeof(IKH));
    for(int i=0;i<6;i++) IKH[i][i]=1.0;
    for(int i=0;i<6;i++) for(int j=0;j<3;j++) IKH[i][j]-=K[i][j];

    double IKHP[6][6], IKHT[6][6], IKHPIKHT[6][6];
    for(int i=0;i<6;i++) for(int j=0;j<6;j++){
        IKHP[i][j]=0.; for(int k=0;k<6;k++) IKHP[i][j]+=IKH[i][k]*s->P[k][j];
    }
    for(int i=0;i<6;i++) for(int j=0;j<6;j++) IKHT[j][i]=IKH[i][j];
    for(int i=0;i<6;i++) for(int j=0;j<6;j++){
        IKHPIKHT[i][j]=0.; for(int k=0;k<6;k++) IKHPIKHT[i][j]+=IKHP[i][k]*IKHT[k][j];
    }
    /* KRK^T */
    double KR[6][3], KT[3][6], KRKT[6][6];
    for(int i=0;i<6;i++) for(int j=0;j<3;j++){
        KR[i][j]=0.; for(int k=0;k<3;k++) KR[i][j]+=K[i][k]*R_noise[k][j];
    }
    for(int i=0;i<6;i++) for(int j=0;j<3;j++) KT[j][i]=K[i][j];
    for(int i=0;i<6;i++) for(int j=0;j<6;j++){
        KRKT[i][j]=0.; for(int k=0;k<3;k++) KRKT[i][j]+=KR[i][k]*KT[k][j];
    }
    for(int i=0;i<6;i++) for(int j=0;j<6;j++) s->P[i][j]=IKHPIKHT[i][j]+KRKT[i][j];
    /* Symmetrise */
    for(int i=0;i<6;i++) for(int j=i+1;j<6;j++){
        double avg=0.5*(s->P[i][j]+s->P[j][i]);
        s->P[i][j]=avg; s->P[j][i]=avg;
    }

    s->update_count++;
    return 1;
}

/* ── Public API ───────────────────────────────────────────────── */

void CPE_default_cam_params(CPE_CamParams *cam) {
    cam->f        = 800.0;
    cam->cx       = 320.0;   /* W/2 */
    cam->cy       = 240.0;   /* H/2 */
    cam->W        = 640.0;
    cam->H        = 480.0;
    cam->sigma_px = 1.5;
    cam->min_range = 0.05;
    cam->max_range = 5000.0;
    /* IS-1002 bus corners plus two solar-wing roots and one dock-face marker. */
    double corners[CPE_MAX_MODEL_PTS][3] = {
        { CFG_CHIEF_BODY_HALF_X_M,  CFG_CHIEF_BODY_HALF_Y_M,  CFG_CHIEF_BODY_HALF_Z_M},
        { CFG_CHIEF_BODY_HALF_X_M, -CFG_CHIEF_BODY_HALF_Y_M,  CFG_CHIEF_BODY_HALF_Z_M},
        {-CFG_CHIEF_BODY_HALF_X_M,  CFG_CHIEF_BODY_HALF_Y_M,  CFG_CHIEF_BODY_HALF_Z_M},
        {-CFG_CHIEF_BODY_HALF_X_M, -CFG_CHIEF_BODY_HALF_Y_M,  CFG_CHIEF_BODY_HALF_Z_M},
        { CFG_CHIEF_BODY_HALF_X_M,  CFG_CHIEF_BODY_HALF_Y_M, -CFG_CHIEF_BODY_HALF_Z_M},
        { CFG_CHIEF_BODY_HALF_X_M, -CFG_CHIEF_BODY_HALF_Y_M, -CFG_CHIEF_BODY_HALF_Z_M},
        {-CFG_CHIEF_BODY_HALF_X_M,  CFG_CHIEF_BODY_HALF_Y_M, -CFG_CHIEF_BODY_HALF_Z_M},
        {-CFG_CHIEF_BODY_HALF_X_M, -CFG_CHIEF_BODY_HALF_Y_M, -CFG_CHIEF_BODY_HALF_Z_M},
        { 1.50,  0.00,  0.00},
        {-1.50,  0.00,  0.00},
        { 0.00,  0.40,  CFG_CHIEF_BODY_HALF_Z_M},
    };
    for(int i=0;i<CPE_MAX_MODEL_PTS;i++) for(int j=0;j<3;j++) cam->model_pts[i][j]=corners[i][j];
    cam->n_model_pts = CPE_MAX_MODEL_PTS;
}

void CPE_init(CPE_State *s,
              const CPE_CamParams *cam,
              double dt,
              double sigma_omega_process,
              double sigma_pnp_deg,
              double gate_k) {
    memset(s, 0, sizeof(CPE_State));
    s->dt     = dt;
    s->gate_k = gate_k;
    s->max_reproj_rms_px = 50.0;
    s->pose_age_s = INFINITY;
    s->cam    = *cam;

    /* Random initial quaternion — unknown chief attitude */
    /* Use a fixed near-identity to avoid dependency on srand */
    s->q[0]=1.0; s->q[1]=s->q[2]=s->q[3]=0.0;

    /* P init: 30 deg attitude uncertainty, 5 deg/s omega uncertainty */
    _reset_pose_covariance(s, 30.0, 5.0);

    /* Q: small attitude diffusion + omega process noise */
    double sig_q_proc = 0.01*M_PI/180.0 * sqrt(dt);
    double sig_w_proc = sigma_omega_process * sqrt(dt);
    for(int i=0;i<3;i++) s->Q[i][i]=sig_q_proc*sig_q_proc;
    for(int i=3;i<6;i++) s->Q[i][i]=sig_w_proc*sig_w_proc;

    /* R_meas: PnP orientation noise */
    double sig_pnp = sigma_pnp_deg*M_PI/180.0;
    for(int i=0;i<3;i++) s->R_meas[i][i]=sig_pnp*sig_pnp;

    printf("  ChiefPoseEstimator (EKF): sigma_pnp=%.1fdeg  "
           "sigma_omega_proc=%.4fdeg/s/sqrt(s)  gate=%.0fsigma\n",
           sigma_pnp_deg,
           sigma_omega_process*180.0/M_PI,
           gate_k);
}

CPE_Result CPE_update(CPE_State *s,
                       const double dr_lvlh[3],
                       const double q_chief[4]) {
    /* Predict */
    _predict(s);
    if (isfinite(s->pose_age_s)) s->pose_age_s += s->dt;
    _reset_debug(s);

    if (s->pose_age_s > 60.0 && s->update_count >= 10) {
        double att_floor = (30.0*M_PI/180.0) * (30.0*M_PI/180.0);
        for(int i=0;i<3;i++) if(s->P[i][i] < att_floor) s->P[i][i] = att_floor;
    }

    /* Range-dependent R gain scheduling — matches Python exactly */
    double true_range = _norm3(dr_lvlh);
    if (true_range < 2.0) {
        _q2R(s->q, s->last_R_b2l);
        s->has_R_b2l = 1;
        s->status = 3;
        if (s->update_count >= 10) s->valid = 1;

        CPE_Result res;
        res.omega[0] = s->omega[0];
        res.omega[1] = s->omega[1];
        res.omega[2] = s->omega[2];
        res.valid    = s->valid;
        res.status   = s->status;
        res.pose_age_s = s->pose_age_s;
        return res;
    }
    double r_scale;
    if      (true_range < 5.0)  r_scale = 0.25 + 0.75*(true_range-2.0)/3.0;
    else if (true_range < 20.0) r_scale = 0.5;
    else                         r_scale = 1.0;

    double R_use[3][3];
    for(int i=0;i<3;i++) for(int j=0;j<3;j++)
        R_use[i][j] = s->R_meas[i][j] * r_scale;

    /* PnP measurement */
    double R_meas[3][3];
    int got_pnp = _estimate_orientation(s, dr_lvlh, q_chief, R_meas);
    if (got_pnp) {
        if (isfinite(s->reproj_rms_px) && s->reproj_rms_px > s->max_reproj_rms_px) {
            s->status = 6;
        } else if (s->pose_age_s > 10.0 &&
                   isfinite(s->reproj_rms_px) && s->reproj_rms_px < 10.0) {
            _R2q(R_meas, s->q);
            s->omega[0]=s->omega[1]=s->omega[2]=0.0;
            _reset_pose_covariance(s, 5.0, 3.0);
            for(int i=0;i<3;i++) for(int j=0;j<3;j++)
                s->last_R_b2l[i][j] = R_meas[i][j];
            s->has_R_b2l = 1;
            s->pose_age_s = 0.0;
            s->update_count++;
            s->status = 7;
        } else if (_update(s, R_meas, R_use)) {
            for(int i=0;i<3;i++) for(int j=0;j<3;j++)
                s->last_R_b2l[i][j] = R_meas[i][j];
            s->has_R_b2l = 1;
            s->pose_age_s = 0.0;
            s->status = 1;
        } else if (isfinite(s->reproj_rms_px) && s->reproj_rms_px < 5.0) {
            _R2q(R_meas, s->q);
            s->omega[0]=s->omega[1]=s->omega[2]=0.0;
            _reset_pose_covariance(s, 5.0, 3.0);
            for(int i=0;i<3;i++) for(int j=0;j<3;j++)
                s->last_R_b2l[i][j] = R_meas[i][j];
            s->has_R_b2l = 1;
            s->pose_age_s = 0.0;
            s->update_count++;
            s->status = 7;
        } else {
            s->status = 2;
        }
    }

    if (s->update_count >= 10) s->valid = 1;

    CPE_Result res;
    res.omega[0] = s->omega[0];
    res.omega[1] = s->omega[1];
    res.omega[2] = s->omega[2];
    res.valid    = s->valid;
    res.status   = s->status;
    res.pose_age_s = s->pose_age_s;
    return res;
}

CPE_Result CPE_update_rotation(CPE_State *s,
                                const double R_body2lvlh_meas[3][3],
                                double range_m) {
    _predict(s);
    if (isfinite(s->pose_age_s)) s->pose_age_s += s->dt;
    _reset_debug(s);

    if (s->pose_age_s > 60.0 && s->update_count >= 10) {
        double att_floor = (30.0*M_PI/180.0) * (30.0*M_PI/180.0);
        for(int i=0;i<3;i++) if(s->P[i][i] < att_floor) s->P[i][i] = att_floor;
    }

    double r = range_m > 0.0 ? range_m : 1e9;
    double r_scale;
    if      (r < 2.0)  r_scale = 0.05;
    else if (r < 5.0)  r_scale = 0.25 + 0.75*(r-2.0)/3.0;
    else if (r < 20.0) r_scale = 0.5;
    else               r_scale = 1.0;

    double R_use[3][3];
    for(int i=0;i<3;i++) for(int j=0;j<3;j++)
        R_use[i][j] = s->R_meas[i][j] * r_scale;

    int accepted = _update(s, R_body2lvlh_meas, R_use);
    if (!accepted && s->pose_age_s > 10.0) {
        _R2q(R_body2lvlh_meas, s->q);
        s->omega[0]=s->omega[1]=s->omega[2]=0.0;
        _reset_pose_covariance(s, 5.0, 3.0);
        accepted = 1;
        s->status = 7;
    } else {
        s->status = accepted ? 1 : 2;
    }

    if (accepted) {
        for(int i=0;i<3;i++) for(int j=0;j<3;j++)
            s->last_R_b2l[i][j] = R_body2lvlh_meas[i][j];
        s->has_R_b2l = 1;
        s->pose_age_s = 0.0;
        s->update_count++;
    }
    if (s->update_count >= 10) s->valid = 1;

    CPE_Result res;
    res.omega[0] = s->omega[0];
    res.omega[1] = s->omega[1];
    res.omega[2] = s->omega[2];
    res.valid = s->valid;
    res.status = s->status;
    res.pose_age_s = s->pose_age_s;
    return res;
}
