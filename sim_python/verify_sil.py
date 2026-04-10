"""
verify_sil.py — Python golden model vs C TH-EKF
================================================
Run from Satellite_GNC root:  python sim_python/verify_sil.py
  (or via build.bat which calls exactly this)

Root cause of previous failure:
  The flight sim uses a package layout:
    flight sim/estimation/th_ekf.py    ← actual file
  Not a flat layout:
    flight sim/th_ekf.py               ← does NOT exist

  So the import must be:
    from estimation.th_ekf import THEKF
  with FLIGHT_SIM (the parent of estimation/) on sys.path.
"""

import sys
import os

# ── Path setup ────────────────────────────────────────────────────
# FLIGHT_SIM: parent of estimation/, control/, etc.
# Adding it lets us do: from estimation.th_ekf import THEKF
FLIGHT_SIM = r"C:\Users\Venkat\OneDrive\Desktop\appex\flight sim"

# SIL_ROOT: the Satellite_GNC\ folder (parent of sim_python\)
# __file__ = Satellite_GNC\sim_python\verify_sil.py
# dirname(abspath(__file__)) = Satellite_GNC\sim_python
# dirname(...) = Satellite_GNC
SIL_ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))

# Insert SIL_ROOT first so 'from sim_python.wrapper import THEKF_C' resolves.
# Insert FLIGHT_SIM after so 'from estimation.th_ekf import THEKF' resolves.
# Using append instead of insert(0) for FLIGHT_SIM means SIL local modules
# always win, and flight sim modules are found as a fallback — clean separation.
if SIL_ROOT not in sys.path:
    sys.path.insert(0, SIL_ROOT)
if FLIGHT_SIM not in sys.path:
    sys.path.append(FLIGHT_SIM)

# ── Imports ───────────────────────────────────────────────────────
import numpy as np

# Golden model: Python THEKF from the flight sim.
# The flight sim stores it at:  flight sim/estimation/th_ekf.py
# With FLIGHT_SIM on sys.path the import is:
try:
    from estimation.th_ekf import THEKF as THEKF_Python
except ImportError as e:
    # Try direct import as fallback (flat layout)
    try:
        from th_ekf import THEKF as THEKF_Python
        print("[verify_sil] Used flat import fallback: from th_ekf import THEKF")
    except ImportError:
        print(f"\n  ERROR importing Python golden model: {e}")
        print(f"  Searched in sys.path:")
        for p in sys.path:
            print(f"    {p}")
        print(f"\n  Expected ONE of:")
        print(f"    {FLIGHT_SIM}\\estimation\\th_ekf.py  (package layout)")
        print(f"    {FLIGHT_SIM}\\th_ekf.py              (flat layout)")
        print(f"\n  Fix: check your FLIGHT_SIM path in verify_sil.py line 20")
        sys.exit(1)

# C wrapper — drop-in for THEKF
from sim_python.wrapper import THEKF_C

# ── Test parameters ───────────────────────────────────────────────
MU    = 3.986004418e14
A_GEO = 42164e3
E_GEO = 0.001
DT    = 10.0

PASS_POS_M    = 1e-4    # m   — position divergence threshold
PASS_VEL_MS   = 1e-7    # m/s — velocity divergence threshold
PASS_COV      = 1e-6    # —   — covariance element divergence threshold


# ── Run function (identical for both EKFs) ────────────────────────
def run(ekf, n_steps, x0):
    np.random.seed(42)
    P0 = np.diag([50.0**2]*3 + [0.5**2]*3)
    ekf.initialise(x0, P0=P0, nu0=0.0)

    R = np.diag([0.25,
                 np.radians(0.1)**2,
                 np.radians(0.1)**2])

    for _ in range(n_steps):
        ekf.predict()
        dr = ekf.x[:3].copy()
        r  = np.linalg.norm(dr)
        z  = np.array([
            r           + np.random.randn() * 0.3,
            np.arctan2(dr[1], dr[0])
                        + np.random.randn() * np.radians(0.05),
            np.arctan2(dr[2], np.linalg.norm(dr[:2]))
                        + np.random.randn() * np.radians(0.05),
        ])
        ekf.update(z, R, gate_k=50.0)

    return ekf.x.copy(), ekf.P.copy()


# ── Main ──────────────────────────────────────────────────────────
def main():
    print("=" * 58)
    print("  SIL Verification: Python TH-EKF  vs  C TH-EKF")
    print("=" * 58)

    x0     = np.array([0., 500., 0., 0., 1e-3, 0.])
    N      = 360    # 360 × 10s = 3600s = 1 GEO orbit
    labels = ['dx [m]', 'dy [m]', 'dz [m]',
              'dvx[m/s]', 'dvy[m/s]', 'dvz[m/s]']

    print(f"\n  Running {N} predict/update steps ({N*DT:.0f}s) …")

    x_py, P_py = run(
        THEKF_Python(a_chief=A_GEO, e_chief=E_GEO, mu=MU, dt=DT),
        N, x0)
    x_c, P_c = run(
        THEKF_C(a_chief=A_GEO, e_chief=E_GEO, mu=MU, dt=DT),
        N, x0)

    # ── State divergence ─────────────────────────────────────────
    print(f"\n  State divergence after {N} steps:")
    state_ok = True
    for lbl, ep, ec in zip(labels, x_py, x_c):
        err = abs(ep - ec)
        thresh = PASS_POS_M if 'm]' in lbl else PASS_VEL_MS
        ok  = err < thresh
        if not ok:
            state_ok = False
        mark = '✓' if ok else '✗'
        print(f"    {mark} {lbl:<12}  err={err:.2e}  "
              f"(thr={thresh:.0e})  py={ep:+.6f}  c={ec:+.6f}")

    # ── Covariance divergence ─────────────────────────────────────
    cov_err = abs(P_py - P_c).max()
    cov_ok  = cov_err < PASS_COV
    print(f"\n  Covariance max element error: {cov_err:.2e}  "
          f"(thr={PASS_COV:.0e})  {'✓' if cov_ok else '✗'}")

    # ── Summary ───────────────────────────────────────────────────
    all_ok = state_ok and cov_ok
    print("\n" + "=" * 58)
    if all_ok:
        print("  ✓  ALL PASS — C EKF matches Python golden model")
    else:
        print("  ✗  DIVERGENCE DETECTED — check output above")
    print("=" * 58)

    return 0 if all_ok else 1


if __name__ == "__main__":
    sys.exit(main())