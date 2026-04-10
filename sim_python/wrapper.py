"""
wrapper.py — ctypes bridge: Python <-> C TH-EKF
================================================
Drop-in replacement for th_ekf.THEKF.

Usage (in flight sim main.py):
    import sys
    sys.path.insert(0, r"C:\\Users\\Venkat\\OneDrive\\Desktop\\appex\\Satellite_GNC")
    from sim_python.wrapper import THEKF_C as THEKF

Falls back to Python THEKF automatically if gnc_lib.dll is not built yet.

IMPORTANT — package layout fix:
    The flight sim stores THEKF at:
        flight sim/estimation/th_ekf.py
    NOT at:
        flight sim/th_ekf.py
    The fallback import therefore uses:
        from estimation.th_ekf import THEKF
    with FLIGHT_SIM_PATH (the parent of estimation/) on sys.path.
"""

import ctypes
import numpy as np
import os
import sys

# Parent folder of estimation/, control/, etc. in the flight sim
FLIGHT_SIM_PATH = r"C:\Users\Venkat\OneDrive\Desktop\appex\flight sim"

_HERE = os.path.dirname(os.path.abspath(__file__))   # sim_python\
_ROOT = os.path.dirname(_HERE)                        # Satellite_GNC\


def _find_lib():
    """Locate gnc_lib in Satellite_GNC\ root."""
    for name in ["gnc_lib.dll", "gnc_lib.so", "gnc_lib.dylib"]:
        p = os.path.join(_ROOT, name)
        if os.path.exists(p):
            return p
    raise FileNotFoundError(
        f"gnc_lib not found in {_ROOT!r} — run build.bat first")


# ── C struct layout — must exactly mirror THEKF_State in th_ekf.h ──
# Field order and sizes are validated by verify_sil.py.
# Total size: 8 scalars + 6 doubles + 36 doubles + 36 doubles + 2 doubles = 704 bytes
class _THEKF_State_C(ctypes.Structure):
    _fields_ = [
        # Orbit parameters (8 × double)
        ("a",     ctypes.c_double),
        ("e",     ctypes.c_double),
        ("mu",    ctypes.c_double),
        ("dt",    ctypes.c_double),
        ("n",     ctypes.c_double),
        ("p",     ctypes.c_double),
        ("h_orb", ctypes.c_double),
        ("eta",   ctypes.c_double),
        # State (6 × double)
        ("x",     ctypes.c_double * 6),
        # Covariance P (6×6 = 36 × double, row-major)
        ("P",     ctypes.c_double * 36),
        # Process noise Q (6×6 = 36 × double, row-major)
        ("Q",     ctypes.c_double * 36),
        # True anomaly tracking
        ("nu",    ctypes.c_double),
        ("t_ekf", ctypes.c_double),
    ]


# ── Load library ──────────────────────────────────────────────────
try:
    _lib = ctypes.CDLL(_find_lib())

    _lib.THEKF_init.argtypes = [
        ctypes.POINTER(_THEKF_State_C),
        ctypes.c_double,  # a_chief_m
        ctypes.c_double,  # e_chief
        ctypes.c_double,  # mu
        ctypes.c_double,  # dt_s
        ctypes.c_double,  # q_pos
        ctypes.c_double,  # q_vel
    ]
    _lib.THEKF_init.restype = None

    _lib.THEKF_seed.argtypes = [
        ctypes.POINTER(_THEKF_State_C),
        ctypes.POINTER(ctypes.c_double),  # x0[6]
        ctypes.POINTER(ctypes.c_double),  # P0[36]  (NULL → keep)
        ctypes.c_double,                  # nu0
    ]
    _lib.THEKF_seed.restype = None

    _lib.THEKF_predict.argtypes = [
        ctypes.POINTER(_THEKF_State_C),
        ctypes.POINTER(ctypes.c_double),  # accel_lvlh[3]  (NULL → zeros)
    ]
    _lib.THEKF_predict.restype = None

    _lib.THEKF_update.argtypes = [
        ctypes.POINTER(_THEKF_State_C),
        ctypes.POINTER(ctypes.c_double),  # z_meas[3]
        ctypes.POINTER(ctypes.c_double),  # R_meas[9] row-major
        ctypes.c_double,                  # gate_k
    ]
    _lib.THEKF_update.restype = ctypes.c_int

    _C_AVAILABLE = True
    print("[wrapper] C library loaded OK →", _find_lib())

except (FileNotFoundError, OSError) as _e:
    print(f"[wrapper] WARNING: {_e}")
    print("[wrapper] Falling back to Python THEKF")
    _C_AVAILABLE = False


# ── Python class — drop-in for THEKF ─────────────────────────────
class THEKF_C:
    """
    Drop-in replacement for estimation.th_ekf.THEKF.
    Uses the compiled C library when available; falls back to Python.

    API is identical to THEKF:
        initialise(x0, P0=None, nu0=0.0)
        predict(accel_lvlh=None)
        update(z, R_meas, gate_k=5.0)  → bool
        reinit_from_measurements(sensor, true_cw_pos, ...)  → bool
        .x, .P, .nu, .position, .velocity, .position_std
    """

    def __init__(self, a_chief, e_chief,
                 mu=3.986004418e14, dt=1.0,
                 q_pos=1e-4, q_vel=1e-8):

        self._use_c = _C_AVAILABLE

        if not self._use_c:
            # Python fallback — import from flight sim package
            if FLIGHT_SIM_PATH not in sys.path:
                sys.path.insert(0, FLIGHT_SIM_PATH)
            try:
                from estimation.th_ekf import THEKF as _PyTHEKF
            except ImportError:
                # Flat layout fallback (th_ekf.py directly in flight sim root)
                from th_ekf import THEKF as _PyTHEKF
            self._py = _PyTHEKF(a_chief=a_chief, e_chief=e_chief,
                                  mu=mu, dt=dt, q_pos=q_pos, q_vel=q_vel)
            return

        self._s = _THEKF_State_C()
        _lib.THEKF_init(ctypes.byref(self._s),
                        float(a_chief), float(e_chief),
                        float(mu), float(dt),
                        float(q_pos), float(q_vel))

    # ── State accessors ───────────────────────────────────────────

    @property
    def x(self):
        if not self._use_c: return self._py.x
        return np.array(list(self._s.x))

    @x.setter
    def x(self, val):
        if not self._use_c: self._py.x = val; return
        for i in range(6): self._s.x[i] = float(val[i])

    @property
    def P(self):
        if not self._use_c: return self._py.P
        return np.array(list(self._s.P)).reshape(6, 6)

    @P.setter
    def P(self, val):
        if not self._use_c: self._py.P = val; return
        flat = np.asarray(val).flatten()
        for i in range(36): self._s.P[i] = float(flat[i])

    @property
    def nu(self):
        if not self._use_c: return self._py.nu
        return float(self._s.nu)

    @nu.setter
    def nu(self, val):
        if not self._use_c: self._py.nu = val; return
        self._s.nu = float(val)

    @property
    def position(self):
        return self.x[0:3].copy()

    @property
    def velocity(self):
        return self.x[3:6].copy()

    @property
    def position_std(self):
        return np.sqrt(np.maximum(np.diag(self.P)[0:3], 0.0))

    # ── Core EKF methods ─────────────────────────────────────────

    def initialise(self, x0, P0=None, nu0=0.0):
        if not self._use_c:
            self._py.initialise(x0, P0, nu0); return
        x0_c = (ctypes.c_double * 6)(*np.asarray(x0).tolist())
        if P0 is not None:
            P0_c = (ctypes.c_double * 36)(*np.asarray(P0).flatten().tolist())
            _lib.THEKF_seed(ctypes.byref(self._s), x0_c, P0_c, float(nu0))
        else:
            _lib.THEKF_seed(ctypes.byref(self._s), x0_c, None, float(nu0))

    def predict(self, accel_lvlh=None):
        if not self._use_c:
            self._py.predict(accel_lvlh); return
        if accel_lvlh is not None and np.any(np.asarray(accel_lvlh) != 0):
            a_c = (ctypes.c_double * 3)(*np.asarray(accel_lvlh).tolist())
            _lib.THEKF_predict(ctypes.byref(self._s), a_c)
        else:
            _lib.THEKF_predict(ctypes.byref(self._s), None)

    def update(self, z, R_meas, gate_k=5.0):
        if not self._use_c:
            return self._py.update(z, R_meas, gate_k)
        z_c = (ctypes.c_double * 3)(*np.asarray(z).tolist())
        R_c = (ctypes.c_double * 9)(*np.asarray(R_meas).flatten().tolist())
        return bool(_lib.THEKF_update(
            ctypes.byref(self._s), z_c, R_c, float(gate_k)))

    def reinit_from_measurements(self, sensor, true_cw_pos,
                                  n_avg=10, P_pos_m=2.0, P_vel_ms=0.05):
        if not self._use_c:
            return self._py.reinit_from_measurements(
                sensor, true_cw_pos, n_avg, P_pos_m, P_vel_ms)

        rng = np.linalg.norm(true_cw_pos)
        bs  = (true_cw_pos / rng if rng > 1.0
               else np.array([0., -1., 0.]))
        ests = []
        for _ in range(n_avg):
            z, _R = sensor.measure(true_cw_pos, bs)
            if z is not None:
                r, az, el = z
                ests.append(np.array([
                    r * np.cos(el) * np.cos(az),
                    r * np.cos(el) * np.sin(az),
                    r * np.sin(el)]))
        if not ests:
            return False

        xn = self.x
        xn[0:3] = np.mean(ests, axis=0)
        xn[3:6] = 0.0
        self.x = xn
        self.P = np.diag([P_pos_m**2] * 3 + [P_vel_ms**2] * 3)
        return True