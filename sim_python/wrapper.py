# -*- coding: utf-8 -*-
"""
wrapper.py -- ctypes bridge: Python <-> C GNC library
=====================================================
Drop-in replacement for estimation.th_ekf.THEKF.

New bindings vs old wrapper:
  + THEKF_update_position()         -- linear camera position update
  + THEKF_update_velocity_doppler() -- scalar Doppler radial-velocity update
  + THEKF_inflate_process_noise()   -- P inflation for TERMINAL phase
  + RPOD_prox_ops() / RPOD_terminal() / RPOD_lost_target() / RPOD_formation_hold()
    all use updated struct layouts matching rpod_ctrl.h

Usage (in flight sim main.py):
    import sys
    sys.path.insert(0, r"C:\\Users\\Venkat\\OneDrive\\Desktop\\appex\\Satellite_GNC")
    from sim_python.wrapper import THEKF_C as THEKF

Falls back to Python THEKF automatically if gnc_lib.dll is not built yet.
"""

import ctypes
import numpy as np
import os
import sys
from typing import Optional

FLIGHT_SIM_PATH = r"C:\Users\Venkat\OneDrive\Desktop\appex\flight sim"

# -- Pylance / pyright: suppress unresolvable runtime path imports --
# These modules live in FLIGHT_SIM_PATH which is only on sys.path at
# runtime (injected above). Pylance can't see them statically -- that
# is expected and not a bug. The # type: ignore comments on the
# import lines suppress the false-positive red squiggles.
# To make Pylance fully happy, add a pyrightconfig.json (generated
# by build.bat) that adds FLIGHT_SIM_PATH to extraPaths.

_HERE = os.path.dirname(os.path.abspath(__file__))   # sim_python\
_ROOT = os.path.dirname(_HERE)                        # Satellite_GNC\


def _find_lib():
    for name in ["gnc_lib.dll", "gnc_lib.so", "gnc_lib.dylib"]:
        p = os.path.join(_ROOT, name)
        if os.path.exists(p):
            return p
    raise FileNotFoundError(
        f"gnc_lib not found in {_ROOT!r} -- run build.bat first")


# -- C struct layout -- must exactly mirror THEKF_State in th_ekf.h --
class _THEKF_State_C(ctypes.Structure):
    _fields_ = [
        ("a",     ctypes.c_double),
        ("e",     ctypes.c_double),
        ("mu",    ctypes.c_double),
        ("dt",    ctypes.c_double),
        ("n",     ctypes.c_double),
        ("p",     ctypes.c_double),
        ("h_orb", ctypes.c_double),
        ("eta",   ctypes.c_double),
        ("x",     ctypes.c_double * 6),
        ("P",     ctypes.c_double * 36),
        ("Q",     ctypes.c_double * 36),
        ("nu",    ctypes.c_double),
        ("t_ekf", ctypes.c_double),
    ]


# -- RPOD structs -- mirror rpod_ctrl.h exactly ------------------
class _RPOD_State_C(ctypes.Structure):
    _fields_ = [
        ("pos", ctypes.c_double * 3),
        ("vel", ctypes.c_double * 3),
    ]


class _RPOD_TermState_C(ctypes.Structure):
    _fields_ = [
        ("pos",                ctypes.c_double * 3),
        ("vel",                ctypes.c_double * 3),
        ("port_lvlh",          ctypes.c_double * 3),
        ("port_axis_lvlh",     ctypes.c_double * 3),
        ("port_vel_lvlh",      ctypes.c_double * 3),
        ("R_body_to_lvlh",     (ctypes.c_double * 3) * 3),
        ("attitude_align_deg", ctypes.c_double),
        ("cone_angle_deg",     ctypes.c_double),
        ("cone_error_deg",     ctypes.c_double),
        ("lateral_m",          ctypes.c_double),
        ("has_port",           ctypes.c_int),
        ("has_body_R",         ctypes.c_int),
        ("has_attitude_align", ctypes.c_int),
        ("has_geometry",       ctypes.c_int),
        ("geometry_ok",        ctypes.c_int),
        ("body_clear",         ctypes.c_int),
        ("capture_core",       ctypes.c_int),
    ]


class _RPOD_FormHoldState_C(ctypes.Structure):
    _fields_ = [
        ("pos",      ctypes.c_double * 3),
        ("vel",      ctypes.c_double * 3),
        ("n_chief",  ctypes.c_double),
        ("accel_max",ctypes.c_double),
    ]


# -- Load library --------------------------------------------------
try:
    _lib = ctypes.CDLL(_find_lib())

    # -- THEKF_init ------------------------------------------------
    _lib.THEKF_init.argtypes = [
        ctypes.POINTER(_THEKF_State_C),
        ctypes.c_double, ctypes.c_double, ctypes.c_double,
        ctypes.c_double, ctypes.c_double, ctypes.c_double,
    ]
    _lib.THEKF_init.restype = None

    # -- THEKF_seed ------------------------------------------------
    _lib.THEKF_seed.argtypes = [
        ctypes.POINTER(_THEKF_State_C),
        ctypes.POINTER(ctypes.c_double),  # x0[6]
        ctypes.POINTER(ctypes.c_double),  # P0[36] or NULL
        ctypes.c_double,                  # nu0
    ]
    _lib.THEKF_seed.restype = None

    # -- THEKF_predict ---------------------------------------------
    _lib.THEKF_predict.argtypes = [
        ctypes.POINTER(_THEKF_State_C),
        ctypes.POINTER(ctypes.c_double),  # accel_lvlh[3] or NULL
    ]
    _lib.THEKF_predict.restype = None

    # -- THEKF_update (ranging) ------------------------------------
    _lib.THEKF_update.argtypes = [
        ctypes.POINTER(_THEKF_State_C),
        ctypes.POINTER(ctypes.c_double),  # z_meas[3]
        ctypes.POINTER(ctypes.c_double),  # R_meas[9]
        ctypes.c_double,                  # gate_k
    ]
    _lib.THEKF_update.restype = ctypes.c_int

    # -- THEKF_update_position (camera) ---------------------------
    _lib.THEKF_update_position.argtypes = [
        ctypes.POINTER(_THEKF_State_C),
        ctypes.POINTER(ctypes.c_double),  # z_pos[3]
        ctypes.POINTER(ctypes.c_double),  # R_pos[9]
        ctypes.c_double,                  # gate_k
    ]
    _lib.THEKF_update_position.restype = ctypes.c_int

    # -- THEKF_update_velocity_doppler -----------------------------
    _lib.THEKF_update_velocity_doppler.argtypes = [
        ctypes.POINTER(_THEKF_State_C),
        ctypes.c_double,                  # v_radial_meas
        ctypes.POINTER(ctypes.c_double),  # r_hat[3]
        ctypes.c_double,                  # sigma_radial
    ]
    _lib.THEKF_update_velocity_doppler.restype = ctypes.c_int

    # -- THEKF_inflate_process_noise -------------------------------
    _lib.THEKF_inflate_process_noise.argtypes = [
        ctypes.POINTER(_THEKF_State_C),
        ctypes.c_double,                  # scale
    ]
    _lib.THEKF_inflate_process_noise.restype = None

    # -- RPOD functions --------------------------------------------
    _lib.RPOD_prox_ops.argtypes = [
        ctypes.POINTER(_RPOD_State_C),
        ctypes.c_double,                  # truth_range
        ctypes.c_double,                  # n_chief (rad/s)
        ctypes.c_double,                  # accel_max
        ctypes.POINTER(ctypes.c_double),  # accel_out[3]
    ]
    _lib.RPOD_prox_ops.restype = ctypes.c_int

    _lib.RPOD_terminal.argtypes = [
        ctypes.POINTER(_RPOD_TermState_C),
        ctypes.c_double,
        ctypes.POINTER(ctypes.c_double),
        ctypes.POINTER(ctypes.c_int),     # is_braking
    ]
    _lib.RPOD_terminal.restype = ctypes.c_int

    _lib.RPOD_lost_target.argtypes = [
        ctypes.POINTER(_RPOD_State_C),
        ctypes.c_double,
        ctypes.POINTER(ctypes.c_double),
    ]
    _lib.RPOD_lost_target.restype = None

    _lib.RPOD_formation_hold.argtypes = [
        ctypes.POINTER(_RPOD_FormHoldState_C),
        ctypes.POINTER(ctypes.c_double),
    ]
    _lib.RPOD_formation_hold.restype = None

    _C_AVAILABLE = True
    print("[wrapper] C library loaded OK ->", _find_lib())

except (FileNotFoundError, OSError) as _e:
    print(f"[wrapper] WARNING: {_e}")
    print("[wrapper] Falling back to Python THEKF")
    _C_AVAILABLE = False


# -- Python class -- drop-in for THEKF -----------------------------
class THEKF_C:
    """
    Drop-in replacement for estimation.th_ekf.THEKF.
    Uses the compiled C library when available; falls back to Python.

    API matches THEKF exactly:
        initialise(x0, P0=None, nu0=0.0)
        predict(accel_lvlh=None)
        update(z, R_meas, gate_k=5.0)               -> bool
        update_position(z_pos, R_pos, gate_k=5.0)   -> bool   <- NEW
        update_velocity_doppler(v_radial, r_hat,
                                sigma_radial=0.005)  -> bool   <- NEW
        inflate_process_noise(scale=10.0)                     <- NEW
        reinit_from_measurements(...)                -> bool
        .x, .P, .nu, .position, .velocity, .position_std, .velocity_std
    """

    def __init__(self, a_chief, e_chief,
                 mu=3.986004418e14, dt=1.0,
                 q_pos=1e-4, q_vel=1e-8):

        self._use_c = _C_AVAILABLE

        if not self._use_c:
            if FLIGHT_SIM_PATH not in sys.path:
                sys.path.insert(0, FLIGHT_SIM_PATH)
            try:
                from estimation.th_ekf import THEKF as _PyTHEKF  # type: ignore[import]
            except ImportError:
                from th_ekf import THEKF as _PyTHEKF  # type: ignore[import]
            self._py = _PyTHEKF(a_chief=a_chief, e_chief=e_chief,
                                  mu=mu, dt=dt, q_pos=q_pos, q_vel=q_vel)
            return

        self._s = _THEKF_State_C()
        _lib.THEKF_init(ctypes.byref(self._s),
                        float(a_chief), float(e_chief),
                        float(mu), float(dt),
                        float(q_pos), float(q_vel))

    # -- State accessors -------------------------------------------

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

    @property
    def velocity_std(self):
        return np.sqrt(np.maximum(np.diag(self.P)[3:6], 0.0))

    # -- Core EKF methods -----------------------------------------

    def initialise(self,
                   x0,
                   P0: Optional[np.ndarray] = None,
                   nu0: float = 0.0) -> None:
        if not self._use_c:
            self._py.initialise(x0, P0, nu0)  # type: ignore[arg-type]
            return
        x0_c = (ctypes.c_double * 6)(*np.asarray(x0).tolist())
        if P0 is not None:
            P0_c = (ctypes.c_double * 36)(*np.asarray(P0).flatten().tolist())
            _lib.THEKF_seed(ctypes.byref(self._s), x0_c, P0_c, float(nu0))
        else:
            _lib.THEKF_seed(ctypes.byref(self._s), x0_c, None, float(nu0))

    def predict(self, accel_lvlh: Optional[np.ndarray] = None) -> None:
        if not self._use_c:
            # Pass None explicitly when no accel — matches Python THEKF signature
            self._py.predict(accel_lvlh)  # type: ignore[arg-type]
            return
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

    def update_position(self, z_pos, R_pos, gate_k=5.0):
        """
        Linear camera position update.
        z_pos: [dx, dy, dz] in LVLH [m]
        R_pos: 3×3 noise covariance
        Mirrors Python update_position().
        """
        if not self._use_c:
            return self._py.update_position(z_pos, R_pos, gate_k)
        z_c = (ctypes.c_double * 3)(*np.asarray(z_pos).tolist())
        R_c = (ctypes.c_double * 9)(*np.asarray(R_pos).flatten().tolist())
        return bool(_lib.THEKF_update_position(
            ctypes.byref(self._s), z_c, R_c, float(gate_k)))

    def update_velocity_doppler(self, v_radial_meas, r_hat, sigma_radial=0.005):
        """
        Scalar Doppler radial-velocity update.
        v_radial_meas: scalar range-rate [m/s]
        r_hat: EKF unit range vector (from EKF position, NOT truth)
        sigma_radial: 1-sigma noise [m/s]
        Mirrors Python update_velocity_doppler().
        """
        if not self._use_c:
            return self._py.update_velocity_doppler(
                v_radial_meas, r_hat, sigma_radial)
        r_c = (ctypes.c_double * 3)(*np.asarray(r_hat).tolist())
        return bool(_lib.THEKF_update_velocity_doppler(
            ctypes.byref(self._s),
            float(v_radial_meas), r_c, float(sigma_radial)))

    def inflate_process_noise(self, scale=10.0):
        """
        Inflate P[0:3,0:3] by scale * Q[0,0].
        Call at TERMINAL entry to handle degenerate camera geometry.
        Mirrors Python inflate_process_noise().
        """
        if not self._use_c:
            self._py.inflate_process_noise(scale); return
        _lib.THEKF_inflate_process_noise(ctypes.byref(self._s), float(scale))

    def inject_velocity(self, vel_true, sigma_ms=0.020):
        """
        Hard-inject velocity into EKF state (pseudo-measurement).
        Mirrors Python inject_velocity().
        """
        if not self._use_c:
            self._py.inject_velocity(vel_true, sigma_ms); return
        # Implemented in Python for the C wrapper -- sets x[3:6] directly.
        noise = np.random.normal(0, sigma_ms, 3)
        xn = self.x
        xn[3:6] = np.asarray(vel_true) + noise
        self.x = xn
        Pn = self.P
        Pn[3:6, 3:6] = np.eye(3) * (sigma_ms ** 2)
        self.P = Pn

    def reinit_from_measurements(self, sensor, true_cw_pos,
                                  n_avg=10, P_pos_m=2.0, P_vel_ms=0.05):
        if not self._use_c:
            return self._py.reinit_from_measurements(
                sensor, true_cw_pos, n_avg, P_pos_m, P_vel_ms)

        rng = np.linalg.norm(true_cw_pos)
        bs = (true_cw_pos / rng if rng > 1.0 else np.array([0., -1., 0.]))
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
        self.P = np.diag([P_pos_m**2]*3 + [P_vel_ms**2]*3)
        return True
