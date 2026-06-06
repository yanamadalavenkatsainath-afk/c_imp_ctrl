"""
realtime_driver.py -- Python ctypes driver for C SIL flight loop
================================================================
Feeds simulated sensor packets to the C flight loop DLL at 100 Hz
and collects CommandFrame telemetry into a structured log.

Usage (from Satellite_GNC root):
    python sim_python/realtime_driver.py

Requires: gnc_lib.dll compiled with flight_loop.c included (see build.bat).

Architecture:
    Python sim   ->  SensorFrame*  ->  flight_loop_step()  ->  CommandFrame*
    (fills fields)                  (C processes 10ms tick)  (Python reads)
"""

import ctypes
import ctypes.util
import os
import sys
import time
import math
import numpy as np
from dataclasses import dataclass, field
from typing import Optional, List

# -- DLL path -----------------------------------------------------
_HERE = os.path.dirname(os.path.abspath(__file__))
_ROOT = os.path.dirname(_HERE)
_DLL_PATH = os.path.join(_ROOT, "gnc_lib.dll")
DOCK_PORT_BODY = np.array([0.0, 0.0, 0.5])
DOCK_AXIS_BODY = np.array([0.0, 0.0, 1.0])
CHIEF_OMEGA_LVLH = np.array([0.0012, -0.0015, 0.0008])
PORT_SENSOR_RANGE_M = 10.0
PORT_SENSOR_SIGMA_M = 0.02

# -- ctypes struct mirrors of C headers ---------------------------

class GyroPacket(ctypes.Structure):
    _fields_ = [
        ("omega_xyz", ctypes.c_double * 3),
        ("valid",     ctypes.c_uint8),
        ("_pad",      ctypes.c_uint8 * 7),
    ]

class RangePacket(ctypes.Structure):
    _fields_ = [
        ("range_m",       ctypes.c_double),
        ("azimuth_rad",   ctypes.c_double),
        ("elevation_rad", ctypes.c_double),
        ("valid",         ctypes.c_uint8),
        ("_pad",          ctypes.c_uint8 * 7),
    ]

class CameraPacket(ctypes.Structure):
    _fields_ = [
        ("pos_lvlh", ctypes.c_double * 3),
        ("R_diag",   ctypes.c_double * 3),
        ("valid",    ctypes.c_uint8),
        ("_pad",     ctypes.c_uint8 * 7),
    ]

class PortPacket(ctypes.Structure):
    _fields_ = [
        ("port_lvlh", ctypes.c_double * 3),
        ("port_axis_lvlh", ctypes.c_double * 3),
        ("port_vel_lvlh", ctypes.c_double * 3),
        ("R_body_to_lvlh", (ctypes.c_double * 3) * 3),
        ("R_diag",    ctypes.c_double * 3),
        ("valid",     ctypes.c_uint8),
        ("_pad",      ctypes.c_uint8 * 7),
    ]

class MagPacket(ctypes.Structure):
    _fields_ = [
        ("body",     ctypes.c_double * 3),
        ("inertial", ctypes.c_double * 3),
        ("valid",    ctypes.c_uint8),
        ("_pad",     ctypes.c_uint8 * 7),
    ]

class SunPacket(ctypes.Structure):
    _fields_ = [
        ("body",     ctypes.c_double * 3),
        ("inertial", ctypes.c_double * 3),
        ("valid",    ctypes.c_uint8),
        ("_pad",     ctypes.c_uint8 * 7),
    ]

class SensorFrame(ctypes.Structure):
    _fields_ = [
        ("gyro",        GyroPacket),
        ("range",       RangePacket),
        ("camera",      CameraPacket),
        ("port",        PortPacket),
        ("mag",         MagPacket),
        ("sun",         SunPacket),
        ("sim_tick",    ctypes.c_uint64),
        ("sim_time_s",  ctypes.c_double),
    ]

class NavState(ctypes.Structure):
    _fields_ = [
        ("pos_lvlh", ctypes.c_double * 3),
        ("vel_lvlh", ctypes.c_double * 3),
        ("pos_std",  ctypes.c_double * 3),
        ("vel_std",  ctypes.c_double * 3),
        ("range_m",  ctypes.c_double),
    ]

class AttState(ctypes.Structure):
    _fields_ = [
        ("q_wxyz",           ctypes.c_double * 4),
        ("bias_xyz",         ctypes.c_double * 3),
        ("quest_quality",    ctypes.c_double),
        ("pointing_err_deg", ctypes.c_double),
    ]

class GuidanceCmd(ctypes.Structure):
    _fields_ = [
        ("accel_lvlh",  ctypes.c_double * 3),
        ("torque_rw",   ctypes.c_double * 3),
        ("dipole_mtq",  ctypes.c_double * 3),
        ("fsw_mode",    ctypes.c_int32),
        ("rpod_mode",   ctypes.c_int32),
    ]

class TimingTelemetry(ctypes.Structure):
    _fields_ = [
        ("tick",              ctypes.c_uint64),
        ("loop_time_ms",      ctypes.c_double),
        ("max_loop_time_ms",  ctypes.c_double),
        ("missed_deadlines",  ctypes.c_uint32),
        ("deadline_ms",       ctypes.c_double),
    ]

class RPODTelemetry(ctypes.Structure):
    _fields_ = [
        ("port_range_m",       ctypes.c_double),
        ("port_vrel_ms",       ctypes.c_double),
        ("attitude_align_deg", ctypes.c_double),
        ("cone_error_deg",     ctypes.c_double),
        ("lateral_m",          ctypes.c_double),
        ("phase_elapsed_s",    ctypes.c_double),
        ("has_port",           ctypes.c_int32),
        ("geometry_ok",        ctypes.c_int32),
        ("body_clear",         ctypes.c_int32),
        ("capture_core",       ctypes.c_int32),
        ("timeout_code",       ctypes.c_int32),
        ("pose_age_s",         ctypes.c_double),
        ("spin_sync_rate_cmd", ctypes.c_double * 3),
        ("pose_status",        ctypes.c_int32),
        ("pose_valid",         ctypes.c_int32),
        ("spin_sync_active",   ctypes.c_int32),
    ]

class CommandFrame(ctypes.Structure):
    _fields_ = [
        ("nav",         NavState),
        ("att",         AttState),
        ("cmd",         GuidanceCmd),
        ("timing",      TimingTelemetry),
        ("rpod",        RPODTelemetry),
        ("ekf_updated", ctypes.c_uint8),
        ("_pad",        ctypes.c_uint8 * 7),
    ]

def _quat_to_rot(q: np.ndarray) -> np.ndarray:
    w, x, y, z = q
    return np.array([
        [1-2*(y*y+z*z),   2*(x*y-w*z),   2*(x*z+w*y)],
        [  2*(x*y+w*z), 1-2*(x*x+z*z),   2*(y*z-w*x)],
        [  2*(x*z-w*y),   2*(y*z+w*x), 1-2*(x*x+y*y)],
    ])

def _propagate_quat(q: np.ndarray, omega: np.ndarray, dt: float) -> np.ndarray:
    wx, wy, wz = omega
    Omega = np.array([
        [ 0,  -wx, -wy, -wz],
        [ wx,  0,   wz, -wy],
        [ wy, -wz,  0,   wx],
        [ wz,  wy, -wx,  0 ],
    ])
    q_next = q + 0.5 * dt * (Omega @ q)
    return q_next / (np.linalg.norm(q_next) + 1e-30)

# -- Telemetry log row --------------------------------------------
@dataclass
class TelRow:
    tick:          int
    sim_time_s:    float
    pos_lvlh:      np.ndarray
    vel_lvlh:      np.ndarray
    pos_std:       np.ndarray
    accel_cmd:     np.ndarray
    fsw_mode: int
    loop_time_ms:  float
    ekf_updated:   bool
    range_valid:   bool
    camera_valid:  bool
    gyro_valid:    bool

# -- DLL wrapper --------------------------------------------------
class FlightLoopDLL:
    GUIDANCE_MODES = {0: "PROX_OPS", 1: "TERMINAL", 2: "DOCKED",
                      3: "LOST_TARGET", 4: "FORMATION_HOLD"}

    def __init__(self, dll_path: str = _DLL_PATH):
        if not os.path.exists(dll_path):
            raise FileNotFoundError(
                f"gnc_lib.dll not found at {dll_path}\n"
                "Run build.bat first (must include flight_loop.c)."
            )
        self._lib = ctypes.CDLL(dll_path)
        self._bind()

    def _bind(self):
        lib = self._lib

        lib.flight_loop_init.restype  = None
        lib.flight_loop_init.argtypes = [
            ctypes.c_double, ctypes.c_double,
            ctypes.c_double, ctypes.c_double
        ]
        lib.flight_loop_seed_thekf.restype  = None
        lib.flight_loop_seed_thekf.argtypes = [
            ctypes.POINTER(ctypes.c_double),
            ctypes.POINTER(ctypes.c_double),
            ctypes.c_double
        ]
        lib.flight_loop_get_sensor_frame.restype  = ctypes.POINTER(SensorFrame)
        lib.flight_loop_get_sensor_frame.argtypes = []
        lib.flight_loop_get_command_frame.restype  = ctypes.POINTER(CommandFrame)
        lib.flight_loop_get_command_frame.argtypes = []
        lib.flight_loop_step.restype  = ctypes.POINTER(CommandFrame)
        lib.flight_loop_step.argtypes = []
        lib.flight_loop_reset.restype  = None
        lib.flight_loop_reset.argtypes = []
        lib.flight_loop_get_thekf_state.restype  = None
        lib.flight_loop_get_thekf_state.argtypes = [
            ctypes.POINTER(ctypes.c_double),
            ctypes.POINTER(ctypes.c_double)
        ]

    def init(self, a_chief_m, e_chief, mu, dt_thekf_s=0.1):
        self._lib.flight_loop_init(a_chief_m, e_chief, mu, dt_thekf_s)

    def seed_thekf(self, x0: np.ndarray, P0: np.ndarray, nu0: float = 0.0):
        x0_c = (ctypes.c_double * 6)(*x0)
        P0_c = (ctypes.c_double * 36)(*P0.flatten())
        self._lib.flight_loop_seed_thekf(x0_c, P0_c, nu0)

    def get_sensor_frame(self) -> "ctypes.POINTER[SensorFrame]":  # type: ignore[valid-type]
        return self._lib.flight_loop_get_sensor_frame()

    def get_command_frame(self) -> "ctypes.POINTER[CommandFrame]":  # type: ignore[valid-type]
        return self._lib.flight_loop_get_command_frame()

    def step(self) -> "ctypes.POINTER[CommandFrame]":  # type: ignore[valid-type]
        return self._lib.flight_loop_step()

    def reset(self):
        self._lib.flight_loop_reset()

    def get_thekf_state(self):
        x_buf = (ctypes.c_double * 6)()
        P_buf = (ctypes.c_double * 36)()
        self._lib.flight_loop_get_thekf_state(x_buf, P_buf)
        return np.array(list(x_buf)), np.array(list(P_buf)).reshape(6, 6)


# -- Fake sensor generator --------------------------------------- 
class FakeSensorSim:
    """
    Minimal closed-loop sim: propagates CW relative motion,
    generates noisy sensor readings, injects dropout scenarios.
    """
    def __init__(self, x0: np.ndarray, n_chief: float, dt: float = 0.01,
                 rng_seed: int = 42):
        self.x   = x0.copy()   # [dx, dy, dz, dvx, dvy, dvz]
        self.n   = n_chief
        self.dt  = dt
        self.t   = 0.0
        self.rng = np.random.default_rng(rng_seed)
        self.tick = -1   # incremented to 0 on first _cw_step, aligns with g_tick=0 in C

        # True attitude quaternion [w,x,y,z] propagated every gyro tick.
        # Used to rotate v_inertial into body frame for mag measurement,
        # giving the MEKF a real signal to observe and estimate gyro bias.
        self.q_true = np.array([1.0, 0.0, 0.0, 0.0])
        self._omega_true = np.array([0.001, 0.0005, -0.0003])  # rad/s, matches gyro sim
        self.q_chief = np.array([1.0, 0.0, 0.0, 0.0])
        self.omega_chief_lvlh = CHIEF_OMEGA_LVLH.copy()

        # Dropout scenario flags (set externally)
        self.range_dropout  = False
        self.camera_dropout = False
        self.gyro_dropout   = False

    def _cw_step(self, accel: Optional[np.ndarray] = None) -> np.ndarray:
        """Simple CW propagation for faking truth dynamics."""
        n, dt = self.n, self.dt
        x = self.x
        nt = n * dt
        s, c = math.sin(nt), math.cos(nt)

        # CW STM (circular approximation)
        Phi = np.array([
            [4-3*c,    0, 0,  s/n,        2*(1-c)/n, 0],
            [6*(s-nt), 1, 0, -2*(1-c)/n,  (4*s-3*nt)/n, 0],
            [0,        0, c,  0,           0,          s/n],
            [3*n*s,    0, 0,  c,           2*s,        0],
            [-6*n*(1-c), 0, 0, -2*s,       4*c-3,      0],
            [0,          0, -n*s, 0,        0,          c],
        ])
        Bu = np.zeros(6)
        if accel is not None:
            Bu[3:] = accel * dt
        self.x = Phi @ x + Bu
        self.t += dt
        self.tick += 1

        # Propagate true attitude quaternion (first-order integration)
        wx, wy, wz = self._omega_true
        Omega = np.array([
            [ 0,   -wx, -wy, -wz],
            [ wx,   0,   wz, -wy],
            [ wy,  -wz,  0,   wx],
            [ wz,   wy, -wx,  0 ],
        ])
        q = self.q_true
        dq = 0.5 * dt * Omega @ q
        q = q + dq
        self.q_true = q / np.linalg.norm(q)
        self.q_chief = _propagate_quat(self.q_chief, self.omega_chief_lvlh, dt)

        return self.x

    def generate_frame(self, accel_cmd: Optional[np.ndarray] = None) -> SensorFrame:
        """Propagate sim one step and fill a SensorFrame."""
        self._cw_step(accel_cmd)
        sf = SensorFrame()

        # -- Gyro (100 Hz) ----------------------------
        if not self.gyro_dropout:
            omega_true = np.array([0.001, 0.0005, -0.0003])
            noise = self.rng.standard_normal(3) * 1e-4
            for i in range(3):
                sf.gyro.omega_xyz[i] = omega_true[i] + noise[i]
            sf.gyro.valid = 1
        else:
            sf.gyro.valid = 0

        # -- 10 Hz sensors: range, camera, magnetometer -----------
        if self.tick % 10 == 0:
            dr  = self.x[:3]
            rng = np.linalg.norm(dr)
            az  = math.atan2(dr[1], dr[0])
            el  = math.atan2(dr[2], math.sqrt(dr[0]**2 + dr[1]**2))

            # Range
            if not self.range_dropout:
                sf.range.range_m       = rng + self.rng.standard_normal() * 0.3
                sf.range.azimuth_rad   = az  + self.rng.standard_normal() * 1.7e-3
                sf.range.elevation_rad = el  + self.rng.standard_normal() * 1.7e-3
                sf.range.valid = 1
            else:
                sf.range.valid = 0

            # Camera (close range only, < 100m)
            if rng < 100.0 and not self.camera_dropout:
                for i in range(3):
                    sf.camera.pos_lvlh[i] = dr[i] + self.rng.standard_normal() * 0.1
                    sf.camera.R_diag[i]   = 0.01
                sf.camera.valid = 1
            else:
                sf.camera.valid = 0

            if rng < PORT_SENSOR_RANGE_M and not self.camera_dropout:
                R_c2l = _quat_to_rot(self.q_chief)
                port_lvlh = R_c2l @ DOCK_PORT_BODY
                axis_lvlh = R_c2l @ DOCK_AXIS_BODY
                port_vel = np.cross(self.omega_chief_lvlh, port_lvlh)
                z_port = port_lvlh + self.rng.standard_normal(3) * PORT_SENSOR_SIGMA_M
                for i in range(3):
                    sf.port.port_lvlh[i] = z_port[i]
                    sf.port.port_axis_lvlh[i] = axis_lvlh[i]
                    sf.port.port_vel_lvlh[i] = port_vel[i]
                    sf.port.R_diag[i]    = PORT_SENSOR_SIGMA_M ** 2
                    for j in range(3):
                        sf.port.R_body_to_lvlh[i][j] = R_c2l[i, j]
                sf.port.valid = 1
            else:
                sf.port.valid = 0

            # Magnetometer -- needed for MEKF bias estimation.
            # z_body = Rb(q_true)^T @ v_inertial + noise
            # where Rb is built from the true attitude quaternion.
            # This gives a real observation signal so the MEKF can
            # observe and estimate gyro bias.
            v_inertial = np.array([0.6, 0.0, 0.8])
            v_inertial = v_inertial / np.linalg.norm(v_inertial)
            w, x, y, z = self.q_true
            # Rotation matrix R maps inertial -> body: body = R @ inertial
            Rb = np.array([
                [1-2*(y*y+z*z),   2*(x*y-w*z),   2*(x*z+w*y)],
                [  2*(x*y+w*z), 1-2*(x*x+z*z),   2*(y*z-w*x)],
                [  2*(x*z-w*y),   2*(y*z+w*x), 1-2*(x*x+y*y)],
            ])
            z_body = Rb @ v_inertial + self.rng.standard_normal(3) * 1e-2
            z_body = z_body / np.linalg.norm(z_body)
            for i in range(3):
                sf.mag.body[i]     = z_body[i]
                sf.mag.inertial[i] = v_inertial[i]
            sf.mag.valid = 1

        sf.sim_tick   = self.tick
        sf.sim_time_s = self.t
        return sf


# -- Main SIL driver -----------------------------------------------
def run_sil(n_ticks: int = 3600,
            scenario: str = "nominal",
            verbose: bool = True) -> List[TelRow]:
    """
    Run n_ticks (100 Hz ticks = n_ticks/100 seconds) of SIL.
    Returns list of TelRow for post-processing.

    Scenarios:
      "nominal"        -- all sensors valid
      "range_dropout"  -- range drops out at tick 1000-1200
      "camera_spike"   -- camera sends bad spike at tick 500
      "gyro_bias"      -- gyro bias jumps at tick 800
    """
    MU    = 3.986004418e14
    A_GEO = 42164e3
    E_GEO = 0.001
    N     = math.sqrt(MU / A_GEO**3)

    dll = FlightLoopDLL()
    dll.init(A_GEO, E_GEO, MU, 0.1)   # TH-EKF at 10 Hz

    x0  = np.array([0., 500., 0., 0., 1e-3, 0.])
    P0  = np.diag([50.**2]*3 + [0.5**2]*3)
    dll.seed_thekf(x0, P0, 0.0)

    sim  = FakeSensorSim(x0, N, dt=0.01)
    log: List[TelRow] = []

    last_accel = np.zeros(3)
    t0_wall    = time.perf_counter()

    if verbose:
        print(f"\n{'='*60}")
        print(f"  SIL Real-Time Driver  --  scenario: {scenario}")
        print(f"  {n_ticks} ticks @ 100 Hz = {n_ticks/100:.1f}s sim time")
        print(f"{'='*60}")

    for tick in range(n_ticks):
        # -- Dropout injection ----------------------
        sim.range_dropout  = False
        sim.camera_dropout = False
        sim.gyro_dropout   = False

        if scenario == "range_dropout" and 1000 <= tick < 1200:
            sim.range_dropout = True
        elif scenario == "camera_spike" and tick == 500:
            sim.camera_dropout = False   # allow camera but inject spike below
        elif scenario == "gyro_bias" and tick >= 800:
            pass  # handled in sensor frame overwrite below

        # -- Generate fake sensor frame -------------
        sf_py = sim.generate_frame(last_accel)

        # Gyro bias jump injection
        if scenario == "gyro_bias" and tick >= 800:
            for i in range(3):
                sf_py.gyro.omega_xyz[i] += 0.005   # 5 mrad/s step bias

        # Camera spike injection
        if scenario == "camera_spike" and tick == 500:
            sf_py.camera.valid = 1
            sf_py.camera.pos_lvlh[0] = 9999.0   # blatant spike

        # -- Copy into DLL sensor frame -------------
        dll_sf = dll.get_sensor_frame()
        ctypes.memmove(dll_sf.contents, ctypes.addressof(sf_py),  # type: ignore[arg-type]
                       ctypes.sizeof(SensorFrame))

        # -- Step C flight loop ---------------------
        cf_ptr = dll.step()
        cf = cf_ptr.contents

        # -- Harvest command for next sim step -----
        last_accel = np.array(list(cf.cmd.accel_lvlh))

        # -- Log -----------------------------------
        if tick % 10 == 0:   # log at 10 Hz (guidance rate)
            row = TelRow(
                tick          = tick,
                sim_time_s    = sf_py.sim_time_s,
                pos_lvlh      = np.array(list(cf.nav.pos_lvlh)),
                vel_lvlh      = np.array(list(cf.nav.vel_lvlh)),
                pos_std       = np.array(list(cf.nav.pos_std)),
                accel_cmd     = last_accel.copy(),
                fsw_mode = cf.cmd.fsw_mode,
                loop_time_ms  = cf.timing.loop_time_ms,
                ekf_updated   = bool(cf.ekf_updated),
                range_valid   = bool(sf_py.range.valid),
                camera_valid  = bool(sf_py.camera.valid),
                gyro_valid    = bool(sf_py.gyro.valid),
            )
            log.append(row)

    elapsed_wall = time.perf_counter() - t0_wall
    cf_ptr = dll.get_command_frame()
    cf     = cf_ptr.contents

    if verbose:
        print(f"\n  Sim time : {n_ticks/100:.1f}s  Wall time: {elapsed_wall*1000:.1f}ms")
        print(f"  Max loop : {cf.timing.max_loop_time_ms:.4f}ms  "
              f"(deadline {cf.timing.deadline_ms:.1f}ms)")
        print(f"  Missed   : {cf.timing.missed_deadlines} deadlines")
        print(f"  EKF pos  : {list(np.round(cf.nav.pos_lvlh, 2))} m")
        print(f"  Accel    : {list(np.round(cf.cmd.accel_lvlh, 5))} m/s2")
        mode = FlightLoopDLL.GUIDANCE_MODES.get(cf.cmd.fsw_mode, "?")
        print(f"  Mode     : {mode}")

    return log


def print_dropout_summary(log: List[TelRow], scenario: str):
    """Print a summary of sensor availability and EKF updates."""
    n_range_drop  = sum(1 for r in log if not r.range_valid)
    n_cam_drop    = sum(1 for r in log if not r.camera_valid)
    n_ekf_updated = sum(1 for r in log if r.ekf_updated)
    n_total       = len(log)

    print(f"\n  Dropout summary ({scenario}):")
    print(f"    Range dropout ticks   : {n_range_drop}/{n_total}")
    print(f"    Camera dropout ticks  : {n_cam_drop}/{n_total}")
    print(f"    EKF updated ticks     : {n_ekf_updated}/{n_total}")


if __name__ == "__main__":
    import argparse
    parser = argparse.ArgumentParser()
    parser.add_argument("--scenario", default="nominal",
                        choices=["nominal", "range_dropout",
                                 "camera_spike", "gyro_bias"])
    parser.add_argument("--ticks", type=int, default=3600)
    args = parser.parse_args()

    log = run_sil(n_ticks=args.ticks, scenario=args.scenario, verbose=True)
    print_dropout_summary(log, args.scenario)
