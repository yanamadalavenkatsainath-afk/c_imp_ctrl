"""
debug_sil.py — Targeted diagnostics for the 3 SIL failures.

Run from Satellite_GNC root:
    python sim_python/debug_sil.py

Probes:
  D1 — Is THEKF_update actually being called and returning 1?
       Print ekf_updated flag and pos_std every 10 ticks for 50 ticks.

  D2 — Is the P ceiling the problem?
       Read back raw P diagonal via flight_loop_get_thekf_state()
       before and after the first accepted update.

  D3 — Is sf.mag.valid actually reaching the DLL?
       Print mag.valid and att_bias every 50 ticks for 300 ticks.

  D4 — Does MEKF_update fire at all?
       Print att_bias at ticks 0,100,200,...,2000 during gyro-bias test.
"""

import sys, os, ctypes, math
import numpy as np

_HERE = os.path.dirname(os.path.abspath(__file__))
_ROOT = os.path.dirname(_HERE)
if _ROOT not in sys.path:
    sys.path.insert(0, _ROOT)

from sim_python.realtime_driver import (
    FlightLoopDLL, FakeSensorSim, SensorFrame
)

MU    = 3.986004418e14
A_GEO = 42164e3
E_GEO = 0.001
N_GEO = math.sqrt(MU / A_GEO**3)

dll = FlightLoopDLL()


# ═══════════════════════════════════════════════════════════════
# D1 + D2 — EKF update acceptance and P ceiling
# ═══════════════════════════════════════════════════════════════
print("\n" + "="*60)
print("D1+D2: EKF update acceptance + P-ceiling diagnosis")
print("="*60)

dll.reset()
dll.init(A_GEO, E_GEO, MU, 0.1)
x0 = np.array([0., 500., 0., 0., 1e-3, 0.])
P0 = np.diag([50.**2]*3 + [0.5**2]*3)
dll.seed_thekf(x0, P0, 0.0)

sim = FakeSensorSim(x0, N_GEO, dt=0.01)
last_accel = np.zeros(3)
n_accepted = 0

print(f"\n  Initial P diagonal (from seed):")
x_raw, P_raw = dll.get_thekf_state()
print(f"    pos: {P_raw.diagonal()[:3]}")
print(f"    vel: {P_raw.diagonal()[3:]}")

print(f"\n  {'tick':>5}  {'range_valid':>11}  {'ekf_updated':>11}  "
      f"{'pos_std[1]':>12}  {'P[1][1] raw':>12}  {'n_accepted':>10}")

for tick in range(150):
    sf_py = sim.generate_frame(last_accel)

    dll_sf = dll.get_sensor_frame()
    ctypes.memmove(dll_sf, ctypes.addressof(sf_py),
                   ctypes.sizeof(SensorFrame))

    cf_ptr = dll.step()
    cf = cf_ptr.contents
    last_accel = np.array(list(cf.cmd.accel_lvlh))

    if cf.ekf_updated:
        n_accepted += 1

    if tick % 10 == 0:
        x_raw, P_raw = dll.get_thekf_state()
        pos_std_1 = math.sqrt(abs(cf.nav.pos_std[1]))  # from command frame
        p11_raw   = P_raw[1][1]                         # direct from EKF state
        print(f"  {tick:>5}  {bool(sf_py.range.valid)!s:>11}  "
              f"{bool(cf.ekf_updated)!s:>11}  "
              f"{cf.nav.pos_std[1]:>12.4f}  "
              f"{p11_raw:>12.4f}  "
              f"{n_accepted:>10}")

print(f"\n  Total accepted updates in 150 ticks: {n_accepted}")
print(f"  (expect ~15 — one per 10 ticks at guidance rate)")


# ═══════════════════════════════════════════════════════════════
# D3 — mag.valid reaching DLL + att_bias changing
# ═══════════════════════════════════════════════════════════════
print("\n" + "="*60)
print("D3: mag.valid in SensorFrame + MEKF bias motion")
print("="*60)

dll.reset()
dll.init(A_GEO, E_GEO, MU, 0.1)
dll.seed_thekf(x0, P0, 0.0)

sim2 = FakeSensorSim(x0, N_GEO, dt=0.01)
last_accel = np.zeros(3)

print(f"\n  {'tick':>5}  {'mag_valid (py)':>14}  {'mag_valid (dll)':>15}  "
      f"{'att_bias[0]':>12}  {'att_bias[1]':>12}  {'att_bias[2]':>12}")

for tick in range(310):
    sf_py = sim2.generate_frame(last_accel)

    dll_sf = dll.get_sensor_frame()
    ctypes.memmove(dll_sf, ctypes.addressof(sf_py),
                   ctypes.sizeof(SensorFrame))

    # Read back what the DLL sees AFTER the memmove, BEFORE step
    dll_sf_readback = dll.get_sensor_frame().contents

    cf_ptr = dll.step()
    cf = cf_ptr.contents
    last_accel = np.array(list(cf.cmd.accel_lvlh))

    if tick % 50 == 0:
        print(f"  {tick:>5}  {bool(sf_py.mag.valid)!s:>14}  "
              f"{bool(dll_sf_readback.mag.valid)!s:>15}  "
              f"{cf.att.bias_xyz[0]:>12.6f}  "
              f"{cf.att.bias_xyz[1]:>12.6f}  "
              f"{cf.att.bias_xyz[2]:>12.6f}")


# ═══════════════════════════════════════════════════════════════
# D4 — MEKF bias convergence during gyro-bias test (2000 ticks)
# ═══════════════════════════════════════════════════════════════
print("\n" + "="*60)
print("D4: MEKF bias convergence under 5mrad/s gyro step")
print("="*60)

dll.reset()
dll.init(A_GEO, E_GEO, MU, 0.1)
dll.seed_thekf(x0, P0, 0.0)

sim3 = FakeSensorSim(x0, N_GEO, dt=0.01, rng_seed=99)
TRUE_BIAS_STEP = 0.005
last_accel = np.zeros(3)

print(f"\n  {'tick':>5}  {'bias_err(mrad/s)':>16}  "
      f"{'bias[0](mrad/s)':>16}  {'mag_valid':>9}")

for tick in range(2001):
    sf_py = sim3.generate_frame(last_accel)

    if tick >= 800 and sf_py.gyro.valid:
        for i in range(3):
            sf_py.gyro.omega_xyz[i] += TRUE_BIAS_STEP

    dll_sf = dll.get_sensor_frame()
    ctypes.memmove(dll_sf, ctypes.addressof(sf_py),
                   ctypes.sizeof(SensorFrame))

    cf_ptr = dll.step()
    cf = cf_ptr.contents
    last_accel = np.array(list(cf.cmd.accel_lvlh))

    if tick % 100 == 0:
        bias = np.array(list(cf.att.bias_xyz))
        bias_err = np.linalg.norm(bias - TRUE_BIAS_STEP)
        q_norm = np.linalg.norm(np.array(list(cf.att.q_wxyz)))
        print(f"  {tick:>5}  {bias_err*1000:>16.3f}  "
              f"{bias[0]*1000:>16.3f}  {bool(sf_py.mag.valid)!s:>9}  "
              f"q_norm={q_norm:.6f}")

print("\nDone — review above to identify root causes.")