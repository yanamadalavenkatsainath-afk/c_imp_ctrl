#!/usr/bin/env python3
"""Python plant <-> target hardware closed-loop runner over a serial link.

Protocol v1:
  host sends:  0xA55A5AA5, uint16 payload_len, SensorFrame bytes
  target sends:0x5AA5A55A, uint16 payload_len, CommandFrame bytes

The target side should deserialize SensorFrame, call flight_loop_step(), then
serialize CommandFrame. Sensor packets already carry per-packet checksums and
timestamps; the transport header only frames whole messages.
"""

from __future__ import annotations

import argparse
import ctypes
import struct
import time

import numpy as np

from sim_python.closed_loop_sil import PhysicsPlantSim, DT_FAST
from sim_python.realtime_driver import CommandFrame, SensorFrame, stamp_frame_valid_packets

HOST_MAGIC = 0xA55A5AA5
TARGET_MAGIC = 0x5AA5A55A
HEADER = struct.Struct("<IH")


def _read_exact(port, n: int) -> bytes:
    data = bytearray()
    while len(data) < n:
        chunk = port.read(n - len(data))
        if not chunk:
            raise TimeoutError(f"serial timeout while reading {n} bytes")
        data.extend(chunk)
    return bytes(data)


def _send_frame(port, sf: SensorFrame) -> None:
    payload = ctypes.string_at(ctypes.addressof(sf), ctypes.sizeof(SensorFrame))
    port.write(HEADER.pack(HOST_MAGIC, len(payload)) + payload)


def _recv_command(port) -> CommandFrame:
    magic, n = HEADER.unpack(_read_exact(port, HEADER.size))
    if magic != TARGET_MAGIC:
        raise RuntimeError(f"bad target magic 0x{magic:08X}")
    if n != ctypes.sizeof(CommandFrame):
        raise RuntimeError(f"bad CommandFrame size {n}, expected {ctypes.sizeof(CommandFrame)}")
    payload = _read_exact(port, n)
    cf = CommandFrame()
    ctypes.memmove(ctypes.addressof(cf), payload, n)
    return cf


def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("--port", required=True, help="Serial device, e.g. COM5")
    ap.add_argument("--baud", type=int, default=921600)
    ap.add_argument("--ticks", type=int, default=6000)
    ap.add_argument("--timeout", type=float, default=0.25)
    args = ap.parse_args()

    try:
        import serial
    except ImportError as exc:
        raise SystemExit("pyserial is required: python -m pip install pyserial") from exc

    plant = PhysicsPlantSim()
    accel = np.zeros(3)
    torque = np.zeros(3)
    dipole = np.zeros(3)
    late = 0

    with serial.Serial(args.port, args.baud, timeout=args.timeout) as port:
        t_next = time.perf_counter()
        for tick in range(args.ticks):
            sf = plant.step(DT_FAST, accel, torque, dipole)
            stamp_frame_valid_packets(sf)
            t0 = time.perf_counter()
            _send_frame(port, sf)
            cf = _recv_command(port)
            elapsed = time.perf_counter() - t0
            if elapsed > DT_FAST:
                late += 1

            accel = np.array(list(cf.cmd.accel_lvlh))
            torque = np.array(list(cf.cmd.torque_rw))
            dipole = np.array(list(cf.cmd.dipole_mtq))

            if tick % 100 == 0:
                print(
                    f"tick={tick:6d} mode={cf.cmd.fsw_mode}/{cf.cmd.rpod_mode} "
                    f"range={cf.nav.range_m:8.2f}m loop_rt={elapsed*1e3:6.2f}ms "
                    f"wd=0x{cf.timing.watchdog_flags:08X}"
                )

            t_next += DT_FAST
            sleep_s = t_next - time.perf_counter()
            if sleep_s > 0:
                time.sleep(sleep_s)

    print(f"HIL run complete: ticks={args.ticks}, host-late-frames={late}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
