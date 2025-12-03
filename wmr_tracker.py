import collections
import struct
import time
import socket
import hid
import numpy as np
import sys
import os

if os.name == 'nt':
    import msvcrt
else:
    import select
    import termios
    import tty

from scipy.spatial.transform import Rotation as R

ACCEL_FACTOR = 0.10395

class KeyPoller:
    def __enter__(self):
        if os.name != 'nt':
            self.fd = sys.stdin.fileno()
            self.old_term = termios.tcgetattr(self.fd)
            new_term = termios.tcgetattr(self.fd)
            new_term[3] = new_term[3] & ~termios.ICANON & ~termios.ECHO
            termios.tcsetattr(self.fd, termios.TCSANOW, new_term)
        return self

    def __exit__(self, type, value, traceback):
        if os.name != 'nt':
            termios.tcsetattr(self.fd, termios.TCSANOW, self.old_term)

    def check_key(self):
        if os.name == 'nt':
            if msvcrt.kbhit():
                try:
                    ch = msvcrt.getch().decode('utf-8').lower()
                    return ch
                except: return None
        else:
            dr, dw, de = select.select([sys.stdin], [], [], 0)
            if dr:
                return sys.stdin.read(1).lower()
        return None

class Fusion:
    def __init__(self):
        self.q = np.array([0.0, 0.0, 0.0, 1.0])
        self.gyro_bias = np.zeros(3)
        self.alpha = 0.98

    def set_bias(self, bias_vec):
        self.gyro_bias = bias_vec

    def update(self, dt, gyro, accel):
        gyro_corr = gyro - self.gyro_bias
        q_w, q_x, q_y, q_z = self.q[3], self.q[0], self.q[1], self.q[2]
        gx, gy, gz = gyro_corr

        dq = np.array([
            0.5 * (q_w * gx + q_y * gz - q_z * gy),
            0.5 * (q_w * gy - q_x * gz + q_z * gx),
            0.5 * (q_w * gz + q_x * gy - q_y * gx),
            0.5 * (-q_x * gx - q_y * gy - q_z * gz)
        ])

        self.q += dq * dt
        self.q /= np.linalg.norm(self.q)

        accel_norm = np.linalg.norm(accel)
        if 0.5 < accel_norm < 1.5:
            a_norm = accel / accel_norm
            r = R.from_quat(self.q)
            estimated_g = r.inv().apply(np.array([0, 1, 0]))

            error_axis = np.cross(a_norm, estimated_g)
            corr_factor = (1.0 - self.alpha) * dt * 5.0
            corr_q = R.from_rotvec(error_axis * corr_factor)

            self.q = (corr_q * r).as_quat()
            self.q /= np.linalg.norm(self.q)

    def get_euler(self):
        return R.from_quat(self.q).as_euler("zyx", degrees=True)

def setup_device():
    VID, PID = 0x045E, 0x0659
    try:
        candidates = [d for d in hid.enumerate(VID, PID) if d["product_id"] == PID]
    except: return None

    if not candidates: return None
    path = candidates[0]["path"]

    hmd = hid.device()
    try: hmd.open_path(path)
    except: return None

    hmd.write(bytes([0x00, 0x02, 0x07]))
    time.sleep(0.05)
    try: hmd.write(bytes([0x02, 0x07] + [0] * 62))
    except OSError: pass
    return hmd

def parse_packet(data_bytes, last_ts):
    try:
        unpacked = struct.unpack_from("<B 4H 4Q 96h 4Q 12i 4Q", data_bytes)
    except struct.error: return [], last_ts

    gyro_ts_list = unpacked[5:9]
    raw_gyro = np.array(unpacked[9:105], dtype=np.int16).reshape(3, 32)
    raw_accel = np.array(unpacked[109:121], dtype=np.int32).reshape(3, 4)

    packets = []
    gyro_scale = 0.001 * 0.125
    accel_scale = (0.001 * 9.82) * ACCEL_FACTOR

    for i in range(4):
        ts = gyro_ts_list[i]
        if last_ts == 0:
            last_ts = ts
            continue

        tick = ts - last_ts
        if tick < 0: tick += 2**64
        dt = tick * 1e-7
        last_ts = ts

        if dt <= 0: continue

        start, end = 8*i, 8*i+8
        g_sum = np.sum(raw_gyro[:, start:end], axis=1)

        gyro_vec = np.array([-g_sum[1], -g_sum[0], -g_sum[2]]) * gyro_scale
        accel_vec = np.array([-raw_accel[1,i], -raw_accel[0,i], -raw_accel[2,i]]) * accel_scale

        packets.append((dt, gyro_vec, accel_vec))

    return packets, last_ts

def calibrate(hmd):
    print("--- CALIBRATION ---")
    print("Keep device FLAT and STILL on the table.")
    print("Collecting data in 1 second...")
    time.sleep(1.0)

    end_time = time.time() + 3.0
    gyro_data = []
    last_ts = 0

    while time.time() < end_time:
        d = hmd.read(497, timeout_ms=100)
        if not d or d[0] != 1: continue
        pkts, last_ts = parse_packet(bytearray(d), last_ts)
        for _, g, _ in pkts:
            gyro_data.append(g)

    if not gyro_data: return np.zeros(3)
    bias = np.mean(gyro_data, axis=0)
    print(f"Bias calculated: {bias}")
    return bias

def run():
    hmd = setup_device()
    if not hmd:
        print("Device not found.")
        return

    bias = calibrate(hmd)
    fusion = Fusion()
    fusion.set_bias(bias)

    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    IP, PORT = "127.0.0.1", 4242

    offsets = np.zeros(3)
    centered = False
    last_ts = 0

    print("\n--- TRACKING STARTED ---")
    print("1. Put the cap on your head.")
    print("2. Look at the screen (Center).")
    print("3. Press 'r' or Enter to ZERO the view.")
    print("   (Press 'r' again at any time to re-center)")
    print("   (Press 'q' or Ctrl+C to quit)")

    try:
        with KeyPoller() as key_poller:
            while True:
                data = hmd.read(497, timeout_ms=100)
                if not data or data[0] != 1: continue

                packets, last_ts = parse_packet(bytearray(data), last_ts)

                for dt, gyro, accel in packets:
                    fusion.update(dt, gyro, accel)
                    y, p, r = fusion.get_euler()

                    key = key_poller.check_key()
                    if key == 'q':
                        raise KeyboardInterrupt

                    if key == 'r' or key == '\n' or key == '\r' or not centered:
                        if key:
                            offsets = np.array([y, p, r])
                            centered = True
                            print(f"\r>> RE-CENTERED! [{y:.1f}, {p:.1f}, {r:.1f}]   ", end="")

                    if centered:
                        final_y = y - offsets[0]
                        final_p = p - offsets[1]
                        final_r = r - offsets[2]

                        packet = struct.pack("<dddddd", 0.0, 0.0, 0.0, final_y, -final_p, final_r)
                        sock.sendto(packet, (IP, PORT))

    except KeyboardInterrupt:
        print("\nStopping...")
    finally:
        hmd.close()
        sock.close()

if __name__ == "__main__":
    run()
