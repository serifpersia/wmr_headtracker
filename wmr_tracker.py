import struct
import time
import socket
import hid
import numpy as np
import sys
import os
from collections import deque
from scipy.spatial.transform import Rotation as R

if os.name == 'nt':
    try:
        import msvcrt
    except ImportError:
        pass
else:
    import select
    import termios
    import tty

ACCEL_FACTOR = 0.10395
GYRO_SCALE = 0.001 * 0.125
ACCEL_SCALE = (0.001 * 9.82) * ACCEL_FACTOR 

UDP_IP = "127.0.0.1"
UDP_PORT = 4242

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
            if 'msvcrt' in sys.modules and msvcrt.kbhit():
                try:
                    return msvcrt.getch().decode('utf-8').lower()
                except: return None
        else:
            dr, dw, de = select.select([sys.stdin], [], [], 0)
            if dr: return sys.stdin.read(1).lower()
        return None

class HorizontalFusion:
    def __init__(self):
        self.q = np.array([0.0, 0.0, 0.0, 1.0]) 
        self.gyro_bias = np.zeros(3)
        self.alpha = 0.02 

    def set_bias(self, bias):
        self.gyro_bias = bias

    def update(self, dt, gyro, accel):
        g_corr = gyro - self.gyro_bias
        r_gyro = R.from_rotvec(g_corr * dt)
        self.q = (r_gyro * R.from_quat(self.q)).as_quat()
        self.q /= np.linalg.norm(self.q)

        accel_mag = np.linalg.norm(accel)
        
        if 8.5 < accel_mag < 11.5:
            a_norm = accel / accel_mag
            current_r = R.from_quat(self.q)
            
            estimated_gravity = current_r.inv().apply(np.array([0.0, 0.0, 1.0]))
            
            error_axis = np.cross(estimated_gravity, a_norm)
            
            correction = R.from_rotvec(error_axis * self.alpha)
            self.q = (correction * R.from_quat(self.q)).as_quat()
            self.q /= np.linalg.norm(self.q)

    def get_opentrack_angles(self):
        angles = R.from_quat(self.q).as_euler("xyz", degrees=True)
        
        return angles[2], angles[0], angles[1]

def setup_device():
    VID, PID = 0x045E, 0x0659
    print("Scanning for WMR Sensor...")
    try:
        candidates = [d for d in hid.enumerate(VID, PID) if d["product_id"] == PID]
    except Exception as e:
        print(f"HID Error: {e}")
        return None
    
    if not candidates:
        print("No WMR sensors found.")
        return None
    
    path = None
    
    target_idx = -1
    for arg in sys.argv:
        if arg.isdigit():
            target_idx = int(arg)
            break

    if len(candidates) > 1:
        if target_idx != -1 and target_idx < len(candidates):
            print(f"Auto-selecting Index {target_idx}")
            path = candidates[target_idx]["path"]
        elif os.name == 'nt':
            print("\nMultiple devices found:")
            for i, d in enumerate(candidates):
                path_str = d['path'].decode('utf-8') if isinstance(d['path'], bytes) else str(d['path'])
                print(f"[{i}] {d['product_string']} (Path: ...{path_str[-10:]})")
            
            print("On Windows, usually Index 0 or 1 is the active sensor.")
            while True:
                try:
                    sel = input("Select device index: ")
                    idx = int(sel)
                    if 0 <= idx < len(candidates):
                        path = candidates[idx]["path"]
                        break
                    else:
                        print("Index out of range.")
                except ValueError:
                    print("Invalid input.")
        else:
            print("Auto-selecting first device.")
            path = candidates[0]["path"]
    else:
        path = candidates[0]["path"]
        print(f"Found: {candidates[0]['product_string']}")

    hmd = hid.device()
    try: hmd.open_path(path)
    except Exception as e: 
        print(f"Failed to open device: {e}")
        return None

    # Wake up sensor
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

        gx = g_sum[0] * GYRO_SCALE 
        gy = -g_sum[1] * GYRO_SCALE
        gz = g_sum[2] * GYRO_SCALE
        
        ax = raw_accel[0,i] * ACCEL_SCALE
        ay = -raw_accel[1,i] * ACCEL_SCALE
        az = raw_accel[2,i] * ACCEL_SCALE

        packets.append((dt, np.array([gx, gy, gz]), np.array([ax, ay, az])))

    return packets, last_ts

def calibrate(hmd):
    print("--- CALIBRATION ---")
    print("Keep device FLAT and STATIONARY.")
    time.sleep(0.5)
    
    end_time = time.time() + 2.0
    gyro_data = []
    last_ts = 0

    while time.time() < end_time:
        d = hmd.read(497, timeout_ms=100)
        if not d or d[0] != 1: continue
        pkts, last_ts = parse_packet(bytearray(d), last_ts)
        for _, g, _ in pkts: gyro_data.append(g)

    if not gyro_data: return np.zeros(3)
    bias = np.mean(gyro_data, axis=0)
    print(f"Bias: {bias}")
    return bias

def run():
    hmd = setup_device()
    if not hmd:
        return

    if "--wake" in sys.argv:
        print("Device initialized (Wake Mode). Exiting...")
        time.sleep(1.0)
        hmd.close()
        return

    bias = calibrate(hmd)
    fusion = HorizontalFusion()
    fusion.set_bias(bias)

    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    
    yaw_off, pitch_off, roll_off = 0, 0, 0
    centered = False
    last_ts = 0

    print("\n--- TRACKING ACTIVE ---")
    print("Press [R] to Center, [Q] to Quit")
    
    try:
        with KeyPoller() as key_poller:
            while True:
                data = hmd.read(497, timeout_ms=100)
                if not data or data[0] != 1: continue

                packets, last_ts = parse_packet(bytearray(data), last_ts)

                for dt, gyro, accel in packets:
                    fusion.update(dt, gyro, accel)
                    
                    y, p, r = fusion.get_opentrack_angles()

                    key = key_poller.check_key()
                    if key == 'q': raise KeyboardInterrupt
                    if key == 'r': centered = False

                    if not centered:
                        yaw_off = y
                        pitch_off = p
                        roll_off = r
                        centered = True
                        print(f">> CENTERED << ", end="\r")

                    fy = y - yaw_off
                    fp = p - pitch_off
                    fr = r - roll_off

                    if fy > 180: fy -= 360
                    if fy < -180: fy += 360

                    packet = struct.pack("<dddddd", 0.0, 0.0, 0.0, fy, fp, fr)
                    sock.sendto(packet, (UDP_IP, UDP_PORT))

    except KeyboardInterrupt:
        print("\nStopping...")
    finally:
        hmd.close()
        sock.close()

if __name__ == "__main__":
    run()
