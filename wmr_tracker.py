import collections
import struct
import time
import socket
import hid
import numpy as np
from scipy.spatial.transform import Rotation as R

ACCEL_FACTOR = 9.82 / 93.1336
GYRO_BIAS_X = -0.010476
GYRO_BIAS_Y = 0.003317
GYRO_BIAS_Z = -0.003876

class FilterQueue:
    def __init__(self, size):
        self.elems = collections.deque(maxlen=size)
    def add(self, vec):
        self.elems.append(vec)
    def get_mean(self):
        if not self.elems: return np.zeros(3)
        return np.sum(self.elems, axis=0) / len(self.elems)

class Fusion:
    def __init__(self):
        self.orient = R.from_quat([0, 0, 0, 1])
        self.iterations = 0
        self.accel_fq = FilterQueue(20)
        self.grav_gain = 0.05
        self.device_level_count = 0
        self.grav_error_angle = 0.0
        self.grav_error_axis = np.zeros(3)

    def update(self, dt, ang_vel, accel):
        world_accel = self.orient.apply(accel)
        self.iterations += 1
        self.accel_fq.add(world_accel)
        ang_vel_length = np.linalg.norm(ang_vel)

        if ang_vel_length > 0.0001:
            rot_axis = ang_vel / ang_vel_length
            rot_angle = ang_vel_length * dt
            delta_orient = R.from_rotvec(rot_axis * rot_angle)
            self.orient = self.orient * delta_orient

        gravity_tolerance, ang_vel_tolerance = 0.4, 0.1
        min_tilt_error, max_tilt_error = 0.05, 0.01
        
        is_stable = (abs(np.linalg.norm(accel) - 9.82) < gravity_tolerance * 2.0 and ang_vel_length < ang_vel_tolerance)
        self.device_level_count = self.device_level_count + 1 if is_stable else 0

        if self.device_level_count > 50:
            self.device_level_count = 0
            accel_mean = self.accel_fq.get_mean()
            if abs(np.linalg.norm(accel_mean) - 9.82) < gravity_tolerance:
                tilt = np.array([accel_mean[2], 0, -accel_mean[0]])
                tilt_norm = np.linalg.norm(tilt)
                if tilt_norm > 0: tilt /= tilt_norm
                accel_mean /= np.linalg.norm(accel_mean)
                up = np.array([0, 1.0, 0])
                dot_product = np.clip(np.dot(up, accel_mean), -1.0, 1.0)
                tilt_angle = np.arccos(dot_product)
                if tilt_angle > max_tilt_error:
                    self.grav_error_angle, self.grav_error_axis = tilt_angle, tilt

        if self.grav_error_angle > min_tilt_error:
            if self.iterations < 2000:
                use_angle = -self.grav_error_angle
                self.grav_error_angle = 0
            else:
                use_angle = -self.grav_gain * self.grav_error_angle * 0.005 * (5.0 * ang_vel_length + 1.0)
                self.grav_error_angle += use_angle
            corr_quat = R.from_rotvec(self.grav_error_axis * use_angle)
            self.orient = corr_quat * self.orient

        self.orient = R.from_quat(self.orient.as_quat() / np.linalg.norm(self.orient.as_quat()))

    def get_orientation_object(self):
        return self.orient

def setup_device():
    VID, PID = 0x045E, 0x0659
    print("Scanning for WMR Sensor interfaces...")
    try:
        candidates = [d for d in hid.enumerate(VID, PID) if d["product_id"] == PID]
    except Exception as e:
        print(f"HID Error: {e}")
        return None
    if not candidates:
        print("No WMR sensors found.")
        return None
    
    path = None
    if len(candidates) == 1:
        path = candidates[0]["path"]
        print(f"Found: {candidates[0]['product_string']}")
    else:
        print("\nMultiple devices found:")
        for i, d in enumerate(candidates):
            try: path_str = d['path'].decode('utf-8')
            except: path_str = str(d['path'])
            print(f"[{i}] {d['product_string']} (Path: {path_str})")
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

    hmd = hid.device()
    try: hmd.open_path(path)
    except Exception as e: print(f"Failed to open device: {e}"); return None
    
    hmd.write(bytes([0x00, 0x02, 0x07]))
    time.sleep(0.05)
    try: hmd.write(bytes([0x02, 0x07] + [0] * 62))
    except OSError: pass
    return hmd

def process_packet(data_bytes, fusion_obj, last_ts):
    try:
        unpacked_data = struct.unpack_from("<B 4H 4Q 96h 4Q 12i 4Q", data_bytes)
    except struct.error: return None, last_ts
    gyro_timestamps = unpacked_data[5:9]
    raw_gyro = np.array(unpacked_data[9:105], dtype=np.int16).reshape(3, 32)
    raw_accel = np.array(unpacked_data[109:121], dtype=np.int32).reshape(3, 4)
    current_orient = None
    
    for i in range(4):
        gyro_ts = gyro_timestamps[i]
        if last_ts == 0: last_ts = gyro_ts; continue
        tick_delta = gyro_ts - last_ts
        if tick_delta < 0: tick_delta += 2**64
        dt = float(tick_delta * 1e-7)
        last_ts = gyro_ts
        if dt <= 0: continue
        
        gyro_scale = 0.001 * 0.125
        accel_scale = (0.001 * 9.82) * ACCEL_FACTOR
        
        start_idx, end_idx = 8*i, 8*i+8
        c0 = np.sum(raw_gyro[0, start_idx:end_idx])
        c1 = np.sum(raw_gyro[1, start_idx:end_idx])
        c2 = np.sum(raw_gyro[2, start_idx:end_idx])
        
        gyro_mapped = np.array([-c1, -c0, -c2])
        ang_vel = gyro_mapped.astype(np.float32) * gyro_scale
        
        ang_vel[0] -= GYRO_BIAS_X
        ang_vel[1] -= GYRO_BIAS_Y
        ang_vel[2] -= GYRO_BIAS_Z
        
        accel_mapped = np.array([-raw_accel[1,i], -raw_accel[0,i], -raw_accel[2,i]], dtype=np.float32)
        accel = accel_mapped * accel_scale
        
        fusion_obj.update(dt, ang_vel, accel)
        current_orient = fusion_obj.get_orientation_object()
        
    return current_orient, last_ts

def run_tracker():
    hmd = setup_device()
    if not hmd: return
    fusion, last_gyro_ts = Fusion(), 0
    IP, PORT = "127.0.0.1", 4242
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    
    offsets = np.zeros(3)
    calibrated = False
    start_time = time.time()
    warmup_time = 2.0
    
    print(f"\n--- Tracker Running ---")
    print(f"Keep device FLAT on table. Calibrating view in {warmup_time} seconds...")
    
    try:
        while True:
            data = hmd.read(497, timeout_ms=100)
            if not data or data[0] != 1: continue
            
            orient_obj, last_gyro_ts = process_packet(bytearray(data), fusion, last_gyro_ts)
            
            if orient_obj is not None:
                yaw, pitch, roll = orient_obj.as_euler("zyx", degrees=True)
                
                if not calibrated and (time.time() - start_time) > warmup_time:
                    offsets = np.array([yaw, pitch, roll])
                    calibrated = True
                    print(f"!!! ZEROED !!! (Offsets: {offsets})")
                    print("You can now pick up the device.")

                if calibrated:
                    final_yaw = yaw - offsets[0]
                    final_pitch = pitch - offsets[1]
                    final_roll = roll - offsets[2]
                    
                    packet = struct.pack("<dddddd", 0.0, 0.0, 0.0, final_yaw, -final_pitch, final_roll)
                    sock.sendto(packet, (IP, PORT))
                    
    except KeyboardInterrupt: print("\nStopping...")
    finally:
        if hmd: hmd.close()
        if sock: sock.close()

if __name__ == "__main__":
    run_tracker()