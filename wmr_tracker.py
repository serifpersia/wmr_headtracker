import collections
import socket
import struct
import time
import hid
import numpy as np
from scipy.spatial.transform import Rotation as R
import argparse

FF_USE_GRAVITY = 1
GYRO_BIAS = np.array([0.0, 0.0, 0.0], dtype=np.float32)

ORIENTATIONS = [
    "horizontal_right",
    "horizontal_left",
    "vertical_right",
    "vertical_left",
]

parser = argparse.ArgumentParser(description="3DOF WMR Head Tracker")
parser.add_argument(
    "--orientation",
    type=str,
    choices=ORIENTATIONS,
    default="horizontal_right",
    help=f"Set the board orientation (default: horizontal_right). Choices: {', '.join(ORIENTATIONS)}"
)
args = parser.parse_args()
ORIENTATION = args.orientation

class FilterQueue:
    def __init__(self, size):
        self.elems = collections.deque(maxlen=size)

    def add(self, vec):
        self.elems.append(vec)

    def get_mean(self):
        if not self.elems:
            return np.zeros(3)
        return np.sum(self.elems, axis=0) / len(self.elems)

class Fusion:
    def __init__(self):
        self.orient = R.from_quat([0, 0, 0, 1])
        self.iterations = 0
        self.accel_fq = FilterQueue(20)
        self.flags = FF_USE_GRAVITY
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

        if self.flags & FF_USE_GRAVITY:
            gravity_tolerance, ang_vel_tolerance = 0.4, 0.1
            min_tilt_error, max_tilt_error = 0.05, 0.01

            is_stable = (
                abs(np.linalg.norm(accel) - 9.82) < gravity_tolerance * 2.0
                and ang_vel_length < ang_vel_tolerance
            )
            self.device_level_count = self.device_level_count + 1 if is_stable else 0

            if self.device_level_count > 50:
                self.device_level_count = 0
                accel_mean = self.accel_fq.get_mean()
                if abs(np.linalg.norm(accel_mean) - 9.82) < gravity_tolerance:
                    tilt = np.array([accel_mean[2], 0, -accel_mean[0]])
                    tilt_norm = np.linalg.norm(tilt)
                    if tilt_norm > 0:
                        tilt /= tilt_norm

                    accel_mean_norm_val = np.linalg.norm(accel_mean)
                    if accel_mean_norm_val > 0:
                        accel_mean /= accel_mean_norm_val

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
                    use_angle = -self.grav_gain * self.grav_error_angle * 0.005 * (
                        5.0 * ang_vel_length + 1.0
                    )
                    self.grav_error_angle += use_angle

                corr_quat = R.from_rotvec(self.grav_error_axis * use_angle)
                self.orient = corr_quat * self.orient

        self.orient = R.from_quat(self.orient.as_quat())

    def get_orientation(self):
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
            try:
                path_str = d['path'].decode('utf-8')
            except:
                path_str = str(d['path'])
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
                print("Invalid input. Please enter a number.")

    hmd = hid.device()
    try:
        hmd.open_path(path)
    except Exception as e:
        print(f"Failed to open device: {e}")
        return None

    print("\n--- Waking up device ---")
    hmd.write(bytes([0x00, 0x02, 0x07]))
    time.sleep(0.05)
    try:
        hmd.write(bytes([0x02, 0x07] + [0] * 62))
    except OSError:
        pass
    return hmd

def calibrate_imu(hmd):
    print("\n--- CALIBRATING (KEEP STATIONARY) ---")
    gyro_sum = np.zeros(3)
    samples = 0
    target_samples = 200
    while samples < target_samples:
        data = hmd.read(497, timeout_ms=1000)
        if not data or data[0] != 1: continue
        unpacked = struct.unpack_from("<B 4H 4Q 96h", bytearray(data))
        raw_gyro = np.array(unpacked[9:105], dtype=np.int16).reshape(3, 32)
        pkt_avg = np.mean(raw_gyro, axis=1)
        gyro_sum += pkt_avg
        samples += 1
        print(f"Calibrating: {samples}/{target_samples}", end="\r")
    global GYRO_BIAS
    GYRO_BIAS = gyro_sum / samples
    print(f"\nBias Found: {GYRO_BIAS}")
    print("Starting Tracking...")

def process_packet(data_bytes, fusion_obj, last_ts):
    try:
        unpacked_data = struct.unpack_from("<B 4H 4Q 96h 4Q 12i 4Q", data_bytes)
    except struct.error:
        return None, last_ts

    diagnostics = []
    gyro_timestamps = unpacked_data[5:9]
    raw_gyro = np.array(unpacked_data[9:105], dtype=np.int16).reshape(3, 32)
    raw_accel = np.array(unpacked_data[109:121], dtype=np.int32).reshape(3, 4)

    for i in range(4):
        gyro_ts = gyro_timestamps[i]
        if last_ts == 0:
            last_ts = gyro_ts
            continue
        tick_delta = gyro_ts - last_ts
        if tick_delta < 0:
            tick_delta += 2**64
        dt = float(tick_delta * (1.0 / 10_000_000.0))
        last_ts = gyro_ts
        if dt <= 0:
            continue

        gyro_scale, accel_scale = 0.001 * 0.125, 0.001 * 1.0
        start_idx, end_idx = 8 * i, 8 * i + 8
        sum_c0 = np.sum(raw_gyro[0, start_idx:end_idx]) - (GYRO_BIAS[0] * 8)
        sum_c1 = np.sum(raw_gyro[1, start_idx:end_idx]) - (GYRO_BIAS[1] * 8)
        sum_c2 = np.sum(raw_gyro[2, start_idx:end_idx]) - (GYRO_BIAS[2] * 8)
        gyro_sum = np.array([sum_c0, sum_c2, sum_c1])
        ang_vel = gyro_sum.astype(np.float32) * gyro_scale

        accel = np.array(
            [raw_accel[0, i], raw_accel[2, i], raw_accel[1, i]], dtype=np.float32
        ) * (accel_scale * 9.82)

        fusion_obj.update(dt, ang_vel, accel)
        final_orientation = fusion_obj.get_orientation()
        eulers = final_orientation.as_euler("zyx", degrees=True)
        diagnostics.append({"udp_packet": (eulers[0], -eulers[1], eulers[2])})

    return diagnostics, last_ts

def map_orientation(yaw, pitch, roll, orientation):
    if orientation == "horizontal_right":
        return pitch, roll, -yaw
    elif orientation == "horizontal_left":
        return pitch, -roll, yaw
    elif orientation == "vertical_right":
        return yaw, roll, pitch
    elif orientation == "vertical_left":
        return yaw, -roll, -pitch
    else:
        return pitch, roll, -yaw

def run_tracker():
    hmd = setup_device()
    if not hmd: return
    calibrate_imu(hmd)
    sock, fusion, last_gyro_ts = None, Fusion(), 0
    try:
        IP, PORT = "127.0.0.1", 4242
        sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        print(f"\n--- Starting Tracker --- Sending to {IP}:{PORT}...")

        while True:
            data = hmd.read(497, timeout_ms=100)
            if not data or data[0] != 1:
                continue
            diagnostics, last_gyro_ts = process_packet(bytearray(data), fusion, last_gyro_ts)
            if not diagnostics: continue
            yaw, pitch, roll = diagnostics[-1]["udp_packet"]
            pitch, roll, yaw = map_orientation(yaw, pitch, roll, ORIENTATION)
            packet = struct.pack("<dddddd", 0.0, 0.0, 0.0, pitch, roll, yaw)
            sock.sendto(packet, (IP, PORT))
    except KeyboardInterrupt:
        print("\nStopping...")
    finally:
        if hmd: hmd.close()
        if sock: sock.close()

if __name__ == "__main__":
    run_tracker()
