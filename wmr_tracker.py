import collections
import struct
import time
import socket
import hid
import numpy as np
import sys
import os

UDP_IP = "127.0.0.1"
UDP_PORT = 4242

ACCEL_FACTOR = 0.10363 
GYRO_SCALE = 0.001 * 0.125
ACCEL_SCALE = (0.001 * 9.82) * ACCEL_FACTOR 
TICK_SCALE = 1e-7

GRAVITY_TOLERANCE = 1.35 
STABILITY_THRESHOLD = 10
FILTER_GAIN = 0.5  

if os.name == 'nt':
    try: import msvcrt
    except ImportError: pass
else:
    import select; import termios; import tty

def q_mult(q1, q2):
    w1, x1, y1, z1 = q1
    w2, x2, y2, z2 = q2
    return np.array([
        -x1*x2 - y1*y2 - z1*z2 + w1*w2,
         x1*w2 + y1*z2 - z1*y2 + w1*x2,
        -x1*z2 + y1*w2 + z1*x2 + w1*y2,
         x1*y2 - y1*x2 + z1*w2 + w1*z2
    ])

def q_rotate(q, v):
    qx, qy, qz = q[1], q[2], q[3]
    qw = q[0]
    tx = qy*v[2] - qz*v[1]
    ty = qz*v[0] - qx*v[2]
    tz = qx*v[1] - qy*v[0]
    tx += qw * v[0]; ty += qw * v[1]; tz += qw * v[2]
    rx = qy*tz - qz*ty; ry = qz*tx - qx*tz; rz = qx*ty - qy*tx
    return np.array([v[0] + 2.0 * rx, v[1] + 2.0 * ry, v[2] + 2.0 * rz])

def q_from_rotvec(axis, angle):
    half_angle = angle * 0.5
    s = np.sin(half_angle)
    return np.array([np.cos(half_angle), axis[0]*s, axis[1]*s, axis[2]*s])

def q_to_euler(q):
    w, x, y, z = q
    ysqr = y * y
    t0 = +2.0 * (w * x + y * z)
    t1 = +1.0 - 2.0 * (x * x + ysqr)
    roll = np.arctan2(t0, t1)
    t2 = +2.0 * (w * y - z * x)
    t2 = np.clip(t2, -1.0, 1.0)
    pitch = np.arcsin(t2)
    t3 = +2.0 * (w * z + x * y)
    t4 = +1.0 - 2.0 * (ysqr + z * z)
    yaw = np.arctan2(t3, t4)
    return np.degrees(np.array([yaw, pitch, roll]))

class FastFusion:
    def __init__(self, default_bias):
        self.q = np.array([1.0, 0.0, 0.0, 0.0])
        self.iterations = 0
        self.gyro_bias = default_bias
        self.device_level_count = 0
        self.grav_error_angle = 0.0
        self.grav_error_axis = np.zeros(3)
        self.accel_buffer = np.zeros((20, 3))
        self.accel_idx = 0
        self.accel_count = 0
        self.accel_sum = np.zeros(3)

    def set_bias(self, bias):
        self.gyro_bias = bias

    def add_accel_sample(self, vec):
        self.accel_sum -= self.accel_buffer[self.accel_idx]
        self.accel_buffer[self.accel_idx] = vec
        self.accel_sum += vec
        self.accel_idx = (self.accel_idx + 1) % 20
        if self.accel_count < 20: self.accel_count += 1
        
    def get_accel_mean(self):
        if self.accel_count == 0: return np.zeros(3)
        return self.accel_sum / self.accel_count

    def update(self, dt, ang_vel, accel):
        ang_vel_corr = ang_vel - self.gyro_bias
        world_accel = q_rotate(self.q, accel)
        self.iterations += 1
        self.add_accel_sample(world_accel)
        
        ang_vel_length = np.linalg.norm(ang_vel_corr)
        if ang_vel_length > 0.0001:
            rot_axis = ang_vel_corr / ang_vel_length
            rot_angle = ang_vel_length * dt
            delta_q = q_from_rotvec(rot_axis, rot_angle)
            self.q = q_mult(self.q, delta_q)

        accel_mag = np.linalg.norm(accel)
        ang_vel_tolerance = 0.15
        is_stable = (abs(accel_mag - 9.82) < GRAVITY_TOLERANCE and ang_vel_length < ang_vel_tolerance)
        
        if is_stable:
            if self.device_level_count < STABILITY_THRESHOLD * 2:
                self.device_level_count += 1
        else:
            if self.device_level_count > 0:
                self.device_level_count -= 1

        if self.device_level_count > STABILITY_THRESHOLD:
            if self.device_level_count % 5 == 0: 
                accel_mean = self.get_accel_mean()
                if abs(np.linalg.norm(accel_mean) - 9.82) < GRAVITY_TOLERANCE:
                    tilt = np.array([accel_mean[2], 0.0, -accel_mean[0]])
                    tilt_norm = np.linalg.norm(tilt)
                    if tilt_norm > 0: tilt /= tilt_norm
                    
                    accel_mean_norm = accel_mean / np.linalg.norm(accel_mean)
                    up = np.array([0.0, 1.0, 0.0])
                    dot_product = np.clip(np.dot(up, accel_mean_norm), -1.0, 1.0)
                    tilt_angle = np.arccos(dot_product)
                    
                    if tilt_angle > 0.01: 
                        self.grav_error_angle = tilt_angle
                        self.grav_error_axis = tilt

        if self.grav_error_angle > 0.05:
            if self.iterations < 2000:
                use_angle = -self.grav_error_angle * 0.1
                self.grav_error_angle += use_angle
            else:
                factor = FILTER_GAIN * 0.01 * (5.0 * ang_vel_length + 1.0)
                use_angle = -self.grav_error_angle * factor
                self.grav_error_angle += use_angle
            
            corr_q = q_from_rotvec(self.grav_error_axis, use_angle)
            self.q = q_mult(corr_q, self.q)

        n = np.linalg.norm(self.q)
        if n > 0: self.q /= n

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
        if os.name != 'nt': termios.tcsetattr(self.fd, termios.TCSANOW, self.old_term)
    def check_key(self):
        if os.name == 'nt':
            if 'msvcrt' in sys.modules and msvcrt.kbhit():
                try: return msvcrt.getch().decode('utf-8').lower()
                except: return None
        else:
            dr, dw, de = select.select([sys.stdin], [], [], 0)
            if dr: return sys.stdin.read(1).lower()
        return None

def setup_device():
    VID, PID = 0x045E, 0x0659
    try:
        candidates = [d for d in hid.enumerate(VID, PID) if d["product_id"] == PID]
        if not candidates: return None
        
        target_idx = 0
        for arg in sys.argv:
            if arg.isdigit(): target_idx = int(arg); break
            
        path = candidates[target_idx]["path"] if target_idx < len(candidates) else candidates[0]["path"]
        
        hmd = hid.device()
        hmd.open_path(path)
        hmd.write(bytes([0x00, 0x02, 0x07]))
        time.sleep(0.05)
        try: hmd.write(bytes([0x02, 0x07] + [0] * 62))
        except OSError: pass
        return hmd
    except: return None

def calibrate(hmd):
    print("--- CALIBRATION (2s) ---")
    print("Keep device FLAT.")
    packet_struct = struct.Struct("< B 4H 4Q 96h 4Q 12i 4Q")
    collected_gyro = []
    end_time = time.time() + 2.0
    while time.time() < end_time:
        data = hmd.read(497, timeout_ms=50)
        if not data or data[0] != 1: continue
        try:
            unpacked = packet_struct.unpack_from(bytearray(data))
            raw_gyro = np.array(unpacked[9:105], dtype=np.int16)
            for i in range(4):
                s, e = i*8, i*8+8
                g0 = np.sum(raw_gyro[s:e]) 
                g1 = np.sum(raw_gyro[32+s:32+e])
                g2 = np.sum(raw_gyro[64+s:64+e])
                collected_gyro.append([g1 * GYRO_SCALE, g2 * GYRO_SCALE, g0 * GYRO_SCALE])
        except: pass
    if not collected_gyro: return None
    return np.mean(collected_gyro, axis=0)

def run():
    hmd = setup_device()
    if not hmd: 
        print("Device not found."); return

    if "--wake" in sys.argv:
        time.sleep(1.0); hmd.close(); return

    hardcoded_bias = np.array([0.00667593, 0.00111542, -0.00532668])
    fusion = FastFusion(hardcoded_bias)

    fresh_bias = calibrate(hmd)
    if fresh_bias is not None:
        fusion.set_bias(fresh_bias)

    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    packet_struct = struct.Struct("< B 4H 4Q 96h 4Q 12i 4Q")
    udp_struct = struct.Struct("<dddddd")
    udp_buffer = bytearray(48)
    
    offsets = np.zeros(3)
    centered = False
    last_ts = 0

    print("\n--- TRACKING ACTIVE ---")
    print("Press [R] to Center, [Q] to Quit")
    
    try:
        with KeyPoller() as key_poller:
            while True:
                data = hmd.read(497, timeout_ms=20)
                if not data or data[0] != 1: continue
                
                try: tup = packet_struct.unpack_from(bytearray(data))
                except: continue
                
                gyro_timestamps = tup[5:9]
                
                for i in range(4):
                    gyro_ts = gyro_timestamps[i]
                    if last_ts == 0: last_ts = gyro_ts; continue
                    
                    tick_delta = gyro_ts - last_ts
                    if tick_delta < 0: tick_delta += 2**64
                    dt = float(tick_delta) * TICK_SCALE
                    last_ts = gyro_ts
                    if dt <= 0: continue
                    
                    base_g = 9; s = i * 8
                    g0 = sum(tup[base_g + s : base_g + s + 8])
                    g1 = sum(tup[base_g + 32 + s : base_g + 32 + s + 8])
                    g2 = sum(tup[base_g + 64 + s : base_g + 64 + s + 8])
                    
                    base_a = 109
                    a0 = tup[base_a + 0*4 + i]
                    a1 = tup[base_a + 1*4 + i]
                    a2 = tup[base_a + 2*4 + i]
                    
                    gyro_vec = np.array([g1, g2, g0]) * GYRO_SCALE
                    accel_vec = np.array([a1, a2, a0]) * ACCEL_SCALE
                    
                    fusion.update(dt, gyro_vec, accel_vec)

                y, p, r = q_to_euler(fusion.q)
                
                key = key_poller.check_key()
                if key == 'q': break
                if key == 'r': centered = False

                if not centered:
                    offsets = np.array([y, p, r])
                    centered = True
                    print(f">> CENTERED <<                         ", end="\r")

                fy = y - offsets[0]
                fp = p - offsets[1]
                fr = r - offsets[2]

                if fy > 180: fy -= 360
                elif fy < -180: fy += 360

                udp_struct.pack_into(udp_buffer, 0, 0.0, 0.0, 0.0, -fp, fy, -fr)
                sock.sendto(udp_buffer, (UDP_IP, UDP_PORT))

    except KeyboardInterrupt: pass
    finally:
        try: hmd.close()
        except: pass
        sock.close()

if __name__ == "__main__":
    run()