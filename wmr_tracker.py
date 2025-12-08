import struct
import time
import socket
import hid
import math
import sys
import os

UDP_IP = "127.0.0.1"
UDP_PORT = 4242

ACCEL_FACTOR = 0.10398
ACCEL_SCALE = 0.00982 * ACCEL_FACTOR
GYRO_SCALE = 0.000125
TICK_SCALE = 1e-7

GRAVITY_TOLERANCE = 1.35
STABILITY_THRESHOLD = 10
FILTER_GAIN = 0.5
GYRO_DEADZONE = 0.0035

if os.name == 'nt':
    try: import msvcrt
    except ImportError: pass
else:
    import select; import termios; import tty

def normalize_q(q):
    n = math.sqrt(q[0]*q[0] + q[1]*q[1] + q[2]*q[2] + q[3]*q[3])
    if n > 0:
        inv = 1.0 / n
        q[0] *= inv; q[1] *= inv; q[2] *= inv; q[3] *= inv

def q_mult_inplace(q1, w2, x2, y2, z2):
    w1, x1, y1, z1 = q1
    q1[0] = w1*w2 - x1*x2 - y1*y2 - z1*z2
    q1[1] = w1*x2 + x1*w2 + y1*z2 - z1*y2
    q1[2] = w1*y2 - x1*z2 + y1*w2 + z1*x2
    q1[3] = w1*z2 + x1*y2 - y1*x2 + z1*w2

def q_rotate(q, v, out):
    qw, qx, qy, qz = q
    vx, vy, vz = v
    tx = qy*vz - qz*vy
    ty = qz*vx - qx*vz
    tz = qx*vy - qy*vx
    tx += qw*vx
    ty += qw*vy
    tz += qw*vz
    rx = qy*tz - qz*ty
    ry = qz*tx - qx*tz
    rz = qx*ty - qy*tx
    out[0] = vx + 2.0 * rx
    out[1] = vy + 2.0 * ry
    out[2] = vz + 2.0 * rz

def q_to_euler(q):
    w, x, y, z = q
    ysqr = x * x
    t3 = 2.0 * (w * y + z * x)
    t4 = 1.0 - 2.0 * (ysqr + y * y)
    yaw = math.degrees(math.atan2(t3, t4))
    
    t2 = 2.0 * (w * x - y * z)
    t2 = 1.0 if t2 > 1.0 else (-1.0 if t2 < -1.0 else t2)
    pitch = math.degrees(math.asin(t2))
    
    t0 = 2.0 * (w * z + x * y)
    t1 = 1.0 - 2.0 * (z * z + x * x)
    roll = math.degrees(math.atan2(t0, t1))
    
    return yaw, pitch, roll

class FastFusion:
    def __init__(self, default_bias):
        self.q = [1.0, 0.0, 0.0, 0.0]
        self.gyro_bias = default_bias
        self.iterations = 0
        self.device_level_count = 0
        self.grav_error_angle = 0.0
        self.grav_error_axis = [0.0, 0.0, 0.0]
        self.accel_buffer = [[0.0, 0.0, 0.0] for _ in range(20)]
        self.accel_idx = 0
        self.accel_count = 0
        self.accel_sum = [0.0, 0.0, 0.0]
        self.world_accel_cache = [0.0, 0.0, 0.0]

    def update(self, dt, gx, gy, gz, ax, ay, az):
        gx -= self.gyro_bias[0]
        gy -= self.gyro_bias[1]
        gz -= self.gyro_bias[2]

        g_mag_sq = gx*gx + gy*gy + gz*gz
        if g_mag_sq < GYRO_DEADZONE * GYRO_DEADZONE:
            gx = gy = gz = 0.0
            g_mag = 0.0
        else:
            g_mag = math.sqrt(g_mag_sq)

        if g_mag > 0.000001:
            half_angle = g_mag * dt * 0.5
            s = math.sin(half_angle) / g_mag
            dq_w = math.cos(half_angle)
            dq_x = gx * s
            dq_y = gy * s
            dq_z = gz * s
            q_mult_inplace(self.q, dq_w, dq_x, dq_y, dq_z)
            normalize_q(self.q)

        q_rotate(self.q, (ax, ay, az), self.world_accel_cache)
        wax, way, waz = self.world_accel_cache

        self.iterations += 1
        
        old = self.accel_buffer[self.accel_idx]
        self.accel_sum[0] += wax - old[0]
        self.accel_sum[1] += way - old[1]
        self.accel_sum[2] += waz - old[2]
        
        old[0], old[1], old[2] = wax, way, waz
        self.accel_idx = (self.accel_idx + 1) % 20
        if self.accel_count < 20: self.accel_count += 1

        a_mag = math.sqrt(ax*ax + ay*ay + az*az)
        is_stable = (abs(a_mag - 9.82) < GRAVITY_TOLERANCE and g_mag < 0.15)

        if is_stable:
            if self.device_level_count < STABILITY_THRESHOLD * 2:
                self.device_level_count += 1
        else:
            if self.device_level_count > 0:
                self.device_level_count -= 1

        if self.device_level_count > STABILITY_THRESHOLD:
            if self.device_level_count % 5 == 0:
                inv_count = 1.0 / self.accel_count
                mx = self.accel_sum[0] * inv_count
                my = self.accel_sum[1] * inv_count
                mz = self.accel_sum[2] * inv_count
                
                m_norm = math.sqrt(mx*mx + my*my + mz*mz)
                
                if abs(m_norm - 9.82) < GRAVITY_TOLERANCE:
                    tx, ty, tz = mz, 0.0, -mx
                    t_norm = math.sqrt(tx*tx + ty*ty + tz*tz)
                    if t_norm > 0:
                        inv_t = 1.0 / t_norm
                        tx *= inv_t; ty *= inv_t; tz *= inv_t

                    dot = my / m_norm
                    if dot > 1.0: dot = 1.0
                    elif dot < -1.0: dot = -1.0
                    
                    tilt_angle = math.acos(dot)

                    if tilt_angle > 0.01:
                        self.grav_error_angle = tilt_angle
                        self.grav_error_axis[0] = tx
                        self.grav_error_axis[1] = ty
                        self.grav_error_axis[2] = tz

        if self.grav_error_angle > 0.05:
            if self.iterations < 2000:
                use_angle = -self.grav_error_angle * 0.1
            else:
                use_angle = -self.grav_error_angle * (FILTER_GAIN * 0.01 * (5.0 * g_mag + 1.0))
            
            self.grav_error_angle += use_angle
            
            half_angle = use_angle * 0.5
            s = math.sin(half_angle)
            c = math.cos(half_angle)
            
            rx = self.grav_error_axis[0] * s
            ry = self.grav_error_axis[1] * s
            rz = self.grav_error_axis[2] * s
            
            qw, qx, qy, qz = self.q
            self.q[0] = c*qw - rx*qx - ry*qy - rz*qz
            self.q[1] = c*qx + rx*qw + ry*qz - rz*qy
            self.q[2] = c*qy - rx*qz + ry*qw + rz*qx
            self.q[3] = c*qz + rx*qy - ry*qx + rz*qw
            
            normalize_q(self.q)

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
        if not candidates: return None, None
        
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
        return hmd, path
    except: return None, None

def calibrate_gyro(hmd, packet_struct):
    sys.stdout.write("\033[2J\033[H")
    print("------------------------------------------------")
    print("  CALIBRATING... KEEP DEVICE STILL (2 SEC)      ")
    print("------------------------------------------------")
    
    samples = []
    start = time.time()
    
    while time.time() - start < 2.0:
        data = hmd.read(497, timeout_ms=20)
        if not data or data[0] != 1: continue
        try: 
            tup = packet_struct.unpack_from(bytearray(data))
            for i in range(4):
                base_g = 9; s = i * 8
                g0 = sum(tup[base_g + s : base_g + s + 8])
                g1 = sum(tup[base_g + 32 + s : base_g + 32 + s + 8])
                g2 = sum(tup[base_g + 64 + s : base_g + 64 + s + 8])
                samples.append((g1, g2, g0))
        except: continue
        
        p = int((time.time() - start) * 20)
        sys.stdout.write(f"\r[{'=' * p}{' ' * (40 - p)}] {len(samples)} samples")
        sys.stdout.flush()

    if not samples: return [0.0, 0.0, 0.0]
    
    sums = [0.0, 0.0, 0.0]
    for s in samples:
        sums[0] += s[0]; sums[1] += s[1]; sums[2] += s[2]
    
    count = len(samples)
    bias = [
        (sums[0] / count) * GYRO_SCALE,
        (sums[1] / count) * GYRO_SCALE,
        (sums[2] / count) * GYRO_SCALE
    ]
    return bias

def run():
    hmd, path = setup_device()
    if not hmd: print("Device not found."); return

    if "--wake" in sys.argv:
        time.sleep(1.0); hmd.close(); return

    packet_struct = struct.Struct("< B 4H 4Q 96h 4Q 12i 4Q")
    udp_struct = struct.Struct("<dddddd")
    udp_buffer = bytearray(48)

    bias = calibrate_gyro(hmd, packet_struct)
    print("\nCalibration complete. Driver active.")
    print("Press [Q] to Quit, [R] to Center.")
    
    fusion = FastFusion(bias)
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    
    off_y, off_p, off_r = 0.0, 0.0, 0.0
    last_ts = 0
    
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
                    
                    fusion.update(dt, 
                                  g1 * GYRO_SCALE, g2 * GYRO_SCALE, g0 * GYRO_SCALE, 
                                  a1 * ACCEL_SCALE, a2 * ACCEL_SCALE, a0 * ACCEL_SCALE)

                y, p, r = q_to_euler(fusion.q)
                
                key = key_poller.check_key()
                if key == 'q': break
                if key == 'r': off_y, off_p, off_r = y, p, r

                fy = y - off_y
                fp = p - off_p
                fr = r - off_r
                udp_struct.pack_into(udp_buffer, 0, 0.0, 0.0, 0.0, -fy, fr, -fp)
                sock.sendto(udp_buffer, (UDP_IP, UDP_PORT))

    except KeyboardInterrupt: pass
    finally:
        try: hmd.close()
        except: pass
        sock.close()
        print("\nDriver Closed.")

if __name__ == "__main__":
    run()