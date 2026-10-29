import numpy as np
import time
import os
import sys
from coppeliasim_zmqremoteapi_client import RemoteAPIClient
from pynput import keyboard

sys.stdout.reconfigure(line_buffering=True)

BASE_DIR = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
RESULTS_DIR = os.path.join(BASE_DIR, 'results')
os.makedirs(RESULTS_DIR, exist_ok=True)
os.makedirs('results', exist_ok=True)

print("Menghubungkan ke CoppeliaSim Remote API...")
client = RemoteAPIClient()
sim = client.require('sim')
sim.setStepping(False)

def get_handle(name_candidates):
    for name in name_candidates:
        try:
            return sim.getObject(name)
        except Exception:
            continue
    raise RuntimeError(f"Tidak dapat menemukan objek: {name_candidates}")

target = get_handle(['/target', '/Quadcopter/target', ':/target'])
cam = get_handle(['/sensor', '/Quadcopter/fast3DLaserScanner/body/sensor', '/Quadcopter/fast3DLaserScanner/sensor'])

res = sim.getVisionSensorRes(cam) if hasattr(sim, 'getVisionSensorRes') else [64, 64]
W, H = int(res[0]), int(res[1])
near = sim.getObjectFloatParam(cam, sim.visionfloatparam_near_clipping)
far  = sim.getObjectFloatParam(cam, sim.visionfloatparam_far_clipping)
fov  = sim.getObjectFloatParam(cam, sim.visionfloatparam_perspective_angle)
f = (W / 2) / np.tan(fov / 2)

print(f"Sensor terdeteksi: Resolusi {W}x{H}, Near: {near:.2f}m, Far: {far:.2f}m, FOV: {np.degrees(fov):.1f}°")

line = sim.addDrawingObject(sim.drawing_linestrip, 3, 0, -1, 99999, [1, 0, 0])

keys = set()

def on_press(k):
    try:
        keys.add(k.char.lower())
    except AttributeError:
        pass

def on_release(k):
    try:
        keys.discard(k.char.lower())
    except AttributeError:
        pass

kl = keyboard.Listener(on_press=on_press, on_release=on_release)
kl.start()

def depth_to_world():
    buf, _ = sim.getVisionSensorDepth(cam, 1)
    d = np.frombuffer(buf, dtype=np.float32).reshape(H, W)
    z = d
    v, u = np.mgrid[0:H, 0:W]
    x = (u + 0.5 - W / 2) / f * z
    y = (v + 0.5 - H / 2) / f * z
    valid = (z < far * 0.99) & (z > near)
    pts = np.stack([x, y, z], -1)[valid]
    M = np.array(sim.getObjectMatrix(cam, sim.handle_world)).reshape(3, 4)
    return pts @ M[:, :3].T + M[:, 3]

print("\n=== KONTROL TELEOP DRONE ===")
print(" [W] Maju    [S] Mundur")
print(" [A] Kiri    [D] Kanan")
print(" [R] Naik    [F] Turun")
print(" [X] Selesai & Simpan Point Cloud\n")

sim.startSimulation()
cloud = []
step = 0.05

try:
    while 'x' not in keys:
        p = sim.getObjectPosition(target, sim.handle_world)
        if 'w' in keys: p[0] += step
        if 's' in keys: p[0] -= step
        if 'a' in keys: p[1] += step
        if 'd' in keys: p[1] -= step
        if 'r' in keys: p[2] += step
        if 'f' in keys: p[2] -= step
        sim.setObjectPosition(target, sim.handle_world, p)
        sim.addDrawingObjectItem(line, p)
        
        frame_pts = depth_to_world()
        if len(frame_pts) > 0:
            cloud.append(frame_pts)
        time.sleep(0.1)
finally:
    sim.stopSimulation()
    kl.stop()
    if cloud:
        all_pts = np.vstack(cloud)
        out_path1 = os.path.join(RESULTS_DIR, 'raw_cloud.npy')
        np.save(out_path1, all_pts)
        if out_path1 != 'results/raw_cloud.npy':
            try:
                np.save('results/raw_cloud.npy', all_pts)
            except Exception:
                pass
        print(f"\n[SUKSES] Total titik mentah tersimpan: {len(all_pts):,} titik ke '{out_path1}'")
    else:
        print("\n[PERINGATAN] Tidak ada titik yang berhasil diambil.")
