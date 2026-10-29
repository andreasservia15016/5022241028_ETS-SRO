import numpy as np
import matplotlib.pyplot as plt
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

print("Menghubungkan ke CoppeliaSim Remote API untuk Scan 2D...")
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

line = sim.addDrawingObject(sim.drawing_linestrip, 3, 0, -1, 99999, [0, 1, 0])

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

def depth_to_world_2d():
    buf, _ = sim.getVisionSensorDepth(cam, 1)
    d = np.frombuffer(buf, dtype=np.float32).reshape(H, W)
    mid_row = H // 2
    z_row = d[mid_row, :]
    u = np.arange(W)
    
    valid = (z_row < far * 0.99) & (z_row > near)
    u_val = u[valid]
    z_val = z_row[valid]
    x_val = (u_val + 0.5 - W / 2) / f * z_val
    y_val = np.zeros_like(x_val)
    
    pts = np.stack([x_val, y_val, z_val], -1)
    M = np.array(sim.getObjectMatrix(cam, sim.handle_world)).reshape(3, 4)
    pts_world = pts @ M[:, :3].T + M[:, 3]
    return pts_world[:, :2]  # hanya koordinat X, Y dunia

print("\n=== SCAN 2D (Ketinggian Konstan) ===")
print(" [W/S] Maju/Mundur   [A/D] Kiri/Kanan   [X] Selesai & Simpan Gambar 2D\n")

sim.startSimulation()
pts_2d_list = []
step = 0.05

try:
    while 'x' not in keys:
        p = sim.getObjectPosition(target, sim.handle_world)
        if 'w' in keys: p[0] += step
        if 's' in keys: p[0] -= step
        if 'a' in keys: p[1] += step
        if 'd' in keys: p[1] += step
        sim.setObjectPosition(target, sim.handle_world, p)
        sim.addDrawingObjectItem(line, p)
        
        pts_2d = depth_to_world_2d()
        if len(pts_2d) > 0:
            pts_2d_list.append(pts_2d)
        time.sleep(0.1)
finally:
    sim.stopSimulation()
    kl.stop()
    if pts_2d_list:
        all_pts_2d = np.vstack(pts_2d_list)
        plt.figure(figsize=(8, 8))
        plt.scatter(all_pts_2d[:, 0], all_pts_2d[:, 1], c='black', s=2, alpha=0.6, label='Titik Objek 2D')
        plt.title('2D Occupancy / Contour Map dari Drone', fontsize=14)
        plt.xlabel('X (meter)', fontsize=12)
        plt.ylabel('Y (meter)', fontsize=12)
        plt.grid(True, linestyle='--', alpha=0.5)
        plt.axis('equal')
        plt.legend()
        
        out_img1 = os.path.join(RESULTS_DIR, 'scan2d.png')
        plt.savefig(out_img1, dpi=200, bbox_inches='tight')
        if out_img1 != 'results/scan2d.png':
            try:
                plt.savefig('results/scan2d.png', dpi=200, bbox_inches='tight')
            except Exception:
                pass
        plt.close()
        print(f"\n[SUKSES] Scan 2D berhasil disimpan: {len(all_pts_2d):,} titik ke '{out_img1}'")
    else:
        print("\n[PERINGATAN] Tidak ada titik 2D yang tersimpan.")
