import numpy as np
import time
import os
import sys
from coppeliasim_zmqremoteapi_client import RemoteAPIClient

sys.stdout.reconfigure(line_buffering=True)

BASE_DIR = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
RESULTS_DIR = os.path.join(BASE_DIR, 'results')
os.makedirs(RESULTS_DIR, exist_ok=True)
os.makedirs('results', exist_ok=True)

print("=== AUTONOMOUS DRONE 3D MAPPING (LAWNMOWER PATTERN) ===")
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

line = sim.addDrawingObject(sim.drawing_linestrip, 3, 0, -1, 99999, [1, 0, 0])

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

def generate_lawnmower_waypoints(x_min=-2.0, x_max=2.0, y_min=-2.0, y_max=2.0, spacing=0.7, z=2.5):
    wps = []
    ys = np.arange(y_min, y_max + 1e-5, spacing)
    flip = False
    for y in ys:
        xs = (x_max, x_min) if flip else (x_min, x_max)
        wps.append((xs[0], float(y), float(z)))
        wps.append((xs[1], float(y), float(z)))
        flip = not flip
    return wps

def goto_waypoint(sim, target, goal, line, cloud, step=0.1, dt=0.03):
    goal_arr = np.array(goal)
    last_scan = 0
    while True:
        p = np.array(sim.getObjectPosition(target, sim.handle_world))
        diff = goal_arr - p
        dist = np.linalg.norm(diff)
        if dist < step:
            sim.setObjectPosition(target, sim.handle_world, goal_arr.tolist())
            sim.addDrawingObjectItem(line, goal_arr.tolist())
            break
        next_pos = (p + (diff / dist) * step).tolist()
        sim.setObjectPosition(target, sim.handle_world, next_pos)
        sim.addDrawingObjectItem(line, next_pos)
        
        now = time.time()
        if now - last_scan >= 0.08:
            pts = depth_to_world()
            if len(pts) > 0:
                cloud.append(pts)
            last_scan = now
            
        time.sleep(dt)

waypoints = generate_lawnmower_waypoints(x_min=-1.8, x_max=1.8, y_min=-1.8, y_max=1.8, spacing=0.7, z=2.5)
print(f"Pola terbang Lawnmower diinisialisasi: {len(waypoints)} waypoint")

sim.startSimulation()
time.sleep(0.5)

cloud = []
t_start = time.time()

try:
    for i, wp in enumerate(waypoints):
        print(f"Menuju Waypoint [{i+1}/{len(waypoints)}]: X={wp[0]:.2f}, Y={wp[1]:.2f}, Z={wp[2]:.2f} (Terkumpul: {sum(len(c) for c in cloud):,} titik)")
        goto_waypoint(sim, target, wp, line, cloud)
finally:
    sim.stopSimulation()
    t_total = time.time() - t_start
    if cloud:
        all_pts = np.vstack(cloud)
        out_path1 = os.path.join(RESULTS_DIR, 'raw_cloud.npy')
        np.save(out_path1, all_pts)
        if out_path1 != 'results/raw_cloud.npy':
            try:
                np.save('results/raw_cloud.npy', all_pts)
            except Exception:
                pass
        print("\n========================================")
        print("MISI AUTOPILOT MAPPING SELESAI!")
        print("========================================")
        print(f"Total Waktu Terbang : {t_total:.1f} detik")
        print(f"Total Titik Mentah  : {len(all_pts):,} titik")
        print(f"Tersimpan di        : {out_path1}")
        print("========================================")
    else:
        print("\n[PERINGATAN] Tidak ada data point cloud yang terkumpul.")
