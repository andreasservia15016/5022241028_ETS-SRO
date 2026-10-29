import numpy as np
import open3d as o3d
import time
import os
import sys

BASE_DIR = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
RESULTS_DIR = os.path.join(BASE_DIR, 'results')

npy_path = os.path.join(RESULTS_DIR, 'raw_cloud.npy')
if not os.path.exists(npy_path):
    npy_path = 'results/raw_cloud.npy'

if not os.path.exists(npy_path):
    print(f"[ERROR] File '{npy_path}' tidak ditemukan. Jalankan teleop_scan.py atau autopilot_lawnmower.py terlebih dahulu!")
    sys.exit(1)

print(f"Memuat raw point cloud dari: {npy_path}")
pts = np.load(npy_path)
pcd = o3d.geometry.PointCloud(o3d.utility.Vector3dVector(pts))

voxel_size = 0.1
t0 = time.time()
vox = pcd.voxel_down_sample(voxel_size=voxel_size)
dt = time.time() - t0

n_raw = len(pcd.points)
n_vox = len(vox.points)
ratio = (n_vox / n_raw) * 100 if n_raw > 0 else 0
reduction = 100 - ratio

print("========================================")
print("HASIL REDUKSI VOXEL (VOXEL DOWNSAMPLING)")
print("========================================")
print(f"Ukuran Voxel      : {voxel_size} m (10 cm)")
print(f"Titik Mentah (Raw): {n_raw:,} titik")
print(f"Titik Voxel       : {n_vox:,} titik")
print(f"Rasio Tersisa     : {ratio:.2f}%")
print(f"Reduksi Data      : {reduction:.2f}%")
print(f"Waktu Komputasi   : {dt*1000:.2f} ms")
print("========================================")

out_ply1 = os.path.join(RESULTS_DIR, 'voxel.ply')
o3d.io.write_point_cloud(out_ply1, vox)
if out_ply1 != 'results/voxel.ply':
    try:
        o3d.io.write_point_cloud('results/voxel.ply', vox)
    except Exception:
        pass
print(f"Model voxel tersimpan di: {out_ply1}")

# Tampilkan visualisasi jika tidak dalam mode headless
if '--no-gui' not in sys.argv:
    try:
        grid = o3d.geometry.VoxelGrid.create_from_point_cloud(vox, voxel_size=voxel_size)
        print("Membuka jendela visualisasi Open3D (Tutup jendela untuk lanjut)...")
        o3d.visualization.draw_geometries([vox], window_name="3D Voxel Point Cloud")
    except Exception as e:
        print(f"Catatan GUI: {e}")
