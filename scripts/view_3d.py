import os
import sys
import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
import open3d as o3d

BASE_DIR = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
RESULTS_DIR = os.path.join(BASE_DIR, 'results')

vox_path = os.path.join(RESULTS_DIR, 'voxel.ply')
raw_path = os.path.join(RESULTS_DIR, 'raw_cloud.npy')
traj_path = os.path.join(RESULTS_DIR, 'trajectory.npy')

# 1. Muat Titik 3D
if os.path.exists(vox_path):
    print(f"Memuat point cloud dari: {vox_path}")
    pcd = o3d.io.read_point_cloud(vox_path)
    pts = np.asarray(pcd.points)
elif os.path.exists(raw_path):
    print(f"Memuat raw point cloud dari: {raw_path}")
    pts = np.load(raw_path)
    # Subsampling jika terlalu padat agar GUI lancar
    if len(pts) > 30000:
        idx = np.random.choice(len(pts), size=30000, replace=False)
        pts = pts[idx]
else:
    print(f"[ERROR] File point cloud tidak ditemukan di {RESULTS_DIR}")
    sys.exit(1)

# 2. Muat atau Buat Trajectory Drone (Jalur Biru di Atas)
if os.path.exists(traj_path):
    traj = np.load(traj_path)
else:
    # Buat garis sapuan jika file trajectory belum ada
    wps = []
    ys = np.arange(-1.8, 1.8 + 1e-5, 0.7)
    flip = False
    for y in ys:
        xs = (1.8, -1.8) if flip else (-1.8, 1.8)
        wps.append([xs[0], float(y), 2.0])
        wps.append([xs[1], float(y), 2.0])
        flip = not flip
    traj = np.array(wps)

# 3. Warna Berdasarkan Ketinggian Z (Colormap RdYlGn_r: Hijau -> Kuning -> Merah)
z = pts[:, 2]
z_min, z_max = z.min(), z.max()
z_norm = (z - z_min) / (z_max - z_min + 1e-9)
colors = plt.cm.RdYlGn_r(z_norm)

# 4. Buat Figure 3D Interaktif
fig = plt.figure(figsize=(10, 8), facecolor='white')
ax = fig.add_subplot(111, projection='3d', facecolor='white')

# Plot Titik 3D Objek & Ground
ax.scatter(pts[:, 0], pts[:, 1], pts[:, 2], c=colors, s=3.5, alpha=0.85, label='Titik 3D')

# Plot Lintasan Terbang Drone (Garis Biru di Atas)
ax.plot(traj[:, 0], traj[:, 1], traj[:, 2], color='blue', linewidth=2.5, label='Jalur Drone')

# Batas Sumbu Koordinat (Persis Format Standar)
ax.set_xlim([-3, 3])
ax.set_ylim([-3, 3])
ax.set_zlim([0.0, 2.0])

# Label dan Angka Ticks Sumbu Koordinat
ax.set_xticks([-3, -2, -1, 0, 1, 2, 3])
ax.set_yticks([-3, -2, -1, 0, 1, 2, 3])
ax.set_zticks([0.0, 0.5, 1.0, 1.5, 2.0])

ax.set_xlabel('x', fontsize=11, labelpad=5, fontweight='bold')
ax.set_ylabel('Y', fontsize=11, labelpad=5, fontweight='bold')
ax.set_zlabel('Z', fontsize=11, labelpad=5, fontweight='bold')
ax.set_title('3D Map', fontsize=13, fontweight='bold', pad=15)

# Sudut Pandang Awal
ax.view_init(elev=24, azim=-72)

# Label Kotak Hitam 'mapping 3D' di Pojok Kiri Atas
ax.text2D(0.08, 0.85, 'mapping 3D', transform=ax.transAxes,
          fontsize=13, color='white', fontweight='bold',
          bbox=dict(boxstyle='square,pad=0.4', facecolor='black', edgecolor='none'))

print("="*60)
print("MEMBUKA JENDELA 3D MAP INTERAKTIF")
print("="*60)
print("PETUNJUK KONTROL MOUSE:")
print(" • KLIK KIRI + TAHAN lalu GESER : Memutar sudut 3D secara bebas")
print(" • KLIK KANAN + GESER / SCROLL : Zoom In / Zoom Out")
print(" • KLIK TENGAH (Scroll) + GESER: Menggeser posisi (Pan)")
print(" • Tutup jendela untuk selesai")
print("="*60)

plt.tight_layout()
plt.show()
