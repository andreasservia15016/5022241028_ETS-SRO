import numpy as np
import open3d as o3d
import matplotlib.pyplot as plt
import os
import sys

BASE_DIR = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
RESULTS_DIR = os.path.join(BASE_DIR, 'results')

ply_path = os.path.join(RESULTS_DIR, 'voxel.ply')
if not os.path.exists(ply_path):
    ply_path = 'results/voxel.ply'

if not os.path.exists(ply_path):
    print(f"[ERROR] File '{ply_path}' tidak ditemukan. Jalankan postprocess_voxel.py terlebih dahulu!")
    sys.exit(1)

print(f"Memuat point cloud voxel dari: {ply_path}")
vox = o3d.io.read_point_cloud(ply_path)
p = np.asarray(vox.points)

z_min = p[:, 2].min()
z_max = p[:, 2].max()
z_range = np.ptp(p[:, 2])

zn = (p[:, 2] - z_min) / (z_range + 1e-9)

# Gunakan colormap 'jet' (Biru = Ground / Rendah, Merah = Puncak / Tinggi)
cmap = plt.get_cmap('jet')
colors = cmap(zn)[:, :3]
vox.colors = o3d.utility.Vector3dVector(colors)

out_colored1 = os.path.join(RESULTS_DIR, 'colored.ply')
o3d.io.write_point_cloud(out_colored1, vox)
if out_colored1 != 'results/colored.ply':
    try:
        o3d.io.write_point_cloud('results/colored.ply', vox)
    except Exception:
        pass
print(f"[SUKSES] Point cloud berwarna tersimpan di: {out_colored1}")

# Generate colorbar untuk dokumentasi laporan
fig, ax = plt.subplots(figsize=(1.5, 6))
norm = plt.Normalize(vmin=z_min, vmax=z_max)
sm = plt.cm.ScalarMappable(cmap=cmap, norm=norm)
cbar = fig.colorbar(sm, cax=ax)
cbar.set_label('Ketinggian Objek (meter)', fontsize=12, fontweight='bold')
ax.tick_params(labelsize=10)

out_cbar1 = os.path.join(RESULTS_DIR, 'colorbar.png')
fig.savefig(out_cbar1, dpi=200, bbox_inches='tight')
if out_cbar1 != 'results/colorbar.png':
    try:
        fig.savefig('results/colorbar.png', dpi=200, bbox_inches='tight')
    except Exception:
        pass
plt.close(fig)
print(f"[SUKSES] Colorbar tersimpan di: {out_cbar1}")

print("========================================")
print("HASIL COLOR MAPPING KETINGGIAN")
print("========================================")
print(f"Titik Terendah (Ground) : {z_min:.2f} m (Biru)")
print(f"Titik Tertinggi (Puncak): {z_max:.2f} m (Merah)")
print(f"Rentang Ketinggian      : {z_range:.2f} m")
print(f"Total Titik Berwarna    : {len(p):,} titik")
print("========================================")

if '--no-gui' not in sys.argv:
    try:
        print("Membuka jendela visualisasi Open3D (Tutup jendela untuk selesai)...")
        o3d.visualization.draw_geometries([vox], window_name="3D Elevation Colored Point Cloud")
    except Exception as e:
        print(f"Catatan GUI: {e}")
