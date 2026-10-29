import numpy as np
import open3d as o3d
import matplotlib.pyplot as plt
import time
import os
import sys

BASE_DIR = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
RESULTS_DIR = os.path.join(BASE_DIR, 'results')

npy_path = os.path.join(RESULTS_DIR, 'raw_cloud.npy')
if not os.path.exists(npy_path):
    npy_path = 'results/raw_cloud.npy'

if not os.path.exists(npy_path):
    print(f"[ERROR] File '{npy_path}' tidak ditemukan. Jalankan scanning terlebih dahulu!")
    sys.exit(1)

print(f"Memuat raw point cloud dari: {npy_path}")
pts = np.load(npy_path)
pcd = o3d.geometry.PointCloud(o3d.utility.Vector3dVector(pts))
n_raw = len(pcd.points)

voxel_sizes = [0.03, 0.05, 0.08, 0.10, 0.15, 0.20]
point_counts = []
times_ms = []
reduction_ratios = []

print("\n" + "="*70)
print("ANALISIS PERBANDINGAN UKURAN VOXEL (VOXEL RESOLUTION COMPARISON)")
print("="*70)
print(f"{'Voxel Size (m)':<15} | {'Titik Hasil':<15} | {'Reduksi (%)':<15} | {'Waktu (ms)':<12}")
print("-" * 70)

for vs in voxel_sizes:
    # Warmup / running
    t0 = time.perf_counter()
    down = pcd.voxel_down_sample(voxel_size=vs)
    dt = (time.perf_counter() - t0) * 1000  # ms
    
    count = len(down.points)
    red = (1.0 - (count / n_raw)) * 100.0 if n_raw > 0 else 0
    
    point_counts.append(count)
    times_ms.append(dt)
    reduction_ratios.append(red)
    
    print(f"{vs:<15.2f} | {count:<15,} | {red:<15.2f} | {dt:<12.2f}")

print("="*70)

# Simpan Grafik Perbandingan
fig, (ax1, ax2) = plt.subplots(1, 2, figsize=(14, 5))

# Plot 1: Jumlah Titik vs Ukuran Voxel
ax1.plot(voxel_sizes, point_counts, marker='o', color='#1f77b4', linewidth=2.5, markersize=8)
ax1.set_title('Ukuran Voxel vs Jumlah Titik Akhir', fontsize=13, fontweight='bold')
ax1.set_xlabel('Ukuran Voxel (meter)', fontsize=11)
ax1.set_ylabel('Jumlah Titik', fontsize=11)
ax1.grid(True, linestyle='--', alpha=0.6)
for i, txt in enumerate(point_counts):
    ax1.annotate(f"{txt:,}", (voxel_sizes[i], point_counts[i]), textcoords="offset points", xytext=(0,10), ha='center', fontsize=9)

# Plot 2: Waktu Komputasi vs Ukuran Voxel
ax2.plot(voxel_sizes, times_ms, marker='s', color='#d62728', linewidth=2.5, markersize=8)
ax2.set_title('Ukuran Voxel vs Waktu Komputasi (ms)', fontsize=13, fontweight='bold')
ax2.set_xlabel('Ukuran Voxel (meter)', fontsize=11)
ax2.set_ylabel('Waktu Eksekusi (ms)', fontsize=11)
ax2.grid(True, linestyle='--', alpha=0.6)
for i, txt in enumerate(times_ms):
    ax2.annotate(f"{txt:.2f} ms", (voxel_sizes[i], times_ms[i]), textcoords="offset points", xytext=(0,10), ha='center', fontsize=9)

plt.tight_layout()
out_fig1 = os.path.join(RESULTS_DIR, 'voxel_comparison.png')
plt.savefig(out_fig1, dpi=200, bbox_inches='tight')
if out_fig1 != 'results/voxel_comparison.png':
    try:
        plt.savefig('results/voxel_comparison.png', dpi=200, bbox_inches='tight')
    except Exception:
        pass
plt.close(fig)
print(f"\n[SUKSES] Grafik perbandingan voxel disimpan di: {out_fig1}")
