# 3D Mapping Menggunakan Drone (Quadcopter) di CoppeliaSim

Repositori ini berisi implementasi lengkap rekonstruksi pemetaan 3 Dimensi (**3D Mapping**) menggunakan wahana tanpa awak (**Quadcopter Drone**) dengan sensor penglihatan kedalaman (**Vision Depth Sensor**) pada simulator **CoppeliaSim Edu**.

Proyek ini dirancang untuk memenuhi kriteria penilaian **Grade A (5)** pada mata kuliah Sistem Robotika Otonom (SRO), ditambah dua fitur bonus orisinal: **Autopilot Lawnmower Sweep** dan **Multi-Resolution Voxel Benchmark**.

---

## 1. Struktur Repositori

```
drone-3d-mapping/
├── README.md                      # Dokumentasi & panduan eksekusi
├── scene/
│   └── drone_mapping.ttt          # Scene CoppeliaSim dengan orientasi sensor miring 35°
├── scripts/
│   ├── scanner.lua                # Script child sensor CoppeliaSim (sysCall_sensing)
│   ├── teleop_scan.py             # Grade C & B: Kontrol drone + visualisasi jalur + 3D scan
│   ├── scan2d.py                  # Grade BC: Scan 2D kontur horizontal objek
│   ├── autopilot_lawnmower.py     # Bonus 1: Terbang otonom pola sapuan lawnmower
│   ├── postprocess_voxel.py       # Grade AB: Reduksi voxel downsampling (0.1m)
│   ├── postprocess_color.py       # Grade A: Color mapping elevasi ketinggian (Jet colormap)
│   └── compare_voxel_sizes.py     # Bonus 2: Komparasi multi-resolusi ukuran voxel
├── results/
│   ├── raw_cloud.npy              # Data mentah 3D point cloud (982.915 titik)
│   ├── scan2d.png                 # Plot visualisasi 2D occupancy map
│   ├── voxel.ply                  # Point cloud voxel (16.458 titik)
│   ├── colored.ply                # Point cloud berwarna berdasarkan ketinggian
│   ├── colorbar.png               # Skala referensi warna metrik elevasi
│   ├── voxel_comparison.png       # Grafik perbandingan resolusi voxel
│   └── screenshots/               # Tangkapan layar 3D perspektif & tampak atas
├── report/
│   ├── report.md                  # Naskah laporan komprehensif (Markdown)
│   └── report.pdf                 # Laporan akademik resmi siap kumpul (PDF)
└── requirements.txt               # Daftar pustaka Python
```

---

## 2. Kebutuhan Lingkungan & Instalasi

* **Python:** Python 3.10 ke atas (diuji pada Python 3.13)
* **Simulator:** CoppeliaSim Edu 4.5+ (diuji pada CoppeliaSim 4.10)

Pasang seluruh dependensi pustaka Python:
```bash
pip install -r requirements.txt
```

Isi `requirements.txt`:
```
coppeliasim-zmqremoteapi-client
numpy
open3d
matplotlib
pynput
reportlab
```

---

## 3. Konfigurasi Scene & Sensor di CoppeliaSim

1. Buka scene `scene/drone_mapping.ttt` di CoppeliaSim Edu.
2. Pastikan sensor `sensor` pada hierarki `/Quadcopter/fast3DLaserScanner/body/sensor`:
   * **Explicit handling:** `OFF`
   * **Ignore depth info:** `OFF` (tidak dicentang)
   * **Render mode:** `Legacy OpenGL`
   * **Resolusi:** `64 x 64`
   * **Kemiringan Sensor (Tilt):** Dimiringkan 35° ke depan-bawah (Euler: `[-180°, 35°, -90°]`). Vektor optik `[0.574, 0.0, -0.819]` memungkinkan deteksi ground, badan, dan puncak objek secara simultan dalam 1 frame.
3. Script `fast3DLaserScanner` telah diperbarui dengan `scripts/scanner.lua` untuk menghilangkan error `sysCall_sensing` versi lama.

---

## 4. Urutan Menjalankan Skrip

Pastikan CoppeliaSim sedang terbuka dengan scene `drone_mapping.ttt`.

### Opsi A: Misi Pemetaan Otonom (Rekomendasi / Fitur Bonus)
Jalankan misi pemetaan otomatis tanpa perlu mengendalikan joystick manual:
```bash
python scripts/autopilot_lawnmower.py
```
*Drone akan otomatis terbang menyapu 12 waypoint dengan pola boustrophedon, menggambar garis lintasan merah, dan mengakumulasi data titik mentah ke `results/raw_cloud.npy`.*

### Opsi B: Teleoperasi Manual (Grade C & B)
Jika ingin mengendalikan drone secara manual:
```bash
python scripts/teleop_scan.py
```
**Kontrol Keyboard:**
* `W` : Bergerak Maju (+X)
* `S` : Bergerak Mundur (-X)
* `A` : Bergerak ke Kiri (+Y)
* `D` : Bergerak ke Kanan (-Y)
* `R` : Naik Vertikal (+Z)
* `F` : Turun Vertikal (-Z)
* `X` : Selesai pemindaian & simpan `results/raw_cloud.npy`

### Tahap 2: Pemindaian 2D (Grade BC)
```bash
python scripts/scan2d.py
```
*Mengekstrak baris tengah horizontal depth buffer untuk menghasilkan denah kontur okupansi 2D `results/scan2d.png`.*

### Tahap 3: Reduksi Voxel Grid (Grade AB)
```bash
python scripts/postprocess_voxel.py
```
*Mereduksi kerapatan titik dengan voxel 0.1m, menghasilkan file `results/voxel.ply` dan menampilkan statistik reduksi data.*

### Tahap 4: Color Mapping Ketinggian (Grade A)
```bash
python scripts/postprocess_color.py
```
*Memberikan gradien warna elevasi ketinggian Jet (Biru = Lantai, Merah = Puncak) dan menyimpan `results/colored.ply` serta `results/colorbar.png`.*

### Tahap 5: Analisis Komparatif Multi-Ukuran Voxel (Bonus 2)
```bash
python scripts/compare_voxel_sizes.py
```
*Menguji ukuran voxel 0.03m s.d 0.20m dan menghasilkan grafik kurva `results/voxel_comparison.png`.*

---

## 5. Ringkasan Hasil Eksperimen

| Metrik Evaluasi | Hasil Eksperimen Riil |
|---|---|
| **Total Titik Mentah (Raw Cloud)** | **982.915 titik** |
| **Total Titik Voxel (0.1m)** | **16.458 titik** |
| **Persentase Reduksi Data** | **98.33 %** |
| **Waktu Komputasi Voxel** | **45.22 ms** |
| **Elevasi Terendah (Ground)** | **-0.27 m** (Biru Pekat) |
| **Elevasi Tertinggi (Puncak Gedung)** | **1.22 m** (Merah Pekat) |
| **Durasi Terbang Autopilot** | **64.3 detik** (12 Waypoint) |

---

## 6. Tautan Pengumpulan

* **Video (https://drive.google.com/drive/folders/1vakVmaOmMzkNM4MVdnj-l-xGzTtBnzIg?usp=sharing)
  

