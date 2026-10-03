# Smorphi RoboRoarZ Autonomous Robotics Simulator

Simulator robotika interaktif berbasis Web (WebGL / Three.js, Canvas, Tailwind CSS) untuk platform robot modular **Smorphi** dengan fitur **Code Injection** (eksekusi JavaScript navigasi langsung tanpa restart) dan **Randomized Arena Map Generator** untuk persiapan kompetisi **RoboRoarZ (Autonomous Category)**.

---

## 🚀 Fitur Utama & Modul Sistem

### 1. Model Fisika & Spesifikasi Robot Smorphi
- **Struktur Modular**: 4 blok kubus modular masing-masing berukuran $16\text{ cm} \times 16\text{ cm} \times 16\text{ cm}$ dengan massa 500 g per blok (total massa 2.0 kg).
- **Morfologi 7 Bentuk Geometris**: Menggunakan 3 engsel sudut bermotor (konfigurasi Lower-Left-Right / LLR) yang mampu bertransformasi secara halus (*smooth hermite easing interpolation*) ke dalam:
  * **I** (Straight Monomino 1x4 - lebar profil hanya 16 cm untuk melewati celah sempit)
  * **O** (Square 2x2 - tapak stabil 32x32 cm)
  * **L** (L-Shape)
  * **T** (T-Shape)
  * **Z** (Z-Shape)
  * **S** (S-Shape)
  * **J** (J-Shape / variasinya)
- **Kinematika Penggerak Holonomik 3-DOF (16 Roda Mecanum)**:
  * 4 roda Mecanum pada tiap modul (total 16 roda).
  * Persamaan kinematika inversi untuk menghitung kecepatan tiap roda secara dinamis.
  * Mendukung gerakan maju-mundur ($V_x$), *lateral/crabbing* samping ($V_y$), dan rotasi tanpa radius putar ($\Omega_z$).
- **Emulasi Sensor Presisi**:
  * **2D LiDAR 360 Derajat**: Simulasi raycasting 360 beam ($1^\circ$ angular resolution) yang mendeteksi jarak dinding dan rintangan dalam meter ($0.05\text{ m} - 5.0\text{ m}$).
  * **6-DOF IMU**: Menghasilkan data orientasi (*Heading/Yaw* dalam derajat & radian), *yaw rate* ($\omega$), dan akselerasi linier ($a_x, a_y$).
  * **Odometri Roda**: Koordinat global robot $(X, Y)$, kecepatan aktual, dan akumulasi jarak tempuh.

---

### 2. Sistem Peta Acak (Randomized Arena Map)
- **Arena Kompetisi**: Ruang berukuran $5.0\text{ m} \times 5.0\text{ m}$ dibatasi 4 dinding batas perimeter dengan grid metrik presisi.
- **Random Map Generator**:
  * Preset **7.5%** (*Training Ground* - kepadatan rendah)
  * Preset **15.0%** (*RoboRoarZ Standard* - rintangan kotak, pilar silinder, dan lorong sempit)
  * Preset **28.0%** (*Dense Maze & Chokepoints* - labirin menantang dengan banyak celah sempit)
- **Lorong Sempit Khusus (RoboRoarZ Morphing Challenge)**:
  * Menghasilkan celah lorong berukuran $0.26\text{ m} - 0.28\text{ m}$.
  * Karena bentuk standar **"O"** memiliki lebar $0.32\text{ m}$, robot tidak dapat masuk kecuali bertransformasi menjadi bentuk **"I"** (lebar $0.16\text{ m}$).
- **Validasi Peta Otomatis (BFS Path Reachability)**:
  * Memverifikasi jalur antara titik spawn $(0.8, 0.8)$ dan target tujuan $(4.2, 4.2)$ menggunakan algoritma Breadth-First Search (BFS).
  * Menjamin arena selalu dapat dilalui dan tidak menghasilkan jalan buntu total.
- **Sistem Collision Detection**:
  * Separating Axis Theorem (SAT) untuk 4 bounding box modular Smorphi terhadap dinding dan rintangan.
  * Respons kontak lentur (*inelastic wall sliding*) sehingga robot dapat meluncur di sepanjang permukaan dinding tanpa tembus (*anti-tunneling*).

---

### 3. Interactive Code Injection & Script Execution Engine
Dilengkapi dengan Code Editor interaktif di layar (berbasis Ace Editor dengan tema Monokai dark mode) yang memungkinkan pengguna mengedit, mengganti, dan mengeksekusi logika navigasi otonom secara langsung (*hot code reloading*).

#### Input Data Sensor yang Diterima Script:
- `sensors.lidar`: Objek pemindaian 360 beam dengan fungsi penolong:
  * `sensors.lidar.getFront(deg)`: Mengembalikan jarak terdekat sektor depan ($[-deg, +deg]$).
  * `sensors.lidar.getLeft(deg)`: Mengembalikan jarak sektor kiri ($90^\circ$).
  * `sensors.lidar.getRight(deg)`: Mengembalikan jarak sektor kanan ($270^\circ$).
  * `sensors.lidar.getBack(deg)`: Mengembalikan jarak sektor belakang ($180^\circ$).
  * `sensors.lidar[angle]`: Akses langsung indeks sudut beam ($0^\circ - 359^\circ$).
- `sensors.imu`: `{ heading, headingRad, yaw_rate, yaw_rate_rad, ax, ay }`
- `sensors.pose`: `{ x, y, theta, vx, vy, omega, totalDistance }`
- `sensors.shape`: Bentuk geometri aktif saat ini (`"I"`, `"O"`, `"L"`, `"T"`, `"Z"`, `"S"`, `"J"`).
- `sensors.target`: `{ x, y, distance, angle, angleDeg, reached }`
- `sensors.collision`: Boolean apakah robot sedang mengalami kontak fisik.

#### Fungsi Kontrol Output Robot:
- `robot.setVelocity(vx, vy, omega)`: Mengatur kecepatan maju/mundur, crabbing lateral samping, dan rotasi.
- `robot.setShape("I" | "O" | "L" | "T" | "Z" | "S")`: Mengubah konfigurasi morfologi robot.
- `robot.log(pesan)`: Menampilkan log teks pada jendela simulator console.
- `memory`: Objek memori persisten antar-tick siklus simulasi (dapat digunakan untuk menyimpan state machine, nilai PID, timer).

#### Template Script Bawaan:
1. **Basic Autonomous Obstacle Avoidance (Default)**:
   * Robot bergerak maju secara konstan.
   * Jika LiDAR sektor depan ($[-30^\circ, +30^\circ]$) mendeteksi rintangan $< 0.60\text{ m}$, robot berhenti maju dan menggunakan roda Mecanum untuk *crabbing* menyamping serta berotasi.
   * Jika mendeteksi lorong sempit di kiri-kanan ($< 0.38\text{ m}$), robot memanggil `robot.setShape("I")` untuk mengecilkan lebar penampang robot.
2. **RoboRoarZ Goal Seeker (Artificial Potential Field + Morphing)**: Navigasi menuju target goal dengan medan potensial atraktif dan gaya tolak LiDAR.
3. **Mecanum Holonomic Orbit (No-Turn)**: Demonstrasi manuver menyamping tanpa mengubah arah hadap robot (*heading*).
4. **PID Wall Follower**: Menyusuri dinding kanan pada jarak konstan $0.38\text{ m}$.

---

### 4. Tampilan UI / Dashboard Kontrol
- **Viewport Utama (Three.js 3D)**:
  * Tampilan 3D dengan pencahayaan realistis, bayangan lembut, dan partikel laser LiDAR aktif.
  * Tiga mode kamera: **3D Orbit** (rotasi bebas), **2D Top-Down** (tampilan taktis dari atas), dan **Follow Cam** (kamera di belakang robot).
  * Visualisasi 4 blok kubus warna-warni, engsel berotasi, 16 roda Mecanum berputar, serta garis jejak lintasan (*trajectory breadcrumb*).
- **Panel Telemetri Real-Time**:
  * **LiDAR Polar Radar**: Radar polar presisi tinggi dengan lingkaran batas peringatan bahaya $0.6\text{ m}$ (warna merah/kuning/hijau).
  * **IMU Compass Gauge**: Jarum kompas visual dan pembacaan sudut arah (*heading*).
  * **3-DOF Kinematics Bar**: Indikator kecepatan $V_x, V_y, \Omega_z$.
  * **Morphology Preview**: Tampilan visual susunan 4 blok modular aktif.
- **Kontrol Simulasi**:
  * Slider kecepatan simulasi: 0.5x, 1.0x (realtime), 2.0x, 5.0x (turbo).
  * Mode Teleoperasi Manual: Tekan tombol **Manual (WASD + Q/E)** atau gunakan tombol angka 1-7 untuk transformasi bentuk langsung dari keyboard.
  * Efek Audio Web Audio API (*synthesized sound*) untuk suara motor, servo engsel, dan benturan tanpa dependensi file eksternal.

---

## 💻 Cara Menjalankan Simulator

### Opsi 1: Menjalankan Langsung di Browser
Buka file `index.html` langsung menggunakan browser modern (Google Chrome, Microsoft Edge, Mozilla Firefox, dll.):
```powershell
Start-Process "C:\Users\vicky\.gemini\antigravity\scratch\smorphi-simulator\index.html"
```

### Opsi 2: Menjalankan via Local HTTP Server (PowerShell)
Jalankan skrip PowerShell bawaan:
```powershell
powershell -ExecutionPolicy Bypass -File "C:\Users\vicky\.gemini\antigravity\scratch\smorphi-simulator\run_server.ps1"
```
Server lokal akan aktif di `http://localhost:8080/` dan otomatis membuka browser Anda.
