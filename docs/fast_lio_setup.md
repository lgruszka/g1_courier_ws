# FAST-LIO2 setup (standalone, do testu mapowania)

Pakiet `g1_courier_fastlio` daje launch + config do uruchomienia
**niezależnego mapowania** FAST-LIO2 obok naszego głównego stacka
(slam_toolbox). Używasz do porównania jakości mapy 3D-LIO vs 2D-SLAM
zanim zdecydujesz czy migrować całość.

Nie zastępuje `mapping_real.launch.py` — odpalasz to **zamiast** niego
podczas testu, nie razem.

## Wymagania

- Realny G1 z Livox Mid-360 + IMU
- Ubuntu 22.04 + ROS2 Humble
- Workspace `g1_courier_ws` zbudowana (pakiet `g1_courier_fastlio`)

## Instalacja

FAST-LIO2 dla ROS2 nie jest w `apt` — klonujemy do `src/` i budujemy
z workspace. Livox driver wymaga osobnego build script.

### 1. apt deps

```bash
sudo apt install -y libpcl-dev libeigen3-dev
```

### 2. Livox ROS2 Driver (Mid-360)

Repo: <https://github.com/Livox-SDK/livox_ros_driver2> (oficjalny Livox
SDK, ROS2 Humble/Jazzy/Foxy).

```bash
cd ~/g1_courier_ws/src
git clone https://github.com/Livox-SDK/livox_ros_driver2

cd livox_ros_driver2
source /opt/ros/humble/setup.bash
./build.sh humble
```

Argument `humble` jest **wymagany** — bez niego skrypt nie wie którą
distrybucję ROS2 budować. Dla Jazzy: `./build.sh jazzy`. Dla Foxy:
`./build.sh ROS2`.

Livox driver **nie używa standardowego colcon** — zawsze build przez
`./build.sh humble` z katalogu pakietu. `colcon build --packages-select
livox_ros_driver2` failuje.

### 3. FAST-LIO2 dla ROS2

Repo: <https://github.com/Ericsii/FAST_LIO_ROS2> (aktywnie utrzymywany
port ROS2, branch `ros2` jako default).

```bash
cd ~/g1_courier_ws/src
git clone --recursive https://github.com/Ericsii/FAST_LIO_ROS2

cd ~/g1_courier_ws
rosdep install --from-paths src --ignore-src -y

colcon build --symlink-install \
  --packages-select fast_lio livox_interfaces g1_courier_fastlio
source install/setup.bash
```

Flaga `--recursive` przy clone jest **wymagana** — FAST_LIO_ROS2 ma
submoduły (m.in. `ikd-Tree`). Bez niej build failuje na missing header.

### 4. Weryfikacja

```bash
ros2 pkg prefix fast_lio
# spodziewane: /home/parallels/g1_courier_ws/install/fast_lio

ros2 pkg prefix livox_ros_driver2
# spodziewane: katalog z install/ livox driver

ros2 pkg executables fast_lio
# spodziewane: fast_lio fastlio_mapping
```

Jeśli któraś komenda zwraca "Package not found" — wróć do build z
`colcon build` plus `source install/setup.bash`.

## Konfiguracja topików

Domyślny `msg_MID360_launch.py` z `livox_ros_driver2` używa
`~/g1_courier_ws/src/livox_ros_driver2/config/MID360_config.json`.
Sprawdź IP twojego Mid-360 i dostosuj plik configu (default
`192.168.1.1xx`).

Po uruchomieniu Livox driver publikuje:
- `/livox/lidar` (sensor_msgs/PointCloud2) — chmura ~10 Hz
- `/livox/imu` (sensor_msgs/Imu) — ~200 Hz, time-synced hardware'owo

Jeśli twój setup używa innych nazw (np. firmware Unitree publikuje pod
`/utlidar/cloud_livox_360mid` + `/utlidar/imu`), edytuj
`config/g1_mid360.yaml`:
```yaml
common:
    lid_topic:  "/utlidar/cloud_livox_360mid"
    imu_topic:  "/utlidar/imu"
```

## Uruchomienie mapowania

```bash
# Terminal 1 — Livox driver (jeśli nie chodzi już Unitree firmware):
ros2 launch livox_ros_driver2 msg_MID360_launch.py

# Weryfikacja że topiki publikują:
ros2 topic hz /livox/lidar    # ~10 Hz
ros2 topic hz /livox/imu      # ~200 Hz
```

```bash
# Terminal 2 — FAST-LIO mapping z naszym configiem:
ros2 launch g1_courier_fastlio fastlio_mapping.launch.py
```

W RViz Fixed Frame ustaw na `camera_init` (FAST-LIO publikuje TF od
tej ramki). Spodziewane: `/cloud_registered` punkty akumulują się,
`/Odometry` publikuje świeże dane ~10 Hz.

```bash
# Terminal 3 — teleop:
ros2 run teleop_twist_keyboard teleop_twist_keyboard
```

**Ważne**: spowolnij do max 0.1 m/s (`vx`) plus 0.1 rad/s (`wz`).
FAST-LIO traci track przy szybkim ruchu — zbieżność iEKF wymaga małej
delta pose między klatkami. Przejedź całą scenę powoli — front każdego
biurka, każdy korytarz. FAST-LIO buduje mapę "na żywo" w RViz.

## Zapis i przetwarzanie mapy

### Save

```bash
# Gdy mapa wygląda kompletnie (NIE Ctrl+C launchu jeszcze):
ros2 service call /map_save std_srvs/srv/Trigger {}
# Spodziewane: success=True
```

Plik zapisany zgodnie z `map_file_path` w `g1_mid360.yaml` (default
`./g1_map.pcd` — w cwd skąd odpaliłeś launch). Sprawdź dokładną
ścieżkę w logu nodu: FAST-LIO loguje "Saved map to ...".

### Podgląd 3D

```bash
# Dependencies (jednorazowo):
pip install "open3d>=0.17,<0.19" "numpy==1.26.4" pillow

# Open3D GUI:
python3 -c "
import open3d as o3d
pcd = o3d.io.read_point_cloud('g1_map.pcd')
print(f'{len(pcd.points)} pkt')
o3d.visualization.draw_geometries([pcd])
"
```

Alternatywa CLI: `pcl_viewer g1_map.pcd` (z `sudo apt install pcl-tools`).

### Konwersja 3D PCD → 2D PGM (dla nav2)

FAST-LIO zapisuje 3D point cloud, ale nav2 wymaga 2D occupancy grid.
Konwerter w repo: `tools/fastlio_pcd_to_pgm.py`.

```bash
mkdir -p ~/maps
python3 ~/g1_courier_ws/tools/fastlio_pcd_to_pgm.py g1_map.pcd \
    --out-dir ~/maps \
    --name lab_fastlio \
    --resolution 0.05 \
    --z-min -0.4 \
    --z-max 1.5
```

Tworzy:
- `~/maps/lab_fastlio.pgm` (obraz mapy 2D)
- `~/maps/lab_fastlio.yaml` (metadane nav2: origin, resolution, thresholds)

Parametry:
- `--z-min -0.4` — wszystko poniżej (podłoga, kable) wycięte
- `--z-max 1.5` — wszystko powyżej (sufit, lampy) wycięte
- `--resolution 0.05` — 5 cm/pixel, standard nav2
- `--min-points-per-cell 2` (default) — komórka "occupied" gdy ≥2 punkty trafiły

## Użycie mapy FAST-LIO w nav2

```bash
ros2 launch g1_courier_bringup real.launch.py map:=$HOME/maps/lab_fastlio.yaml
```

Format jest identyczny ze slam_toolbox-owym (`lab.yaml`/`lab.pgm`).
AMCL ładuje ją tak samo, planner planuje identycznie. **Nie ma żadnej
różnicy z perspektywy nav2** poza tym jak została zbudowana.

Możesz mieć obie mapy obok siebie i wybierać przez `map:=...`:
- `~/maps/lab.yaml` (slam_toolbox)
- `~/maps/lab_fastlio.yaml` (FAST-LIO)

Porównaj po cyklu A↔B która daje stabilniejszą lokalizację AMCL.

## Porównanie z slam_toolbox

| Aspekt | slam_toolbox | FAST-LIO2 |
|---|---|---|
| Mapa 2D PGM | natywnie | konwersja `tools/fastlio_pcd_to_pgm.py` |
| Odom źródło | `/dog_odom` (firmware) | `/Odometry` (LIO z IMU coupling) |
| Loop closure | tak (Ceres pose graph) | nie natywnie |
| 3D info | brak | tak (PCD) |
| CPU | ~10% | ~25% |
| Czas mapowania labu | ~2 min walking | ~15 min wolny walking (max 0.1 m/s) |

Co sprawdzać w porównaniu:
1. Wyrazistość konturów ścian (FAST-LIO zwykle lepszy przez IMU coupling)
2. Dryf po zamknięciu pętli (slam_toolbox lepszy dzięki loop closure)
3. Fragmentaryczność mapy
4. Liczba dziur w nieuporządkowanym labie

## Troubleshooting

### FAST-LIO dropuje wszystkie chmury, mapa pusta

W logu `[fast_lio]: No new scan` albo `Failed to extract scan_msg`.
Sprawdź:
- `lidar_type` w `g1_mid360.yaml`: `4` dla generic PointCloud2,
  `1` dla Livox proprietary CustomMsg (gdy `xfer_format: 1` w livox driver)
- czy `/livox/lidar` faktycznie publikuje:
  ```bash
  ros2 topic info /livox/lidar --verbose
  ros2 topic hz /livox/lidar
  ```

### FAST-LIO traci track po kilku metrach

- Robot porusza się za szybko — zmniejsz teleop do `vx=0.05`, `wz=0.05`
- IMU bias za duży — ustaw `extrinsic_est_en: true` w `g1_mid360.yaml`
  na pierwszej sesji żeby auto-calibrate; wyłącz po skalibrowaniu
- Sprawdź `gyr_cov` i `b_gyr_cov` w configu — Mid-360 wbudowany IMU
  ma typowo `gyr_cov: 0.001`, `b_gyr_cov: 1e-7`

### `tf` warnings: `camera_init` nie istnieje

FAST-LIO publikuje `camera_init → body` TF. Reszta naszego stacka
oczekuje `odom → base_link`. Dla **standalone testu mapping** to nie
problem (Fixed Frame `camera_init` w RViz wystarczy). Dla integracji
z nav2 — potrzebny `static_transform_publisher` mostek
(`odom → camera_init` identity).

### Mapa drift po zamknięciu pętli

FAST-LIO nie ma natywnego loop closure — znana limitation, nie bug.
Trade-off za lepszą local accuracy. Obejścia:
- Mapuj krótkimi sesjami (do 5 min) zamiast jedną długą
- Po mapowaniu edytuj `g1_map.pcd` ręcznie (Open3D, wyrównaj fragmenty,
  eksportuj)
- Alternatywa z loop closure: `LIO-SAM`
  (<https://github.com/TixiaoShan/LIO-SAM>), ale cięższy CPU
