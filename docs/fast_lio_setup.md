# FAST-LIO2 setup (standalone, do testu mapowania)

Pakiet `g1_courier_fastlio` daje launch + config do uruchomienia
**niezależnego mapowania** FAST-LIO2 obok naszego głównego stacka
(slam_toolbox). Używasz do porównania jakości mapy 3D-LIO vs 2D-SLAM
zanim zdecydujesz czy migrować całość.

Nie zastępuje `mapping_real.launch.py` — odpalisz to **zamiast** niego
podczas testu, nie razem.

## Wymagania

- Realny G1 z Livox Mid-360 + IMU
- Ubuntu 22.04 + ROS2 Humble (potwierdzone wsparcie dla Livox driver2 + FAST-LIO_ROS2)
- Nasza workspace `g1_courier_ws` zbudowana (`g1_courier_fastlio` pakiet)

## Instalacja zależności (jednorazowo)

FAST-LIO2 dla ROS2 nie jest dostępne w `apt`. Plus trzeba uruchomić
Livox driver. Klonujemy do `src/` i budujemy z workspace.

### 1. apt deps

```bash
sudo apt install -y libpcl-dev libeigen3-dev
```

### 2. Livox ROS2 Driver (Mid-360)

**Repo**: <https://github.com/Livox-SDK/livox_ros_driver2>
(oficjalne Livox SDK, ROS2 Humble + Jazzy + Foxy support)

```bash
cd ~/g1_courier_ws/src
git clone https://github.com/Livox-SDK/livox_ros_driver2

# Livox driver wymaga osobnego build script (nie standardowy colcon):
cd livox_ros_driver2
source /opt/ros/humble/setup.bash
./build.sh humble
# Tworzy install/ wewnątrz livox_ros_driver2/. Plus build.sh symlinkuje wynik
# do parent workspace install/.
```

> Argument `humble` jest **wymagany** — bez niego skrypt nie wie który ROS2.
> Dla Jazzy: `./build.sh jazzy`. Dla Foxy: `./build.sh ROS2`.

### 3. FAST-LIO2 dla ROS2

**Repo**: <https://github.com/Ericsii/FAST_LIO_ROS2>
(aktywnie utrzymywany port ROS2, 570⭐, branch `ros2` jako default)

```bash
cd ~/g1_courier_ws/src
git clone --recursive https://github.com/Ericsii/FAST_LIO_ROS2

# Plus rozwiąż zależności:
cd ~/g1_courier_ws
rosdep install --from-paths src --ignore-src -y

# Build:
colcon build --symlink-install --packages-select fast_lio livox_interfaces g1_courier_fastlio
source install/setup.bash
```

> Flaga `--recursive` jest **wymagana** przy clone — FAST_LIO_ROS2 ma submoduły
> (m.in. `ikd-Tree`). Bez `--recursive` build failuje na missing header.

### 4. Weryfikacja

```bash
ros2 pkg prefix fast_lio
# spodziewane: /home/parallels/g1_courier_ws/install/fast_lio

ros2 pkg prefix livox_ros_driver2
# spodziewane: ...

ros2 pkg executables fast_lio
# spodziewane: fast_lio fastlio_mapping
```

Wszystkie trzy komendy muszą zwrócić sensowny output. Jeśli któraś
"Package not found" — wróć do build z `colcon build`.

## Konfiguracja Livox driver dla Mid-360

Livox publikuje na configurable topikach. Domyślny `msg_MID360_launch.py`
z `livox_ros_driver2` używa configów z `~/g1_courier_ws/src/livox_ros_driver2/config/MID360_config.json`. Sprawdź który IP ma twój Mid-360
i dostosuj plik configu jeśli inny niż domyślny `192.168.1.1xx`.

Po uruchomieniu Livox driver publikuje:
- `/livox/lidar` (sensor_msgs/PointCloud2) — chmura ~10 Hz
- `/livox/imu` (sensor_msgs/Imu) — ~200 Hz, time-synced z lidar hardware'owo

Jeśli twój setup używa innych nazw (np. firmware Unitree publikuje pod
`/utlidar/cloud_livox_360mid` + `/utlidar/imu`), edytuj `config/g1_mid360.yaml`:
```yaml
common:
    lid_topic:  "/utlidar/cloud_livox_360mid"
    imu_topic:  "/utlidar/imu"
```

## Uruchomienie mapowania

```bash
# Terminal 1 — odpal Livox driver (jeśli nie chodzi już Unitree firmware):
ros2 launch livox_ros_driver2 msg_MID360_launch.py

# Sprawdź że topiki publikują:
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
# Terminal 3 — teleop, jeźdź bardzo wolno:
ros2 run teleop_twist_keyboard teleop_twist_keyboard
# WAŻNE: spowolnij do max 0.1 m/s. FAST-LIO traci track przy szybkim
# ruchu (zbieżność iEKF wymaga małej delta pose między klatkami).
# Naciskaj `e` aż linear speed = 0.10
# Naciskaj `x` aż angular speed = 0.10
```

Przejedź całą scenę powoli — front każdego biurka, każdy korytarz.
FAST-LIO buduje mapę "na żywo" w RViz.

## Zapis mapy

```bash
# Terminal 4 — gdy mapa wygląda kompletnie (NIE Ctrl+C launchu jeszcze):
ros2 service call /map_save std_srvs/srv/Trigger {}
# Spodziewane: response success=True
```

Plik `scans.pcd` zapisany w `~/.ros/` lub w cwd skąd uruchomiłeś launch.
Sprawdź dokładną ścieżkę w logu nodu — FAST-LIO loguje "Saved map to ...".

## Obejrzenie mapy 3D

```bash
pip install "open3d>=0.17,<0.19" "numpy==1.26.4"
python3 -c "
import open3d as o3d
pcd = o3d.io.read_point_cloud('scans.pcd')
print(f'{len(pcd.points)} pkt')
o3d.visualization.draw_geometries([pcd])
"
```

Plus alternatywa CLI: `pcl_viewer scans.pcd` (z `apt install pcl-tools`).

## Konwersja 3D PCD → 2D PGM (do nav2)

FAST-LIO zapisuje 3D point cloud, ale nav2 wymaga 2D PGM. Konwerter
w naszym repo: `tools/fastlio_pcd_to_pgm.py`.

```bash
# Zależności (jednorazowo):
pip install "open3d>=0.17,<0.19" "numpy==1.26.4" pillow

# Konwersja:
mkdir -p ~/maps
python3 ~/g1_courier_ws/tools/fastlio_pcd_to_pgm.py scans.pcd \
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

## Użyj mapy FAST-LIO w nav2 (identycznie jak slam_toolbox)

```bash
ros2 launch g1_courier_bringup real.launch.py map:=$HOME/maps/lab_fastlio.yaml
```

Format jest identyczny ze slam_toolbox-owym (`lab.yaml`/`lab.pgm`).
AMCL ładuje ją tak samo, planner planuje identycznie. **Nie ma żadnej
różnicy z perspektywy nav2** poza tym jak została zbudowana.

Plus możesz mieć **obie mapy obok siebie** i wybierać przez `map:=...`:
- `~/maps/lab.yaml` (slam_toolbox)
- `~/maps/lab_fastlio.yaml` (FAST-LIO)

Porównaj po cyklu A↔B który daje stabilniejszą lokalizację AMCL.

## Alternatywne podejście — FAST-LIO odometry obok slam_toolbox

Jeśli FAST-LIO mapa działa lepiej **ale** chcesz zachować slam_toolbox
loop closure: użyj FAST-LIO **tylko jako odometry source**.
- `/Odometry` (FAST-LIO) → relay → `odom → base_link` TF
- slam_toolbox nadal mapuje + ma loop closure
- AMCL nadal lokalizuje w 2D

To wymaga adaptacji `odom_tf_relay` (dziś czyta `/dog_odom` od firmware,
można skierować na `/Odometry`). ~1-godzinna zmiana. Daj znać jeśli
chcesz tę ścieżkę.

## Porównanie wyników z slam_toolbox

| Aspekt | slam_toolbox (nasze) | FAST-LIO2 (test) |
|---|---|---|
| Mapa 2D PGM | `~/maps/lab.yaml` | konwersja `tools/fastlio_pcd_to_pgm.py` → `~/maps/lab_fastlio.yaml` |
| Odom źródło | `/dog_odom` (firmware) | `/Odometry` (FAST-LIO z IMU coupling) |
| Loop closure | tak (Ceres pose graph) | nie natywnie |
| 3D info | brak | tak (PCD) |
| CPU | ~10% | ~25% |
| Czas mapowania typowego labu | ~2 min walking | ~15 min wolny walking (max 0.1 m/s) |

Sprawdź:
1. Wyrazistość konturów ścian (FAST-LIO zwykle lepsze przez IMU coupling)
2. Dryf po zamknięciu pętli (slam_toolbox lepszy dzięki loop closure)
3. Fragmentaryczność mapy (FAST-LIO + IMU stable)
4. Liczba dziur w nieuporządkowanym labie

## Troubleshooting

### FAST-LIO dropuje wszystkie chmury, mapa pusta

W logu `[fast_lio]: No new scan` albo `Failed to extract scan_msg`.
Sprawdź:
- `lidar_type` w configu `g1_mid360.yaml` — `4` dla generic PointCloud2,
  `1` dla Livox proprietary CustomMsg (gdy używasz `xfer_format: 1` w livox driver)
- czy `/livox/lidar` faktycznie publikuje:
  ```bash
  ros2 topic info /livox/lidar --verbose
  ros2 topic hz /livox/lidar
  ```

### FAST-LIO traci track po kilku metrach

- Robot porusza się za szybko — zmniejsz teleop do `vx=0.05`, `wz=0.05`
- IMU bias za duży — w `g1_mid360.yaml` ustaw `extrinsic_est_en: true`
  na pierwszej sesji żeby auto-calibrate; wyłącz po skalibrowaniu
- Sprawdź czy `gyr_cov` i `b_gyr_cov` w configu nie są za niskie dla
  twojego IMU (Mid-360 wbudowany ma typowo `gyr_cov: 0.001`,
  `b_gyr_cov: 1e-7`)

### `tf` warnings: `camera_init` nie istnieje

FAST-LIO publikuje `camera_init → body` TF. Plus reszta naszego stacka
oczekuje `odom → base_link`. Dla **standalone testu mapping** nie jest
to problem (FAST-LIO + RViz wystarcza, Fixed Frame `camera_init`). Plus
dla integracji z nav2 — potrzebne static_transform_publisher mostek
(`odom → camera_init` identity).

### `livox_ros_driver2` nie buduje się przez colcon

Driver Livox **nie używa standardowego colcon** — wymaga `./build.sh humble`.
Jeśli próbujesz `colcon build --packages-select livox_ros_driver2` to
może failować. Zawsze build przez `./build.sh humble` z katalogu pakietu.

### Mapa drift po zamknięciu pętli (np. wracając do startu)

FAST-LIO nie ma natywnego loop closure. To znana limitation, nie bug.
Akceptuj jako trade-off za lepszą local accuracy. Plus alternatywy:
- Mapuj krótkimi sesjami (do 5 min) zamiast jedną długą
- Po mapowaniu, edytuj `g1_map.pcd` ręcznie (otwórz w Open3D, wyrównaj
  fragmenty, eksportuj)
- Migracja do `LIO-SAM` (https://github.com/TixiaoShan/LIO-SAM) —
  alternatywa z loop closure, ale cięższa CPU
