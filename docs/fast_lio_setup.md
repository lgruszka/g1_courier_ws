# FAST-LIO2 setup (standalone, do testu mapowania)

Pakiet `g1_courier_fastlio` daje launch + config do uruchomienia
**niezależnego mapowania** FAST-LIO2 obok naszego głównego stacka
(slam_toolbox). Używasz do porównania jakości mapy 3D-LIO vs 2D-SLAM
zanim zdecydujesz czy migrować całość.

Nie zastępuje `mapping_real.launch.py` — odpalisz to **zamiast** niego
podczas testu, nie razem.

## Wymagania

- Realny G1 z Livox Mid-360 + IMU
- Unitree firmware publikujący chmurę i IMU (sprawdź topiki niżej)
- ROS2 Humble + nasza workspace `g1_courier_ws` zbudowana

## Instalacja zależności (jednorazowo)

FAST-LIO2 nie ma w `apt`. Trzeba sklonować ROS2 port:

```bash
cd ~/g1_courier_ws/src

# FAST-LIO ROS2 wrapper
git clone https://github.com/Ericsii/fast_lio_ros2 fast_lio_ros2
# Plus Livox driver z interfaces (FAST-LIO wymaga livox_interfaces)
git clone https://github.com/Livox-SDK/livox_ros_driver2 livox_ros_driver2

# Build:
cd ~/g1_courier_ws
colcon build --packages-select livox_interfaces livox_ros_driver2 fast_lio g1_courier_fastlio
source install/setup.bash
```

> Alternatywne źródło: kurs AI/RL G1 (lekcja L33) używa
> `~/git-repo/fast_lio_ros2`. Jeśli masz dostęp, możesz użyć tamtejszego
> forka który jest tunowany pod G1. Plus `g1_theconstruct_navigation_stack`
> z TheConstruct ma więcej pakietów pomocniczych (pcd_to_pgm,
> open3d_global_localization). Patrz L33 docs w kursie.

## Sprawdź topiki

```bash
# z chodzącym Unitree firmware:
ros2 topic list | grep -E 'livox|utlidar|imu'
# spodziewane:
#   /livox/lidar       (PointCloud2)
#   /livox/imu         (Imu)
# albo:
#   /utlidar/cloud_livox_360mid
#   /utlidar/imu

ros2 topic hz /livox/lidar    # ~10 Hz
ros2 topic hz /livox/imu      # ~200 Hz (lub wyżej)
```

Jeśli twoje topiki są pod inną nazwą (np. `/utlidar/...`), edytuj
`config/g1_mid360.yaml`:
```yaml
common:
    lid_topic:  "/utlidar/cloud_livox_360mid"
    imu_topic:  "/utlidar/imu"
```

## Uruchomienie mapowania

```bash
# Terminal 1 — FAST-LIO + RViz preset:
ros2 launch g1_courier_fastlio fastlio_mapping.launch.py

# Czekaj ~5-10 s. W RViz Fixed Frame: camera_init.
# Spodziewane: /cloud_registered z punktami akumulującymi się.
# Plus /Odometry publikuje świeże dane (~10 Hz).
```

```bash
# Terminal 2 — teleop manualny:
ros2 run teleop_twist_keyboard teleop_twist_keyboard
# WAŻNE: spowolnij maksymalnie — FAST-LIO traci track przy szybkim ruchu.
# Naciskaj `e` żeby zmniejszyć linear speed do 0.10.
# Naciskaj `x` żeby zmniejszyć angular speed do 0.10.
```

Przejedź całą scenę powoli — front każdego biurka, każdy korytarz.
FAST-LIO buduje mapę "na żywo" w RViz.

## Zapisz mapę

```bash
# Terminal 3 — gdy mapa wygląda kompletnie (NIE Ctrl+C launchu jeszcze):
ros2 service call /map_save std_srvs/srv/Trigger {}
# Spodziewane: response success=True
```

Plik `g1_map.pcd` zapisany w katalogu z którego uruchomiłeś launch
(`~/g1_courier_ws/` zwykle).

## Obejrzenie mapy 3D

```bash
pip install "numpy==1.26.4" "open3d>=0.17,<0.19"
python3 -c "
import open3d as o3d
pcd = o3d.io.read_point_cloud('g1_map.pcd')
print(f'{len(pcd.points)} pkt')
o3d.visualization.draw_geometries([pcd])
"
```

Plus kurs ma dedicated viewer: `python3 pcd_viewer.py g1_map.pcd`.

## Konwersja 3D PCD → 2D PGM (do nav2)

FAST-LIO zapisuje 3D point cloud, ale nav2 wymaga 2D PGM. Konwerter
w naszym repo: `tools/fastlio_pcd_to_pgm.py`.

```bash
# Zależności (jednorazowo):
pip install "open3d>=0.17,<0.19" "numpy==1.26.4" pillow

# Konwersja:
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
- `~/maps/lab_fastlio.yaml` (metadane: origin, resolution, thresholds)

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

To wymaga adaptacji `odom_tf_relay` (dziś czyta `/dog_odom` od Tomasza,
można skierować na `/Odometry`). ~1-godzinna zmiana. Daj znać jeśli
chcesz tę ścieżkę.

## Porównanie wyników z slam_toolbox

| Aspekt | slam_toolbox (nasze) | FAST-LIO2 (test) |
|---|---|---|
| Mapa 2D PGM | ~/maps/lab_slam.pgm | konwersja z lab_fastlio.pgm |
| Odom źródło | /dog_odom (firmware) | /Odometry (FAST-LIO z IMU coupling) |
| Loop closure | tak (Ceres) | nie natywnie |
| 3D info | brak | tak (PCD) |
| CPU | ~10% | ~25% |
| Czas mapowania | ~2 min walking | ~15 min wolny walking |

Sprawdź:
1. Wyrazistość konturów ścian (FAST-LIO zwykle lepsze)
2. Dryf po zamknięciu pętli (slam_toolbox lepszy dzięki loop closure)
3. Fragmentaryczność (FAST-LIO + IMU stable)
4. Liczba dziur w nieuporządkowanym labie

## Co dalej jeśli FAST-LIO wygrywa

Jeśli FAST-LIO daje znacznie lepsze mapy:

**Opcja minimalna**: trzymaj FAST-LIO **tylko jako odometry replacement**.
Zastąp `odom_tf_relay` (Tomasz, z `/dog_odom`) → `odom_tf_relay`
z `/Odometry` (FAST-LIO). Slam_toolbox nadal mapuje 2D, ale dostaje
lepszy odom feedback. ~1 dzień pracy.

**Opcja pełna**: migracja do open3d_global_localization (replacement
AMCL) + pełen stack FAST-LIO mapping. Patrz kurs L33-L37. ~2-3 dni.

## Troubleshooting

### FAST-LIO dropuje wszystkie chmury, mapa pusta

Logi pokazują `Failed to extract scan_msg`. Sprawdź:
- `lidar_type` w configu — 4 dla generic PointCloud2, 1 dla Livox proprietary
- czy `/livox/lidar` faktycznie publikuje PointCloud2 (`ros2 topic info`)

### `tf` warnings camera_init nie istnieje

FAST-LIO publikuje `camera_init → body` TF. Plus reszta naszego stacka
oczekuje `odom → base_link`. Dla **standalone testu** nie jest to problem
(FAST-LIO + RViz wystarcza). Plus dla integracji z nav2 — potrzebne
mostek static_transform_publisher (patrz `global_localization_g1.launch.py`
w kursie).

### Mapa drift po zamknięciu pętli

FAST-LIO nie ma natywnego loop closure. Bug nie błąd. Albo:
- Akceptuj jako trade-off za lepszą local accuracy
- Użyj `open3d_global_localization` z kursu jako post-processing
- Migruj do `LIO-SAM` (alternatywa z loop closure, ale cięższa)
