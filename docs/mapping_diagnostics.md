# Lista diagnostyczna mapowania (courier-deploy)

Gdy mapowanie z `mapping_real.launch.py` daje słabe wyniki (mapa
fragmentaryczna, dryf, dziury w ścianach), przejdź tę listę.

## Gdzie konwertujemy 3D na 2D

```
Mid-360 hardware
   ↓ wewnętrzny SDK Livoxa (~250k pkt/s, 40 m, 360° × 59° FoV)
   ↓
Unitree firmware Mid-360 driver
   ↓ sensor_msgs/PointCloud2 na /utlidar/cloud_livox_360mid
   ↓ header.frame_id = "utlidar_lidar" (lub "livox_frame")
   ↓ ~10 Hz, ~25 000 pkt/skan po decimation
   ↓
pointcloud_to_laserscan_node   ← TU REDUKCJA 3D → 2D
   - bierze tylko punkty z pasma height ∈ [min_height, max_height]
     - default deploy: 0.5–0.75 m (real-tested by Tomasz)
     - alternatywa: 0.15–1.40 m (szersze, więcej detail ale więcej szumu)
   - rzutuje na 2D ring wokół base_link (XY plane)
   - agreguje po kącie azymutalnym
   ↓ sensor_msgs/LaserScan na /scan
   ↓ ~720 pkt/scan (0.5° increment, full 360°)
   ↓ ~10 Hz
   ↓
slam_toolbox
   - max_laser_range: ile wzwyż uznaje pomiar (default 8 m — za małe dla Mid-360!)
   - minimum_travel_distance: jak często aktualizuje mapę
   - output /map (nav_msgs/OccupancyGrid)
```

**Kluczowe slice'owanie**: bierzemy ~25 cm wycinek wysokości (0.50–0.75 m)
z chmury 250k pkt/s. Wszystko poza tym jest wyrzucane.

## Lista diagnostyczna krok po kroku

### Test 1 — chmura przychodzi i ma sensowny shape

```bash
ros2 topic hz /utlidar/cloud_livox_360mid
# spodziewane: ~10 Hz stabilnie

ros2 topic echo /utlidar/cloud_livox_360mid --field 'header.stamp,height,width' --once
# width: 10000–30000 pkt
```

- `0 Hz` → Livox driver nie chodzi
- `width < 5000` → driver dekymuje zbyt mocno albo widoczność słaba

### Test 2 — `/scan` ma dane

```bash
ros2 topic hz /scan
# ~10 Hz

ros2 topic echo /scan --field 'ranges' --once 2>/dev/null | head -30
# większość 0.5–8.0, parę inf
```

- wszystkie `inf` → slice height wycina za mocno. Rozszerz `min_height/max_height`.
- wszystkie `0.20` (= `range_min`) → lidar widzi siebie. Zwiększ `range_min`.

### Test 3 — Mid-360 zasięg vs slam_toolbox

```bash
grep max_laser_range src/g1_courier_bringup/config/slam_toolbox_mapping.yaml
# default: 20.0 (zmieniony z 8.0 — Mid-360 fizycznie sięga 40+ m)
```

Jeśli widzisz 8.0 — to za mało dla typowego labu. Podnieś do 20–30.

### Test 4 — update map frequency

```bash
grep -E 'minimum_travel|minimum_time' src/g1_courier_bringup/config/slam_toolbox_mapping.yaml
# minimum_travel_distance: 0.05  (5 cm — z mvelocity_floor 0.12 m/s update ~0.4 s)
# minimum_travel_heading: 0.05   (~3°)
# minimum_time_interval: 0.2     (max 5 Hz update)
```

Gdy widzisz `0.2 m` — slam czeka 1.7 s między aktualizacjami przy carry-mode
prędkości 0.12 m/s. Obniż do 0.05.

### Test 5 — TF latency

```bash
ros2 run tf2_tools view_frames
evince frames.pdf
# sprawdź: odom → base_link → utlidar_lidar (lub livox_frame)

ros2 run tf2_ros tf2_echo odom base_link
# ~50 Hz aktualizacje (z odom_tf_relay)

ros2 run tf2_ros tf2_echo base_link utlidar_lidar
# stałe wartości (static_transform_publisher)
```

- TF brak → `pointcloud_to_laserscan` dropuje wszystko, `/scan` 0 Hz
- TF laguje → slam_toolbox dropuje skany, "Message Filter dropping" w logu

### Test 6 — slam_toolbox logi

```bash
ros2 launch g1_courier_bringup mapping_real.launch.py 2>&1 | grep -E 'dropping|drift|loop|skip|warn|error'
```

Kluczowe:
- `Message Filter dropping` → TF mismatch (scan timestamp poza tolerance)
- `loop closure` → loop closure events (powinny być widoczne przy powrotach)

### Test 7 — wizualnie sprawdź /scan

```bash
python3 ~/g1_courier_ws/tools/lidar_viewer.py
```

Szukaj:
- wyraźne kontury ścian (długie linie)
- fronty biurek (krótkie ~80 cm linie)
- brak "krzaczków" floating w środku
- ≥200 punktów per scan

Słaby slice (za wąski) → rzadki scan, dziury w mapie.

### Test 8 — odom drift

```bash
# robot stojący ~2 minuty w miejscu:
ros2 topic echo /dog_odom --field 'pose.pose.position'
# x, y powinny być stałe ±2 cm
# Jeśli dryfuje >10 cm — firmware odom słaby, rozważ FAST-LIO
```

## Aktualne wartości w configu deploy (po fix)

`slam_toolbox_mapping.yaml`:
```yaml
max_laser_range: 20.0           # było 8.0 — za mało dla Mid-360
minimum_travel_distance: 0.05   # było 0.2 — za rzadko przy ~0.12 m/s
minimum_travel_heading: 0.05    # było 0.2 — za rzadko
minimum_time_interval: 0.2      # było 0.1 — bardziej restrykcyjne
```

`pointcloud_to_laserscan.yaml`:
```yaml
min_height: 0.5                 # Tomasz, real-tested (alternatywa: 0.15)
max_height: 0.75                # Tomasz, real-tested (alternatywa: 1.40)
transform_tolerance: 0.20
queue_size: 40
```

## Gdy 2D nadal słabe — alternatywa

Pakiet `g1_courier_fastlio` daje standalone FAST-LIO2 mapping (3D LiDAR-Inertial)
do porównania. Patrz [docs/fast_lio_setup.md](fast_lio_setup.md).

Plus jeśli chcesz porównać side-by-side:
1. Uruchom `mapping_real.launch.py` — zapisz mapę jako `~/maps/lab_slam.pgm`
2. Uruchom `fastlio_mapping.launch.py` — zapisz `g1_map.pcd`, konwersja do
   `~/maps/lab_fastlio.pgm` (patrz fast_lio_setup.md)
3. Porównaj jakość, dryf, fragmentaryczność.
