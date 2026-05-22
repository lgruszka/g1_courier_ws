# Narzędzia diagnostyczne

Standalone helpery Pythona subskrybujące topiki ROS2. Działają
identycznie z sim i z realnym robotem.

## `cam_viewer.py`

Live preview `head_cam` z bbox AprilTagów i overlay'em dystansu z PnP.

```bash
source ~/g1_courier_ws/install/setup.bash
python3 tools/cam_viewer.py
```

Subskrybuje:
- `/head_cam/image_raw` (sensor_msgs/Image, rgb8)
- `/detections` (apriltag_msgs/AprilTagDetectionArray)
- `/camera_info` (sensor_msgs/CameraInfo, transient_local)

Per detekcja uruchamia `cv2.solvePnPGeneric` (ten sam algorytm co w
`dock_action_server`) i wyświetla `z_c` (głębokość wzdłuż osi optycznej),
pełen dystans 3D, plus offset `(x, y)` w cam frame.

Klawisze: `q` quit, `s` snapshot do `/tmp/cam_snap.png`.

## `lidar_viewer.py`

Live top-down view 2D z `/scan` z RANSAC line fit (dopasowany do
algorytmu `dock_action_server.LidarLineAligner`).

```bash
python3 tools/lidar_viewer.py
```

Wizualizuje:
- Wszystkie punkty scanu (szary = poza forward window, niebieski =
  w stożku ±30° aligner'a, czerwony = inliers RANSAC)
- Stożek forward window (`angle_min .. angle_max`)
- Linia `target_distance` (gdzie zbiega dock_to_table)
- Linia RANSAC fit (zielona) — co dock_action_server widzi jako
  "krawędź biurka"

Klawisze: `q` quit, `s` snapshot do `/tmp/lidar_snap.png`.

## `plan_viz.py`

Renderuje globalny plan nav2 + AMCL pose + inflation costmapy do
`/tmp/nav_plan.png` przy każdym update'cie planu. Używaj z viewerem
auto-reload:

```bash
python3 tools/plan_viz.py &
feh --reload 1 /tmp/nav_plan.png   # albo eog + ręczne F5
```

Subskrybuje `/map`, `/global_costmap/costmap`, `/plan`, `/amcl_pose`.

## `run_mapping_session.sh`

End-to-end orchestrator FAST-LIO mapping. Odpala FAST-LIO, czeka aż
przejedziesz scenę + Enter, wywołuje `/map_save`, generuje wiele
wariantów PGM, otwiera picker GUI.

```bash
# T1 — Livox driver (osobno):
ros2 launch livox_ros_driver2 msg_MID360_launch.py

# T2 — orchestrator:
./tools/run_mapping_session.sh
```

Env vars: `PCD_OUT`, `SCENARIOS_DIR`, `FLIP_Y` (1 dla Mid-360 upside-down).

## `pcd_variant_grid.py`

Batch generator wariantów mapy z jednego PCD (~30 kombinacji
Z-slice × resolution × min_points). Tworzy `manifest.json` z metadanymi
do `map_picker.py`.

```bash
python3 tools/pcd_variant_grid.py ~/maps/last_session.pcd ~/maps/scenarios --flip-y
```

## `map_picker.py`

PyQt5 GUI: lista wariantów + preview PGM + Save as production
(`~/maps/lab.yaml` + `lab.pgm` — odbierane przez `real.launch.py`).

```bash
python3 tools/map_picker.py ~/maps/scenarios
```

Wymaga `manifest.json` (generowane przez `pcd_variant_grid.py`).

## `scan_height_tuner.py`

**Live tuner** dla `pointcloud_to_laserscan` `min_height/max_height`.
Zmiana wartości leci do node'a przez `ros2 param set` — widoczna
**natychmiast** w RViz LaserScan display. Bez restartu launchu.

```bash
# T1: real.launch.py odpalone
# T2: rviz2 z LaserScan /scan
# T3: tuner:
python3 tools/scan_height_tuner.py
```

Slidery + 5 preset buttons + status (Hz, valid pts %). Po znalezieniu
dobrego slice → **Save to yaml**.

Wymaga: `python3-pyqt5 python3-yaml`.

## `fastlio_pcd_to_pgm.py`

Single conversion PCD → PGM (manual, dla ad-hoc). Patrz
`pcd_variant_grid.py` dla batch generowania.
