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
