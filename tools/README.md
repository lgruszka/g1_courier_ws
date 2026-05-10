# Diagnostic Tools

Standalone Python helpers that subscribe to ROS2 topics. Work transparently
with both sim and real robot.

## `cam_viewer.py`

Live `head_cam` preview with AprilTag bounding box and PnP distance overlay.

```bash
source ~/g1_courier_ws/install/setup.bash
python3 tools/cam_viewer.py
```

Subscribes:
- `/head_cam/image_raw` (sensor_msgs/Image, rgb8)
- `/detections` (apriltag_msgs/AprilTagDetectionArray)
- `/camera_info` (sensor_msgs/CameraInfo, transient_local)

Per detection runs `cv2.solvePnPGeneric` (same as `dock_action_server`) and
displays `z_c` (depth along optical axis), full 3D distance, and `(x, y)`
offset in cam frame.

Keys: `q` quit, `s` snapshot to `/tmp/cam_snap.png`.

## `lidar_viewer.py`

Live 2D top-down view of `/scan` with RANSAC line fit (matches
`dock_action_server.LidarLineAligner` algorithm).

```bash
python3 tools/lidar_viewer.py
```

Visualises:
- All scan points (gray = outside forward window, blue = inside aligner's
  ±30° cone, red = RANSAC inliers)
- Forward window cone (`angle_min .. angle_max`)
- `target_distance` line (where dock_to_table converges)
- RANSAC fit line (green) — what dock_action_server sees as "table edge"

Keys: `q` quit, `s` snapshot to `/tmp/lidar_snap.png`.

## `plan_viz.py`

Renders nav2 global plan + AMCL pose + costmap inflation to
`/tmp/nav_plan.png` on every plan update. Use with auto-reload viewer:

```bash
python3 tools/plan_viz.py &
feh --reload 1 /tmp/nav_plan.png   # or eog + manual F5
```

Subscribes `/map`, `/global_costmap/costmap`, `/plan`, `/amcl_pose`.
