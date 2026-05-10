# Real-robot deployment guide (`courier-deploy` branch)

End-to-end procedure for bringing the courier mission up on a physical
Unitree G1 with Livox Mid-360 and RealSense D435i.

> Sim users: this guide does not apply. Switch to `courier-sim` branch.

## Reading order for first-time use

If you've never deployed this stack on a real robot before, follow the
sections in this order — each builds on the previous:

1. [Hardware checklist](#hardware-checklist) — verify every box ticks
2. [Software install (one-time)](#software-install-one-time) — packages, repo, drivers
3. [Day 1 walkthrough](#day-1-walkthrough) — your first 60 minutes with the robot
4. [Build a map](#build-a-map-once-per-lab-layout) — slam_toolbox + Livox slice
5. [Calibrate waypoints from map](#calibrate-waypoints-from-map) — RViz pick-points
6. [First mission run](#first-mission-run) — full A↔B cycle
7. [Troubleshooting](#troubleshooting) — when things break

If you've already done install + map + calibration on a previous visit,
skip straight to [First mission run](#first-mission-run).

## Hardware checklist

- [ ] Unitree G1 (29-DoF) with `arm_sdk` enabled and sport API ready
- [ ] Livox Mid-360 mounted on head, USB connected to onboard PC
- [ ] RealSense D435i mounted in chest plate, USB3
- [ ] Two desks (table A, table B) marked with AprilTag `tag36h11`:
  - Table A → id `5`, side length 0.16 m, mounted on table front edge
  - Table B → id `7`, same family/size
- [ ] Cardboard box with tag `id=10` on top face (size 0.10 m)
- [ ] WiFi or ethernet between onboard PC and dev laptop (for RViz / debug)

## Software install (one-time)

### 1. ROS2 Jazzy + system packages

```bash
sudo apt install -y \
  ros-jazzy-nav2-bringup \
  ros-jazzy-slam-toolbox \
  ros-jazzy-pointcloud-to-laserscan \
  ros-jazzy-py-trees-ros \
  ros-jazzy-rosidl-generator-dds-idl \
  ros-jazzy-apriltag-ros \
  ros-jazzy-apriltag-msgs \
  ros-jazzy-tf2-geometry-msgs \
  ros-jazzy-realsense2-camera \
  ros-jazzy-teleop-twist-keyboard
```

### 2. Workspace + sources

```bash
mkdir -p ~/g1_courier_ws/src && cd ~/g1_courier_ws/src
git clone https://gitlab.com/iAndy77/j2s.git -b courier-deploy courier
ln -s courier/src/* .

# Unitree IDLs
git clone https://github.com/unitreerobotics/unitree_ros2
touch unitree_ros2/example/COLCON_IGNORE

# G1 URDF + meshes (master branch — NOT unitree_ros2)
git clone https://github.com/unitreerobotics/unitree_ros
# Use unitree_ros/robots/g1_description for robot_state_publisher.

# Livox driver
git clone https://github.com/Livox-SDK/livox_ros_driver2
```

### 3. Build

```bash
cd ~/g1_courier_ws
colcon build --symlink-install
source install/setup.bash
```

### 4. Firmware-side bridges

The Unitree G1 firmware exposes these DDS topics via the onboard SDK:

- `/lowstate` — joint positions, IMU, foot contacts (publish)
- `/lowcmd` — joint targets w/ CRC (subscribe; consumed by `arm_sdk`)
- `/cmd_vel` — locomotion command (subscribe; consumed by sport API)

Make sure the firmware-side bridge is running before launching this stack.
Check with:

```bash
ros2 topic hz /lowstate           # expect ~500 Hz
ros2 topic list | grep livox      # /livox/lidar
ros2 topic list | grep camera     # /camera/color/image_raw
```

## Day 1 walkthrough

Concrete sequence for your first session at the lab. Assumes the
[hardware](#hardware-checklist) plus [software install](#software-install-one-time)
are done.

### Minute 0–10: bring up the platform

1. Power on the G1 and let it stand. Verify it's in damping mode (zero
   torque, joints free).
2. Plug Livox + RealSense USB to onboard PC.
3. Start the firmware bridge (proprietary, sport API). Confirm:
   ```bash
   ros2 topic hz /lowstate          # ~500 Hz
   ros2 topic list | grep -E 'livox|camera'
   # /livox/lidar  /camera/color/image_raw  /camera/color/camera_info
   ```
4. From your dev laptop on the same network:
   ```bash
   export ROS_DOMAIN_ID=<same as robot>
   ros2 topic list   # should see all of the above
   ```

### Minute 10–25: smoke-test individual components

Don't run the mission yet — verify each piece works in isolation. Open
4 terminals on your dev laptop. In each one source the workspace:
```bash
source ~/g1_courier_ws/install/setup.bash
```

**Terminal 1** — sensors visualised:
```bash
python3 ~/g1_courier_ws/tools/cam_viewer.py
# Check: image arrives, exposure OK, you can see the lab in front of the robot.
```

**Terminal 2** — LiDAR slice:
```bash
ros2 launch g1_courier_bringup mapping_real.launch.py
# In another terminal:
python3 ~/g1_courier_ws/tools/lidar_viewer.py
# Check: 360° scan visible, walls/desks readable, no obvious gaps.
```

If the lidar slice is empty or sparse, your `min_height/max_height`
in `bringup/config/pointcloud_to_laserscan.yaml` doesn't match the
Livox mount height. The defaults assume Livox at ~1.45 m AGL on the
G1 head.

**Terminal 3** — manual drive test:
```bash
ros2 run teleop_twist_keyboard teleop_twist_keyboard
```
Use sparingly — push w/x at small speeds (`0.1 m/s`). Confirm the
robot translates as commanded. **Stop teleop before continuing.**

**Terminal 4** — kill `mapping_real.launch.py` from terminal 2 with
`Ctrl+C` once everything above looks good.

### Minute 25–45: build the map

```bash
ros2 launch g1_courier_bringup mapping_real.launch.py
# In a separate terminal:
ros2 run teleop_twist_keyboard teleop_twist_keyboard
ros2 run rviz2 rviz2
# Add → Map → topic /map. Watch the map fill in as you drive.
```

Drive the robot through the full lab layout: pass each table front,
each transit corridor, both directions. Slow steady speed (~0.2 m/s)
gives the cleanest map.

When the map looks complete:
```bash
ros2 run nav2_map_server map_saver_cli -f ~/maps/lab
```

You now have `~/maps/lab.pgm` + `~/maps/lab.yaml`.

### Minute 45–60: calibrate waypoints

See [Calibrate waypoints from map](#calibrate-waypoints-from-map)
below — clicks through RViz, edit `waypoints.yaml`, takes ~10 minutes.

After this, you're ready for [First mission run](#first-mission-run).

## Build a map (once per lab layout)

```bash
ros2 launch g1_courier_bringup mapping_real.launch.py
# In a second terminal:
ros2 run teleop_twist_keyboard teleop_twist_keyboard
# Walk the robot through the lab covering both tables and the transit area.
# When the map looks complete in RViz:
ros2 run nav2_map_server map_saver_cli -f ~/maps/lab
```

You now have `~/maps/lab.pgm` + `~/maps/lab.yaml`.

## Calibrate waypoints from map

`src/g1_courier_mission/config/waypoints.yaml` defines per-table predock
poses in **world (map) frame**. After saving a fresh map you must update
the X/Y/yaw of each predock so that:

- nav2 lands the robot ~30 cm in front of the table edge
- tag5/tag7 is centred in `head_cam` field of view from there

### Procedure

1. **Open the map in RViz**:
   ```bash
   ros2 run nav2_map_server map_server --ros-args \
     -p yaml_filename:=$HOME/maps/lab.yaml
   ros2 run rviz2 rviz2
   # Add → Map → topic /map
   ```

2. **Find each table edge in world frame**. Click "Publish Point" tool in
   RViz and click on the front edge of table A. RViz logs `(x, y)` in
   `/clicked_point`. Note the values.

3. **Predock pose**:
   - `predock_x = table_edge_x - 0.50` (50 cm in front, so the dock
     servo has room to advance ~13 cm without colliding)
   - `predock_y = table_edge_y` (centered)
   - `predock_yaw` = orientation that makes the robot **face** the table.
     For a table on the +X side: `0.0`. For −X side: `3.14159`. For −Y:
     `-1.5708`. Etc.

4. **Edit waypoints.yaml** with the new values:
   ```yaml
   tables:
     table_a:
       apriltag_id: 5
       predock_x: 1.42      # measured: table edge at 1.92 m, robot 0.50 m back
       predock_y: 0.00
       predock_yaw: 0.0
       dock_mode: apriltag
       final_xy_tol_m: 0.03
       final_yaw_tol_rad: 0.05
     table_b:
       apriltag_id: 7
       predock_x: 4.10
       ...
   ```

5. **Verify** by manually navigating to the predock and checking
   `head_cam` view:
   ```bash
   ros2 launch g1_courier_bringup real.launch.py map:=$HOME/maps/lab.yaml
   # In RViz, "2D Goal Pose" → click at predock_a position.
   # Wait for nav to finish, then in another terminal:
   ros2 topic echo /detections --once
   # Expect tag id=5 with non-zero corners and centre near (640, 480)/2.
   ```

6. Repeat for table B.

## First mission run

```bash
ros2 launch g1_courier_bringup real.launch.py map:=$HOME/maps/lab.yaml
```

Expected sequence:

1. AMCL converges on the saved map (set initial pose in RViz if needed)
2. Mission BT navigates to predock_a
3. Dock APRILTAG to tag5 (30 cm), then to box tag10 (17 cm)
4. `pick_box` runs P0..P6 sequence → `grasp_verified=true`
5. Carry mode engaged (lower velocity caps from `safety.yaml`)
6. Navigate to predock_b
7. Dock LIDAR_LINE (camera occluded by box)
8. `place_box` runs P5..P0 → `release_verified=true`
9. Retreat 0.5 m, swap A/B, repeat

## Troubleshooting

Each entry: **symptom** → diagnostic command → likely cause → fix.

### `colcon build` fails with `unitree_hg` not found
```bash
ls src/unitree_ros2/unitree_hg     # should exist
```
- **Cause**: `unitree_ros2` not cloned, or `example/COLCON_IGNORE` missing
  causing the example pkg to fail.
- **Fix**: clone per § "Workspace + sources"; `touch src/unitree_ros2/example/COLCON_IGNORE`.

### `ros2 topic list` doesn't show `/lowstate` or `/livox/lidar`
```bash
echo $ROS_DOMAIN_ID                # must match robot
ros2 daemon stop && ros2 daemon start
ros2 topic list
```
- **Cause**: domain ID mismatch, daemon stale, or firmware bridge not running.
- **Fix**: align `ROS_DOMAIN_ID` between robot and dev laptop, restart daemon.

### AMCL doesn't converge (robot icon stays at origin or drifts)
```bash
ros2 topic echo /amcl_pose --once   # check covariance and pose
ros2 topic hz /scan                 # should be ~10 Hz
ros2 run tf2_tools view_frames      # confirm map → odom → base_link
```
- **Cause 1**: no initial pose set.
  **Fix**: in RViz click "2D Pose Estimate", drag arrow on the map at robot's actual location.
- **Cause 2**: scan empty or very sparse.
  **Fix**: check `pointcloud_to_laserscan.yaml` slice heights match
  Livox mount; check `/livox/lidar` is publishing.
- **Cause 3**: poor map coverage.
  **Fix**: re-record map, walk through the transit area more thoroughly.

### Dock APRILTAG never converges
```bash
ros2 topic echo /detections --once   # tag id 5 or 7 visible?
python3 tools/cam_viewer.py          # visual check of bbox
```
- **Cause 1**: tag out of camera FoV from current predock pose.
  **Fix**: edit `predock_x/y/yaw` in `waypoints.yaml` so the tag is
  centred. Use cam_viewer to verify.
- **Cause 2**: D435i image is unrectified or `camera_info` has wrong
  intrinsics.
  **Fix**: ensure `realsense2_camera` publishes `image_rect`; verify
  `camera_info` matches D435i factory values (`fx ≈ 615`, `fy ≈ 615`,
  `cx ≈ 320`, `cy ≈ 240` for 640×480).
- **Cause 3**: lighting or motion blur.
  **Fix**: increase exposure, slow down dock approach, raise tag size.

### `grasp_verified=false` on every pick
```bash
ros2 topic echo /lowstate --field motor_state[<arm_idx>].tau_est
```
- **Cause**: `grasp_tau_threshold_nm` in `arm_skills.yaml` doesn't
  match real box weight + arm pose.
- **Fix**: capture baseline τ before pick, capture τ after lift,
  set threshold to 60% of the difference. Default 1.5 Nm is for ~1 kg
  box; scale linearly.

### Dock LIDAR_LINE wanders / never settles
```bash
python3 tools/lidar_viewer.py
# Drive to predock manually, watch the green RANSAC line.
```
- **Cause 1**: forward window too wide, aligner picking up obstacles
  beside the table.
  **Fix**: narrow `lidar_line.window_*` in `docking.yaml`.
- **Cause 2**: RANSAC inliers too loose, fitting noise.
  **Fix**: tighten `lidar_line.inlier_threshold_m`.
- **Cause 3**: wrong sign on yaw correction (was a known bug — see
  CLAUDE.md "Dock LIDAR_LINE timeout").
  **Fix**: verify `cmd.angular.z = +kp * yaw_err` in `LidarLineAligner`.

### Robot wobbles in carry mode (oscillates while walking)
```bash
ros2 topic echo /cmd_vel --once     # see what arbiter is publishing
```
- **Cause**: carry-mode velocity caps too high for current box weight.
- **Fix**: tighten `max_vx_carry`, `max_vy_carry`, `max_vyaw_carry` in
  `safety/config/safety.yaml`. Start with `0.2 / 0.1 / 0.3`.

### Mission BT loops the same phase forever
```bash
ros2 topic echo /mission_status --once
ros2 node info /mission_node
```
- **Cause**: a phase keeps timing out (e.g. dock A timeout on return
  leg because head_cam is occluded by the carried box).
- **Fix**: ensure `dock_mode: lidar_line` is set on the **return leg**
  table in `waypoints.yaml`. Both `table_a` and `table_b` should be
  `lidar_line` to be carry-independent.

### `nav2 controller "Passing new path" but robot stands still`
```bash
ros2 topic hz /cmd_vel              # 0 Hz means nothing reaches firmware
ros2 topic hz /cmd_vel_nav          # 10 Hz means nav2 IS publishing
```
- **Cause**: jazzy `nav2_bringup` remaps `cmd_vel` → `cmd_vel_nav`.
  Without `cmd_vel_arbiter` running in merge mode, the nav output
  goes nowhere.
- **Fix**: confirm `cmd_vel_arbiter` is running and subscribed to
  `/cmd_vel_nav`. `real.launch.py` does this for you.

### Old fail-recipes worth keeping in mind
Read `CLAUDE.md` § "Najczęstsze problemy które mogą wystąpić" — full
catalogue of bugs encountered during sim development. Most are
sim-specific (mac bridge, MuJoCo zombies) but a few apply to the real
robot too: NavfnPlanner tolerance, APRILTAG `dyaw` sign, LiDAR_LINE
yaw sign.

## Action contracts cheat sheet

When firing actions manually for debug, use these payloads as
templates. Full IDL: `src/g1_courier_msgs/{action,srv}/`.

```bash
# /pick_box  — nominal P0..P6 keyframes:
ros2 action send_goal /pick_box g1_courier_msgs/action/PickBox \
  '{box_pose: {header: {frame_id: ""}}, sequence_name: "pick_box", timeout_s: 30.0}' \
  --feedback

# /place_box  — nominal P5..ZERO:
ros2 action send_goal /place_box g1_courier_msgs/action/PlaceBox \
  '{target_pose: {header: {frame_id: ""}}, sequence_name: "place_box", timeout_s: 30.0}' \
  --feedback

# /dock_to_table APRILTAG (tag5):
ros2 action send_goal /dock_to_table g1_courier_msgs/action/DockToTable \
  '{mode: 0, apriltag_id: 5,
    target_pose: {header: {frame_id: "tag_a"}},
    xy_tolerance_m: 0.03, yaw_tolerance_rad: 0.05, timeout_s: 25.0}' \
  --feedback

# /dock_to_table LIDAR_LINE (predock at world (4.10, 0, 0)):
ros2 action send_goal /dock_to_table g1_courier_msgs/action/DockToTable \
  '{mode: 1, apriltag_id: 0,
    target_pose: {header: {frame_id: "map"},
                  pose: {position: {x: 4.10, y: 0.0, z: 0.0}}},
    xy_tolerance_m: 0.03, yaw_tolerance_rad: 0.05, timeout_s: 25.0}' \
  --feedback

# /courier/navigate_to_pose (predock_a at (1.42, 0, 0) facing +X):
ros2 action send_goal /courier/navigate_to_pose \
  g1_courier_msgs/action/NavigateToPose \
  '{target_pose: {header: {frame_id: "map"},
                  pose: {position: {x: 1.42, y: 0.0, z: 0.0},
                         orientation: {w: 1.0}}},
    waypoint_name: "predock_a",
    xy_tolerance_m: 0.0, yaw_tolerance_rad: 0.0, timeout_s: 60.0}' \
  --feedback

# /retreat — open-loop reverse 0.5 m at 0.15 m/s:
ros2 action send_goal /retreat g1_courier_msgs/action/Retreat \
  '{distance_m: 0.5, speed_mps: 0.15, timeout_s: 10.0}' \
  --feedback

# /safety/set_carry_mode — engage carry-mode caps:
ros2 service call /safety/set_carry_mode g1_courier_msgs/srv/SetCarryMode \
  '{carrying: true}'

# /safety/set_freeze — emergency freeze (zero cmd_vel):
ros2 service call /safety/set_freeze g1_courier_msgs/srv/SetFreeze \
  '{freeze: true}'
```

To **cancel** an action mid-flight: `Ctrl+C` in the `send_goal`
terminal. The action server respects cancel and stops cleanly.

To **list** all available actions:
```bash
ros2 action list
ros2 action info /pick_box        # shows server + client(s)
```

## Configs that you may need to tune

| File | What to tune | When |
|---|---|---|
| `mission/config/waypoints.yaml` | predock per table | every new map |
| `arm_skills/config/arm_skills.yaml` | `grasp_tau_threshold_nm` | per box weight |
| `docking/config/docking.yaml` | dock kp_xy/kp_yaw, target_distance | first deploy + per‐lighting |
| `safety/config/safety.yaml` | carry-mode v limits | per box weight + balance |
| `bringup/config/nav2_params.yaml` | costmap inflation, footprint | per lab clutter |

## Diagnostic tools

In a separate terminal alongside `real.launch.py`:

```bash
python3 ~/g1_courier_ws/tools/cam_viewer.py     # tag bbox + PnP distance
python3 ~/g1_courier_ws/tools/lidar_viewer.py   # /scan top-down + RANSAC
python3 ~/g1_courier_ws/tools/plan_viz.py       # nav2 plan + AMCL pose
```

## Sim parity

The same code paths run in sim (`courier-sim` branch). Differences:

- Sim runs MuJoCo via `g1_courier_sim/sim_bridge/`, real uses Unitree
  firmware bridges.
- Sim sets `kinematic_mode: true` on `pick`/`place` to bypass PD-via-DDS
  jitter; real keeps default `false`.
- Sim publishes `/parcel_state` to release a weld constraint on placement;
  real has no such topic (real fingers handle physical release).
- Sim uses `pupil_apriltags` inside the bridge process; real uses
  `apriltag_ros` as a separate node consuming D435i frames.

The mission BT, dock action server, arm controller, and nav2 stack are
**identical** across branches.
