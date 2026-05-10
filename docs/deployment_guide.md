# Real-robot deployment guide (`courier-deploy` branch)

End-to-end procedure for bringing the courier mission up on a physical
Unitree G1 with Livox Mid-360 and RealSense D435i.

> Sim users: this guide does not apply. Switch to `courier-sim` branch.

## Hardware checklist

- [ ] Unitree G1 (29-DoF) with `arm_sdk` enabled and sport API ready
- [ ] Livox Mid-360 mounted on head, USB connected to onboard PC
- [ ] RealSense D435i mounted in chest plate, USB3
- [ ] Two desks (table A, table B) marked with AprilTag `tag36h11`:
  - Table A → id `5`, side length 0.16 m, mounted on table front edge
  - Table B → id `7`, same family/size
- [ ] Cardboard parcel with tag `id=10` on top face (size 0.10 m)
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
3. Dock APRILTAG to tag5 (30 cm), then to parcel tag10 (17 cm)
4. `pick_box` runs P0..P6 sequence → `grasp_verified=true`
5. Carry mode engaged (lower velocity caps from `safety.yaml`)
6. Navigate to predock_b
7. Dock LIDAR_LINE (camera occluded by parcel)
8. `place_box` runs P5..P0 → `release_verified=true`
9. Retreat 0.5 m, swap A/B, repeat

Fail signs and where to look:

- **AMCL pose drifts**: re-record map, increase coverage in transit area
- **`grasp_verified=false`**: retune `grasp_tau_threshold_nm` in
  `src/g1_courier_arm_skills/config/arm_skills.yaml`. Real value depends
  on parcel weight — start at 1.5, raise if false negatives.
- **Dock APRILTAG never converges**: check `head_cam` actually publishes
  rectified image (`image_rect`); verify intrinsics in `camera_info`
  match D435i factory values
- **Dock LIDAR_LINE wanders**: increase RANSAC inlier threshold or narrow
  the forward window in `src/g1_courier_docking/config/docking.yaml`
- **Robot wobbles in carry mode**: tighten `max_v*_carry` in
  `safety.yaml`

## Configs that you may need to tune

| File | What to tune | When |
|---|---|---|
| `mission/config/waypoints.yaml` | predock per table | every new map |
| `arm_skills/config/arm_skills.yaml` | `grasp_tau_threshold_nm` | per parcel weight |
| `docking/config/docking.yaml` | dock kp_xy/kp_yaw, target_distance | first deploy + per‐lighting |
| `safety/config/safety.yaml` | carry-mode v limits | per parcel weight + balance |
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
