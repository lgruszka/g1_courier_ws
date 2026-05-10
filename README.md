# g1_courier_ws

ROS2 stack for a Unitree G1 humanoid carrying a box between two desks
marked with AprilTags.

> **Where you are now**: this is the `courier-deploy` branch — code for the
> **real Unitree G1**. If you want to play with the simulator first
> (recommended), `git checkout courier-sim` and read that branch's README.

## TL;DR — first time on the team?

1. **Pick your branch** (see [Branches](#branches) below):
   - You have only a laptop → `courier-sim`
   - You're at the lab with a real G1 → `courier-deploy` (this one)
2. **Skim the architecture diagram** ([here](#architecture-diagram)) so you
   know which layer owns what. You don't need to understand it deeply yet.
3. **Build the workspace** ([5 minutes](#build-and-source)).
4. **Run the full mission** ([Running the full mission](#running-the-full-mission)) —
   one launch file brings everything up.
5. **When something breaks**, look at:
   - [Single-skill debugging](#single-skill-debugging) — fire one action, see what happens
   - [Diagnostic viewers](#diagnostic-viewers) — visual feedback for camera / lidar / nav
   - [docs/deployment_guide.md](docs/deployment_guide.md) — full troubleshooting
6. **When you change code**, read [docs/ARCHITECTURE.md](docs/ARCHITECTURE.md)
   first — it is the source of truth for design decisions and code style.

## Branches

The team repo (`gitlab.com/iAndy77/j2s`) has three orphan branches with
independent histories. Pick the one matching your target — they all
share the mission/skills/platform layers but differ in what's running
underneath them.

| Branch | What it is | When to use |
|---|---|---|
| **`courier-sim`** | Ubuntu native MuJoCo bridge + full mission stack | Daily development, no real robot needed |
| **`courier-deploy`** | Real Unitree G1 + Livox Mid-360 + RealSense D435i wiring | Lab sessions with the actual robot |
| `courier-sim-legacy-mac` | Frozen snapshot of the original mac MuJoCo + Linux Parallels setup | Historical reference only — not maintained |

Switching branches:
```bash
cd ~/g1_courier_ws/src/courier
git fetch
git checkout courier-sim       # or courier-deploy
cd ../..
rm -rf build install log        # clear stale artifacts from other branch
colcon build --symlink-install
source install/setup.bash
```

## Hardware prerequisites (this branch)

For sim, see `courier-sim` README — none of this section applies.

- Unitree G1 (29-DoF) with `arm_sdk` enabled and sport API ready
- Livox Mid-360 mounted on head, USB connected to onboard PC
- RealSense D435i in chest plate, USB3
- Two desks (table A, table B) with AprilTag `tag36h11`:
  - Table A → id `5`, side length 0.16 m, on front edge
  - Table B → id `7`, same family/size
- Cardboard box with tag `id=10` on top face (size 0.10 m)
- Saved 2D map at `~/maps/lab.yaml` (build with `mapping_real.launch.py` once per lab)
- Calibrated waypoints in `src/g1_courier_mission/config/waypoints.yaml`

Full hardware checklist + calibration procedure:
[docs/deployment_guide.md](docs/deployment_guide.md).

## Build and source

```bash
cd ~/g1_courier_ws
colcon build --symlink-install
source install/setup.bash
# Add `source ~/g1_courier_ws/install/setup.bash` to your ~/.bashrc to skip
# the source step in future shells.
```

If the build fails, the most common causes are:

- Missing apt deps → see [Required external dependencies](#required-external-dependencies)
- Missing `unitree_hg` / `unitree_api` → clone `unitree_ros2` into `src/`
  (see deployment guide § "Workspace + sources")

## Running the full mission

One command brings up the entire stack: nav2, AMCL, AprilTag detector,
LiDAR-to-laserscan converter, dock / pick / place / retreat servers,
mission BT.

```bash
ros2 launch g1_courier_bringup real.launch.py map:=$HOME/maps/lab.yaml
```

What happens, step by step:

1. AMCL loads `lab.yaml`, waits for an initial pose (set it from RViz
   "2D Pose Estimate" if it doesn't auto-converge).
2. Mission BT navigates to predock for table A.
3. Dock APRILTAG → tag5 (30 cm), then re-dock → box tag10 (17 cm).
4. `pick_box` runs P0..P6 keyframes, grasp verifier confirms via τ jump.
5. Carry mode engages — `cmd_vel_arbiter` lowers velocity caps.
6. Navigate to predock for table B.
7. Dock LIDAR_LINE (head_cam is occluded by the carried box).
8. `place_box` runs P5..ZERO, release verifier confirms τ drop.
9. Retreat 1.0 m, swap A↔B, repeat.

Stop the mission cleanly with `Ctrl+C` in the launch terminal.

> **Watching what happens**: open RViz alongside (`ros2 run rviz2 rviz2`)
> with Map, AMCL Pose, Nav2 Plan, and TF displays. Plus the
> [diagnostic viewers](#diagnostic-viewers) below.

### Limit cycles for testing

By default the BT loops forever. To run N cycles only:

```bash
ros2 launch g1_courier_bringup real.launch.py \
  map:=$HOME/maps/lab.yaml \
  max_cycles:=3
```

(`max_cycles` is a parameter on `mission_node`; set to 0 = infinite.)

## Single-skill debugging

When you want to test one action server in isolation. Useful for:

- Verifying a calibration change without the full mission cycle
- Reproducing a bug that only shows in one phase
- Onboarding — feel out the contracts one at a time

All examples below assume `real.launch.py` is running in another terminal,
or at least the relevant action server has been started.

### Pick

```bash
# Use nominal P0..P6 keyframes (no offset from a measured box pose).
ros2 action send_goal /pick_box g1_courier_msgs/action/PickBox \
  '{box_pose: {header: {frame_id: ""}}, sequence_name: "pick_box", timeout_s: 30.0}' \
  --feedback
```

You'll see phase progression `wait_for_state → approach → grasp → lift → verify`.
Result includes `grasp_verified: true|false`.

### Place

```bash
ros2 action send_goal /place_box g1_courier_msgs/action/PlaceBox \
  '{target_pose: {header: {frame_id: ""}}, sequence_name: "place_box", timeout_s: 30.0}' \
  --feedback
```

### Dock to a table (AprilTag mode)

```bash
ros2 action send_goal /dock_to_table g1_courier_msgs/action/DockToTable \
  '{mode: 0, apriltag_id: 5,
    target_pose: {header: {frame_id: "tag_a"}},
    xy_tolerance_m: 0.03, yaw_tolerance_rad: 0.05, timeout_s: 25.0}' \
  --feedback
```

`mode: 0` is `MODE_APRILTAG`, `1` is `MODE_LIDAR_LINE`, `2` is `MODE_AMCL_ONLY`
(see `src/g1_courier_msgs/action/DockToTable.action`).

### Dock to a table (LiDAR line mode — for when carrying a box)

```bash
ros2 action send_goal /dock_to_table g1_courier_msgs/action/DockToTable \
  '{mode: 1, apriltag_id: 0,
    target_pose: {header: {frame_id: "map"},
                  pose: {position: {x: 4.10, y: 0.0, z: 0.0}}},
    xy_tolerance_m: 0.03, yaw_tolerance_rad: 0.05, timeout_s: 25.0}' \
  --feedback
```

### Navigate to a 2D pose

```bash
ros2 action send_goal /courier/navigate_to_pose \
  g1_courier_msgs/action/NavigateToPose \
  '{target_pose: {header: {frame_id: "map"},
                  pose: {position: {x: 1.42, y: 0.0, z: 0.0},
                         orientation: {w: 1.0}}},
    waypoint_name: "predock_a", xy_tolerance_m: 0.0, yaw_tolerance_rad: 0.0,
    timeout_s: 60.0}' \
  --feedback
```

(`xy_tolerance_m: 0.0` means "use nav2 default from `nav2_params.yaml`".)

### Retreat (open-loop reverse)

```bash
ros2 action send_goal /retreat g1_courier_msgs/action/Retreat \
  '{distance_m: 0.5, speed_mps: 0.15, timeout_s: 10.0}' \
  --feedback
```

### Toggle carry mode (services on cmd_vel_arbiter)

```bash
# Engage carry-mode velocity caps:
ros2 service call /safety/set_carry_mode g1_courier_msgs/srv/SetCarryMode \
  '{carrying: true}'

# Freeze locomotion (publishes zero velocity regardless of upstream):
ros2 service call /safety/set_freeze g1_courier_msgs/srv/SetFreeze \
  '{freeze: true}'
```

### Inspect mission BT state

```bash
# Latched mission status (cycle counter, current phase):
ros2 topic echo /mission_status --once

# Live BT tick logs (the BT prints stage transitions):
ros2 node info /mission_node
```

## Diagnostic viewers

Run alongside `real.launch.py` (each in its own terminal):

```bash
python3 tools/cam_viewer.py
# Opens window with /head_cam image + AprilTag bounding boxes + PnP distance.
# Useful for: checking exposure, focus, tag detection range.

python3 tools/lidar_viewer.py
# Top-down 2D scan + RANSAC line fit (matches dock LIDAR_LINE algorithm).
# Useful for: checking pcl-to-laserscan slice height, dock alignment.

python3 tools/plan_viz.py
# Saves /tmp/nav_plan.png on every nav2 plan update.
# View with: feh --reload 1 /tmp/nav_plan.png
# Useful for: checking AMCL convergence + nav2 path quality.
```

See `tools/README.md` for keyboard shortcuts (`q`/`s`).

## Building a map (once per lab)

```bash
ros2 launch g1_courier_bringup mapping_real.launch.py
# In another terminal:
ros2 run teleop_twist_keyboard teleop_twist_keyboard
# Drive the robot through the lab, covering both tables and the transit area.
# When the map looks complete in RViz:
ros2 run nav2_map_server map_saver_cli -f ~/maps/lab
```

Then update `src/g1_courier_mission/config/waypoints.yaml` from the new
map — see deployment guide § "Calibrate waypoints from map" for the
RViz-based procedure.

## Architecture diagram

```
┌─────────────────────────────────────────────────────────────┐
│  Mission layer       g1_courier_mission                     │
│  - py_trees_ros Behavior Tree                               │
│  - blackboard: cycle_count, current_target, box_held        │
│  - calls skills as ROS2 actions, retries on failure         │
└──────────────┬──────────────────────────────────────────────┘
               │ ROS2 actions (g1_courier_msgs)
┌──────────────┴──────────────────────────────────────────────┐
│  Skills layer                                               │
│  ┌─────────────────────┐ ┌────────────────────────────────┐ │
│  │ NavigateToPose      │ │ DockToTable                    │ │
│  │ wraps nav2          │ │ MODE_APRILTAG / LIDAR / AMCL   │ │
│  └─────────────────────┘ └────────────────────────────────┘ │
│  ┌─────────────────────┐ ┌────────────────────────────────┐ │
│  │ PickBox / PlaceBox  │ │ Retreat (open-loop reverse)    │ │
│  └─────────────────────┘ └────────────────────────────────┘ │
└──────────────┬──────────────────────────────────────────────┘
               │ /cmd_vel_*, /arm_sdk, /lowstate, TF, /scan
┌──────────────┴──────────────────────────────────────────────┐
│  Platform layer                                             │
│  ┌─────────────────────┐ ┌────────────────────────────────┐ │
│  │ Nav2 stack          │ │ Localization                   │ │
│  │ planner / controller│ │ slam_toolbox (mapping)         │ │
│  └─────────────────────┘ │ nav2_amcl (running)            │ │
│  ┌─────────────────────┐ └────────────────────────────────┘ │
│  │ cmd_vel_arbiter     │ ┌────────────────────────────────┐ │
│  │ - priority routing  │ │ Sensors                        │ │
│  │ - carry-mode limits │ │ Mid360 LiDAR + RealSense D435i │ │
│  │ - freeze + e-stop   │ │ apriltag_ros                   │ │
│  └─────────────────────┘ │ pointcloud_to_laserscan        │ │
│                          └────────────────────────────────┘ │
└─────────────────────────────────────────────────────────────┘
                              │
                              ↓ /lowstate, /livox/lidar, /camera/...
                       Unitree firmware bridges +
                       Livox + RealSense drivers
```

The vertical rule: each layer talks **down** through ROS2 actions /
topics defined in `g1_courier_msgs`. The skill layer never calls another
skill — the mission BT composes them.

## Packages

| Package | Type | Role |
|---|---|---|
| `g1_courier_msgs` | ament_cmake | Action / srv / msg interfaces — the API of the system |
| `g1_courier_arm_skills` | ament_python | `PickBox` and `PlaceBox` action servers, parametric arm controller |
| `g1_courier_docking` | ament_python | `DockToTable` action server with AprilTag / LiDAR / AMCL modes |
| `g1_courier_mission` | ament_python | Behavior Tree mission node, `NavigateToPose` proxy, `Retreat` |
| `g1_courier_safety` | ament_python | `cmd_vel` arbiter with priority, carry mode, freeze and e-stop |
| `g1_courier_bringup` | ament_python | Launch files and configs (nav2, slam_toolbox, AMCL, AprilTag, ...) |

## Mission flow (the BT executes this)

```
loop forever:
  set_carry_mode(off)
  navigate_to_pose(predock_table_A)        # nav2 + AMCL
  dock_to_table(mode=APRILTAG, tag=A)      # 6-DoF visual servo
  pick_box(box_pose_from_tag)              # parametric arm trajectory
  verify_grasp                             # τ threshold
  set_carry_mode(on)                       # lower vx/vyaw, smaller steps

  navigate_to_pose(predock_table_B)        # nav2 + AMCL only
  dock_to_table(mode=LIDAR_LINE, tag=B)    # camera occluded by box
  place_box(target_pose_from_lidar)        # parametric place
  verify_release

  set_carry_mode(off)
  retreat(1.0 m)
  swap A <-> B
```

The BT has retry policies on dock and pick. A failed `verify_grasp`
re-runs dock + pick. A failed `verify_release` escalates to abort.

## How the camera-occlusion problem is solved

When the robot carries the box, the front camera view is mostly blocked,
so AprilTag detection becomes unreliable. The stack handles this by
**never relying on AprilTag for global localization**. Globally we
always run AMCL on a 2D laser scan derived from the LiDAR. AprilTag is
used only to refine the final approach to the pick desk (because the
pick must hit ±2-3 cm tolerance). For the place desk we approach with
two cheaper means combined:

1. AMCL pose, which is already accurate to roughly ±5 cm in a
   well-mapped environment.
2. LiDAR line fitting against the table edge (`MODE_LIDAR_LINE`), which
   corrects the residual lateral and yaw error directly from the scan,
   regardless of camera state.

The dock action takes a `mode` argument so the mission BT chooses
per-table what level of refinement is required.

## Coordination between locomotion and arms

- `cmd_vel_arbiter` exposes a `freeze` service. Before any arm action
  the mission node sets freeze to true, the arbiter publishes zero
  velocities, and the arm action waits until lowstate reports body
  velocity below a threshold before issuing the first arm setpoint.
- `cmd_vel_arbiter` exposes a `carry_mode` service. With box held it
  caps `max_vx`, `max_vy`, and `max_vyaw` at a reduced level (config)
  and switches the duration field accordingly.
- After `place_box` the arms are brought to zero with weight ramped to
  0, returning control to the FSM.

## Configs you'll likely need to tune

| File | What to tune | When |
|---|---|---|
| `mission/config/waypoints.yaml` | `predock_x/y/yaw` per table | Every new map |
| `arm_skills/config/arm_skills.yaml` | `grasp_tau_threshold_nm` | Per box weight |
| `docking/config/docking.yaml` | `kp_xy`, `kp_yaw`, `target_distance` | First deploy + per-lighting |
| `safety/config/safety.yaml` | `max_v*_carry` | Per box weight + balance |
| `bringup/config/nav2_params.yaml` | costmap inflation, footprint | Per lab clutter |

Why-and-how for each is in the [deployment guide](docs/deployment_guide.md)
§ "Configs that you may need to tune".

## What is implemented

- All action / service / message interfaces (`g1_courier_msgs`).
- Parametric arm controller with CRC, keyframe library (`P0..P6` from
  real G1 calibration), weight ramping, grasp verifier hook,
  kinematic-mode sentinel (`mode==99`) for sim-side joint forcing.
- `PickBox` and `PlaceBox` action servers with grasp verifier
  integration.
- `DockToTable` action server with all three modes:
  - `MODE_APRILTAG` — 6-DoF PnP visual servo (table tag5/7 or box tag10)
  - `MODE_LIDAR_LINE` — RANSAC line fit on 2D scan, perpendicular alignment
    (used when carried box occludes head_cam)
  - `MODE_AMCL_ONLY` — trust AMCL (fallback)
- `cmd_vel_arbiter` with priority routing (dock → retreat → nav →
  /cmd_vel), carry-mode velocity caps, freeze service, e-stop latch.
- Behavior Tree mission cycle: `pickup_at_a → transfer_b → pickup_b →
  transfer_a` with `[STAGE START]`/`[STAGE END]` timing logs.
- nav2 stack: AMCL OmniMotionModel + NavfnPlanner A\* + RotationShim →
  RegulatedPurePursuit + costmaps with obstacle/inflation layers.

## Required external dependencies

apt (ROS2 Jazzy):
```
ros-jazzy-nav2-bringup
ros-jazzy-slam-toolbox
ros-jazzy-pointcloud-to-laserscan
ros-jazzy-py-trees-ros
ros-jazzy-rosidl-generator-dds-idl
ros-jazzy-apriltag-ros
ros-jazzy-apriltag-msgs
ros-jazzy-tf2-geometry-msgs
ros-jazzy-realsense2-camera
ros-jazzy-teleop-twist-keyboard
```

ROS2 source (clone into `src/`):
```
unitree_ros2          unitree_hg / unitree_api / unitree_go IDLs
                      https://github.com/unitreerobotics/unitree_ros2
unitree_ros           g1_description URDF + meshes (master branch)
                      https://github.com/unitreerobotics/unitree_ros
livox_ros_driver2     Livox Mid-360 driver
                      https://github.com/Livox-SDK/livox_ros_driver2
```

## Documentation index

- [docs/ARCHITECTURE.md](docs/ARCHITECTURE.md) — design rules, layer
  boundaries, anti-patterns. **Read before changing core code.**
- [docs/deployment_guide.md](docs/deployment_guide.md) — full hardware
  setup, calibration procedure, troubleshooting.
- [tools/README.md](tools/README.md) — diagnostic viewer details.
- [docs/phases/](docs/phases/) — historical chronicle of the
  development phases (sim-focused; helpful for understanding *why* the
  current design exists).
