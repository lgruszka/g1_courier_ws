# g1_courier_ws

ROS2 stack for a Unitree G1 humanoid carrying a parcel between two desks marked with AprilTags.

This workspace is a **starter package**. It defines the architecture, the contracts between layers (actions/services/topics), and provides scaffolds for every node. Some implementations are full, others are deliberate TODOs marked in code.

## Why a new project

The previous PoC (`j2s-light_tracking`) is a reactive AprilTag follower with a hardcoded arm sequence. It works for a demo, but it cannot:

- handle a multi-stage mission (approach → dock → pick → walk → dock → place → retreat → repeat),
- localise globally when the camera is occluded by the carried box,
- verify a grasp,
- recover from a failed phase,
- coordinate locomotion with manipulation explicitly,
- guarantee fine-pose accuracy required for picking.

This stack starts from a proper layered architecture and reuses only the parts of the old code that were correct: the `LowCmd` CRC, the arm SDK weight ramping, the keyframe poses (now parameterized), and the `cmd_vel` deadband/timeout discipline.

## Layered architecture

```
┌─────────────────────────────────────────────────────────────┐
│  Mission layer       g1_courier_mission                     │
│  - py_trees_ros Behavior Tree                               │
│  - blackboard: cycle_count, current_target, box_held        │
│  - calls skills as ROS2 actions, retries on failure         │
└──────────────┬──────────────────────────────────────────────┘
               │ ROS2 actions
┌──────────────┴──────────────────────────────────────────────┐
│  Skills layer                                               │
│  ┌─────────────────────┐ ┌────────────────────────────────┐ │
│  │ NavigateToPose      │ │ DockToTable                    │ │
│  │ wraps nav2 BT       │ │ MODE_APRILTAG / LIDAR / AMCL   │ │
│  │ g1_courier_mission/ │ │ g1_courier_docking             │ │
│  │  navigate_proxy.py  │ │                                │ │
│  └─────────────────────┘ └────────────────────────────────┘ │
│  ┌─────────────────────┐ ┌────────────────────────────────┐ │
│  │ PickBox / PlaceBox  │ │ Retreat                        │ │
│  │ g1_courier_arm_     │ │ short open-loop backup         │ │
│  │  skills             │ │ (in mission node)              │ │
│  └─────────────────────┘ └────────────────────────────────┘ │
└──────────────┬──────────────────────────────────────────────┘
               │ /cmd_vel, /arm_sdk, /lowstate, TF, scans
┌──────────────┴──────────────────────────────────────────────┐
│  Platform layer                                             │
│  ┌─────────────────────┐ ┌────────────────────────────────┐ │
│  │ Nav2 stack          │ │ Localization                   │ │
│  │ planner / controller│ │ slam_toolbox (mapping)         │ │
│  │ + custom BT         │ │ nav2_amcl (running)            │ │
│  └─────────────────────┘ └────────────────────────────────┘ │
│  ┌─────────────────────┐ ┌────────────────────────────────┐ │
│  │ cmd_vel_arbiter     │ │ Sensors                        │ │
│  │ - priority routing  │ │ Mid360 LiDAR + RealSense D435i │ │
│  │ - carry-mode limits │ │ apriltag_ros                   │ │
│  │ - freeze + e-stop   │ │ pointcloud_to_laserscan        │ │
│  └─────────────────────┘ └────────────────────────────────┘ │
│  ┌─────────────────────┐                                    │
│  │ Unitree bridges     │                                    │
│  │ /cmd_vel → sport API│                                    │
│  │ /arm_sdk → LowCmd   │                                    │
│  └─────────────────────┘                                    │
└─────────────────────────────────────────────────────────────┘
```

## Packages

| Package                  | Type         | Role                                                                |
| ------------------------ | ------------ | ------------------------------------------------------------------- |
| `g1_courier_msgs`        | ament_cmake  | Action / srv / msg interfaces - the API of the system               |
| `g1_courier_arm_skills`  | ament_python | `PickBox` and `PlaceBox` action servers, parametric arm controller  |
| `g1_courier_docking`     | ament_python | `DockToTable` action server with AprilTag / LiDAR / AMCL modes      |
| `g1_courier_mission`     | ament_python | Behavior Tree mission node, `NavigateToPose` proxy, `Retreat`       |
| `g1_courier_safety`      | ament_python | `cmd_vel` arbiter with priority, carry mode, freeze and e-stop      |
| `g1_courier_bringup`     | ament_python | Launch files and configs (nav2, slam_toolbox, AMCL, AprilTag, ...)  |

## Mission flow (the BT executes this)

```
loop forever:
  set_carry_mode(off)
  navigate_to_pose(predock_table_A)        # nav2 + AMCL
  dock_to_table(mode=APRILTAG, tag=A)      # 6-DoF visual servo
  pick_box(box_pose_from_tag)              # parametric arm trajectory
  verify_grasp                             # tau threshold
  set_carry_mode(on)                       # lower vx/vyaw, smaller steps

  navigate_to_pose(predock_table_B)        # nav2 + AMCL only
  dock_to_table(mode=LIDAR_LINE, tag=B)    # camera occluded by box
  place_box(target_pose_from_lidar)        # parametric place
  verify_release

  set_carry_mode(off)
  retreat(0.5 m)
  swap A <-> B
```

The BT has retry policies on dock and pick. A failed verify_grasp re-runs dock + pick. A failed verify_release escalates to abort.

## How the camera-occlusion problem is solved

When the robot carries the parcel, the front camera view is mostly blocked, so AprilTag detection becomes unreliable. The stack handles this by **never relying on AprilTag for global localization**. Globally we always run AMCL on a 2D laser scan derived from the LiDAR. AprilTag is used only to refine the final approach to the pick desk (because the pick must hit ±2-3 cm tolerance). For the place desk we approach with two cheaper means combined:

1. AMCL pose, which is already accurate to roughly ±5 cm in a well-mapped environment.
2. LiDAR line fitting against the table edge (`MODE_LIDAR_LINE`), which corrects the residual lateral and yaw error directly from the scan, regardless of camera state.

The dock action takes a `mode` argument so the mission BT chooses per-table what level of refinement is required.

## Coordination between locomotion and arms

- `cmd_vel_arbiter` exposes a `freeze` topic. Before any arm action the mission node sets freeze to true, the arbiter publishes zero velocities, and the arm action waits until lowstate reports body velocity below a threshold before issuing the first arm setpoint.
- `cmd_vel_arbiter` exposes a `carry_mode` topic. With box held it caps `max_vx`, `max_vy` and `max_vyaw` at a reduced level (config) and switches the duration field accordingly.
- After `place_box` the arms are brought to zero with weight ramped to 0, returning control to the FSM.

## Branches

This repo lives on three branches on the team GitLab (`gitlab.com/iAndy77/j2s`):

| Branch | Use case |
|---|---|
| **`courier-sim`** | Ubuntu native sim — primary path for team development |
| **`courier-deploy`** | Real Unitree G1 + Livox Mid-360 deployment |
| `courier-sim-legacy-mac` | Frozen snapshot of the original mac MuJoCo + Linux Parallels setup (not maintained, kept for historical reference) |

Choose the branch matching your target.

## Running (sim — `courier-sim` branch)

### 1. Install Ubuntu native sim deps

See `docs/sim_setup_ubuntu.md` for the full procedure (one-time).
TL;DR: `pip install mujoco unitree_sdk2py pupil-apriltags pygame opencv-python`,
clone upstream `unitree_mujoco` for G1 mesh STL files, symlink them into
`src/g1_courier_sim/g1_courier_sim/sim_bridge/assets/meshes`.

### 2. Build

```bash
cd ~/g1_courier_ws
colcon build --symlink-install
source install/setup.bash
```

### 3. Run sim

Two terminals:

```bash
# Terminal 1 — MuJoCo bridge (opens viewer)
ros2 launch g1_courier_sim sim_bridge.launch.py

# Terminal 2 — full mission stack (nav2 + dock + arm + mission BT)
ros2 launch g1_courier_bringup phase1_full.launch.py
```

### 4. Optional — diagnostic viewers

```bash
python3 tools/cam_viewer.py     # head_cam preview + AprilTag overlays
python3 tools/lidar_viewer.py   # 2D top-down /scan + RANSAC line fit
python3 tools/plan_viz.py       # nav2 plan + AMCL pose + costmap inflation
```

### 5. Build a custom map (optional, one-off)

```bash
ros2 launch g1_courier_bringup mapping.launch.py
# Drive teleop, e.g. ros2 run teleop_twist_keyboard teleop_twist_keyboard
ros2 run nav2_map_server map_saver_cli -f ~/maps/lab
# Then update phase1_full.launch.py 'map' arg to point at your new map.yaml.
```

### 6. Single-skill debug

```bash
ros2 action send_goal /pick_box g1_courier_msgs/action/PickBox "{}"
ros2 action send_goal /dock_to_table g1_courier_msgs/action/DockToTable "{...}"
```

## Running (real robot — `courier-deploy` branch)

See `docs/deployment_guide.md` on that branch.

## What is implemented

- All action / service / message interfaces (`g1_courier_msgs`).
- Parametric arm controller with CRC, keyframe library (`P0..P6` from real
  G1 calibration), weight ramping, grasp verifier hook, kinematic-mode
  sentinel (`mode==99`) for sim-side joint forcing.
- `PickBox` and `PlaceBox` action servers with grasp_verifier integration.
- `DockToTable` action server with all three modes:
  - `MODE_APRILTAG` — 6-DoF PnP visual servo (parcel tag10)
  - `MODE_LIDAR_LINE` — RANSAC line fit on 2D scan, perpendicular alignment
    (used during transfer when carried parcel occludes head_cam)
  - `MODE_AMCL_ONLY` — trust AMCL (fallback)
- `cmd_vel_arbiter` with priority routing (dock → retreat → nav → /cmd_vel),
  carry-mode velocity caps, freeze service, e-stop latch.
- Behavior Tree mission cycle: `pickup_at_a → transfer_b → pickup_b →
  transfer_a` with `[STAGE START]`/`[STAGE END]` timing logs and visual
  pauses between stages.
- nav2 stack: AMCL OmniMotionModel + NavfnPlanner A\* + RotationShim →
  RegulatedPurePursuit + costmaps with obstacle/inflation layers.
- Linux-native MuJoCo bridge in `g1_courier_sim/sim_bridge/`:
  - `TwoHandGrasp` midpoint kinematic tracking (replaces single-hand weld)
  - kinematic mocap movement (welded pelvis + cmd_vel integration)
  - 360° lidar via `mj_ray()`
  - head_cam render + `pupil_apriltags` detection + image publish
  - `reset_all` payload for clean iteration loops
  - 1 Hz GEOM diagnostic log

## Required external dependencies

apt (ROS2 Jazzy):
```
ros-jazzy-nav2-bringup
ros-jazzy-slam-toolbox
ros-jazzy-pointcloud-to-laserscan
ros-jazzy-py-trees-ros
ros-jazzy-rosidl-generator-dds-idl
ros-jazzy-apriltag-msgs
ros-jazzy-tf2-geometry-msgs
```

pip (sim only — Ubuntu native MuJoCo bridge):
```
mujoco
unitree_sdk2py
pupil-apriltags
pygame
opencv-python
```

ROS2 source (clone into `src/`):
```
unitree_ros2  (provides unitree_hg, unitree_api, unitree_go message types)
              https://github.com/unitreerobotics/unitree_ros2
```
