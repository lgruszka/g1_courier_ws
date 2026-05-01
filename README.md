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

## Running

### 1. Build
```bash
cd g1_courier_ws
colcon build --symlink-install
source install/setup.bash
```

### 2. Map an environment (one-off)
```bash
ros2 launch g1_courier_bringup mapping.launch.py
# drive the robot manually around both tables
ros2 run nav2_map_server map_saver_cli -f ~/maps/courier_lab
```

### 3. Run the full mission
```bash
ros2 launch g1_courier_bringup courier_full.launch.py \
  map:=$HOME/maps/courier_lab.yaml
```

### 4. Run a single skill (debug)
```bash
ros2 action send_goal /pick_box g1_courier_msgs/action/PickBox "{ ... }"
ros2 action send_goal /dock_to_table g1_courier_msgs/action/DockToTable "{ ... }"
```

## What is implemented vs. TODO

Implemented:
- All action / service / message interfaces.
- Refactored, parametric arm controller with CRC, keyframe library, weight ramping, grasp verifier hook.
- `PickBox` and `PlaceBox` action servers built on top.
- `DockToTable` action server with `MODE_APRILTAG` (6-DoF visual servo) and `MODE_AMCL_ONLY` (trust AMCL).
- `cmd_vel_arbiter` with priority, freeze, carry mode and e-stop.
- Behavior Tree skeleton for the two-table cycle.
- Launch files and config templates for nav2, slam_toolbox, amcl, pointcloud_to_laserscan and AprilTag.

TODO (clearly marked in code):
- `MODE_LIDAR_LINE` aligner (skeleton ready, line-fit logic missing).
- IK-based parametric correction in arm controller (currently linear delta hook).
- Real `box_pose` extraction from AprilTag inside dock action (placeholder uses tag pose verbatim).
- Tuning of all gains, tolerances and timeouts (placeholders flagged with `TODO_TUNE`).
- `unitree_cmd_vel_bridge` is reused from `j2s-light_tracking` and not vendored here. Add a symlink or copy when wiring to the robot.

## Required external dependencies (apt + pip)

```
ros-${ROS_DISTRO}-nav2-bringup
ros-${ROS_DISTRO}-slam-toolbox
ros-${ROS_DISTRO}-pointcloud-to-laserscan
ros-${ROS_DISTRO}-apriltag-ros
ros-${ROS_DISTRO}-tf2-geometry-msgs
ros-${ROS_DISTRO}-py-trees-ros
unitree_api  (Unitree)
unitree_hg   (Unitree)
```
