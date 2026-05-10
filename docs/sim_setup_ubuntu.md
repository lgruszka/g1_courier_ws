# Ubuntu native sim setup (`courier-sim` branch)

This is the **primary** sim path for the team. Runs the entire stack
(MuJoCo physics + bridge + ROS2 nodes + mission BT + tools) on a single
Ubuntu 24.04 machine. No mac, no Parallels, no DDS bridge between hosts.

> Legacy alternative: mac MuJoCo + Linux Parallels VM. Frozen on branch
> `courier-sim-legacy-mac`. Not maintained — only kept for the original
> developer's environment.

## Prerequisites

- Ubuntu 24.04 with ROS2 Jazzy installed
- Python 3.12 (default on Jazzy)
- Working OpenGL (for MuJoCo viewer + camera rendering)

## One-time install

### 1. ROS2 + workspace dependencies

```bash
sudo apt install -y \
  ros-jazzy-nav2-bringup \
  ros-jazzy-slam-toolbox \
  ros-jazzy-pointcloud-to-laserscan \
  ros-jazzy-py-trees-ros \
  ros-jazzy-rosidl-generator-dds-idl \
  ros-jazzy-apriltag-msgs

# Workspace
mkdir -p ~/g1_courier_ws/src && cd ~/g1_courier_ws/src
git clone https://gitlab.com/iAndy77/j2s.git -b courier-sim courier
ln -s courier/src/* .   # if courier repo's src/ holds packages

# Optionally also clone unitree_ros2 for unitree_hg/unitree_api/unitree_go
# message types (G1 firmware IDLs).
git clone https://github.com/unitreerobotics/unitree_ros2 unitree_ros2
touch unitree_ros2/example/COLCON_IGNORE   # skip example pkg
```

### 2. Python deps for the MuJoCo bridge

```bash
pip install mujoco unitree_sdk2py pupil-apriltags pygame opencv-python
```

> If `unitree_sdk2py` isn't on PyPI, install from source:
>
> ```bash
> git clone https://github.com/unitreerobotics/unitree_sdk2_python
> cd unitree_sdk2_python && pip install -e .
> ```

### 3. Provide G1 mesh STL files

The bundled `scene_courier.xml` references G1 mesh files via
`<compiler meshdir="meshes"/>`. We ship **only the AprilTag textures and
custom XML** — the STL meshes come from upstream `unitree_mujoco`.

```bash
# One-time: clone upstream just for meshes
git clone https://github.com/unitreerobotics/unitree_mujoco /tmp/unitree_mujoco

# Symlink meshes into our bundle so MuJoCo finds them via meshdir
ln -s /tmp/unitree_mujoco/unitree_robots/g1/meshes \
      ~/g1_courier_ws/src/g1_courier_sim/g1_courier_sim/sim_bridge/assets/meshes
```

(Alternatively copy the meshes/ folder if you don't want a symlink.)

### 4. Build + source

```bash
cd ~/g1_courier_ws
colcon build --symlink-install
source install/setup.bash
```

## Run sim

Two terminals:

```bash
# Terminal 1 — MuJoCo bridge (opens viewer window)
ros2 launch g1_courier_sim sim_bridge.launch.py

# Terminal 2 — mission stack (nav2 + dock + arm + mission BT)
ros2 launch g1_courier_bringup phase1_full.launch.py
```

Optionally start the diagnostic viewers in a 3rd/4th terminal:

```bash
python3 ~/g1_courier_ws/tools/cam_viewer.py     # head_cam preview + tag bbox
python3 ~/g1_courier_ws/tools/lidar_viewer.py   # /scan top-down + RANSAC line fit
python3 ~/g1_courier_ws/tools/plan_viz.py       # nav2 plan + AMCL pose
```

## How it works

`sim_bridge_node` (Linux process) runs the same code as the mac bridge:

1. Loads `scene_courier.xml` via MuJoCo
2. Opens MuJoCo viewer (passive — controllable with mouse/keys)
3. SimulationThread @ 200 Hz physics:
   - Apply latest `/cmd_vel` to mocap anchor (kinematic mocap movement)
   - Apply sticky `kinematic_mode` arm targets
   - `mj_step` physics
   - `TwoHandGrasp.update_per_tick` (box midpoint tracking)
   - Render head_cam + AprilTag detection + publish `rt/head_cam/image_raw` + `rt/detections`
   - 360° lidar scan via `mj_ray()` + publish `rt/scan`
   - 1 Hz GEOM log (pelvis/box/cam world XYZ for debugging)
4. Publishes/subscribes via `unitree_sdk2py.ChannelPublisher/Subscriber` over CycloneDDS:
   - publishes: `rt/lowstate`, `rt/scan`, `rt/detections`, `rt/head_cam/image_raw`, `rt/camera_info`, `rt/grasp_status`
   - subscribes: `rt/lowcmd`, `rt/cmd_vel`, `rt/parcel_state`

The `/cmd_vel`, `/lowstate` etc. ROS2 topics on Linux side are bridged to
`rt/...` DDS topic names automatically by `rmw_cyclonedds_cpp`.

## Troubleshooting

- **`ModuleNotFoundError: mujoco`** — `pip install mujoco`
- **`ModuleNotFoundError: unitree_sdk2py`** — see Python deps above
- **MuJoCo viewer crashes / black screen** — OpenGL issue. On VMs without
  proper GPU passthrough, try headless mode: `MUJOCO_GL=osmesa ros2 launch ...`
- **Meshes not found / red boxes for robot links** — symlink `meshes/`
  step in install (point 3) was skipped or pointed wrong place.
- **`/scan` empty / no detections** — check `ros2 topic list | grep -E
  "scan|detections"`. If absent, `sim_bridge_node` died at startup; check
  its terminal for traceback (often missing Python dep).
- **Robot falls over instead of standing** — expected; sim uses a welded
  pelvis (`pin_pelvis` in scene XML) plus kinematic mocap movement. Real
  walking controller is **deferred** (see ARCHITECTURE.md).
