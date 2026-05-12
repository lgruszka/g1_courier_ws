# `robot_view` — wizualizacja G1 w RViz (quickstart)

Launch `robot_view.launch.py` pokazuje **realny G1 w RViz** bez całego
stacka nawigacji. Użyteczne do:
- sprawdzenia że URDF + joint angles dochodzą z robota
- podglądu LiDAR-a Mid-360 jako 2D skan lub 3D chmura
- obserwacji obrazu z kamery D435i
- debugowania TF tree przed odpaleniem `real.launch.py`

## Wymagania

Firmware Unitree na onboard PC publikuje `/lowstate` (~500 Hz). Bez tego
robot stoi w default pose URDF (statyczny model).

Opcjonalnie:
- `livox_ros_driver2` chodzący → włącz `enable_lidar:=true`
- RealSense D435i podłączona po USB → włącz `enable_camera:=true`
  (wymaga `pyrealsense2` + `opencv-python` w aktywnym Python env)

## Uruchomienie

```bash
cd /home/neo/j2s        # workspace (na onboard PC neo lub dev laptopie)
source install/setup.bash

# sam model robota + RViz (default — bez LiDAR-a, bez kamery):
ros2 launch g1_courier_bringup robot_view.launch.py

# plus LiDAR (Mid-360 → /scan, display LaserScan w RViz):
ros2 launch g1_courier_bringup robot_view.launch.py enable_lidar:=true

# plus kamera D435i (panel Image w RViz):
ros2 launch g1_courier_bringup robot_view.launch.py enable_camera:=true

# wszystko naraz:
ros2 launch g1_courier_bringup robot_view.launch.py \
  enable_lidar:=true enable_camera:=true

# headless (bez RViz — np. żeby z innego terminala odpalić własny rviz2):
ros2 launch g1_courier_bringup robot_view.launch.py enable_rviz:=false
```

## Sukces

W RViz po starcie:
- **Grid** XY (siatka 1×1 m)
- **RobotModel** G1 w pozycji `base_link` (0, 0, 0), Fixed Frame = `base_link`
- **TF** strzałki dla `pelvis`, `torso`, `head`, kończyny
- Model **animuje się** zgodnie z fizyczną pozą robota (przegub stawu na realu → ruch w RViz)
- Jeśli `enable_lidar:=true`: **LaserScan** jako kolorowe punkty wokół robota
- Jeśli `enable_camera:=true`: **D435i Color** panel w RViz pokazuje obraz z kamery

## Argi wszystkie

| arg | default | opis |
|---|---|---|
| `enable_rviz` | `true` | odpal RViz z presetem `robot_view.rviz` |
| `enable_lidar` | `false` | pointcloud_to_laserscan + static TF base_link→lidar |
| `enable_camera` | `false` | d435i_node + Image display |
| `cloud_topic` | `/livox/lidar` | input dla pointcloud_to_laserscan |
| `lidar_frame_id` | `livox_frame` | frame_id na cloud_topic |
| `urdf_path` | `g1_description/urdf/g1_29dof.urdf` | override URDF |
| `rviz_config` | `robot_view.rviz` w bringup/rviz | override RViz preset |

## Joint state publishing — jak działa

```
firmware Unitree
   ↓
/lowstate   (unitree_hg/msg/LowState, ~500 Hz)
   ↓ motor_state[0..28]
   ↓
lowstate_to_joint_states  (g1_courier_safety package)
   ↓ mapuje motor index → joint name z g1_29dof.urdf
   ↓
/joint_states  (sensor_msgs/JointState, ~500 Hz)
   ↓
robot_state_publisher
   ↓ FK z URDF (kinematics tree)
   ↓
/tf  (29 transforms: pelvis → ... → wrist_yaw_link)
   ↓
RViz RobotModel display
```

`lowstate_to_joint_states` ma hardcoded mapowanie motor index 0..28 →
joint name (legs, waist, arms — kolejność z g1_29dof.urdf, sprawdź
`DEFAULT_JOINT_NAMES` w `g1_courier_safety/lowstate_to_joint_states.py`).

Jeśli `/lowstate` nie publikuje:
- `/joint_states` jest puste
- `robot_state_publisher` nie ma update'ów
- TF dla body links nigdy się nie publikuje (poza fixed joints z URDF)
- RViz pokazuje robota w **default pose** (T-pose lub similar, zależy od
  URDF)

## Co najczęściej psuje się

| Objaw | Przyczyna | Fix |
|---|---|---|
| Model w T-pose, brak ruchu | `/lowstate` 0 Hz | sprawdź firmware na onboard, `ros2 topic hz /lowstate` |
| Brak modelu, `/robot_description` puste | URDF nie loaded | sprawdź `ros2 pkg prefix g1_description` |
| LaserScan miga | TF base_link→lidar_frame nie istnieje | enable_lidar:=true ustawia static TF |
| Image display "no data" | d435i_node nie chodzi lub kamera niepodłączona | `ros2 topic hz /camera/image_raw` |
| `import pyrealsense2` fail | brak w Python env | `pip install pyrealsense2 opencv-python` w venv |
| Czarny ekran RViz | Parallels VirGL bug | uruchom z `LIBGL_ALWAYS_SOFTWARE=1` |
| `Could not load resource [package://g1_description/meshes/...]` | RViz w terminalu bez sourcowania | `source install/setup.bash` przed `rviz2` (lub przed launch) |
