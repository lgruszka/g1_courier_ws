# Instalacja `robot_view` na nowym komputerze

Cel: świeży Ubuntu 22.04 → wizualizacja G1 w RViz w ~15 min.

## Wymagania bazowe

- Ubuntu 22.04
- ROS2 Humble zainstalowany ([oficjalna instrukcja](https://docs.ros.org/en/humble/Installation/Ubuntu-Install-Debs.html))
- Połączenie sieciowe z robotem G1 (Ethernet / WiFi w tej samej podsieci)

## Krok 1 — pakiety apt

```bash
sudo apt install -y \
  ros-humble-rviz2 \
  ros-humble-robot-state-publisher \
  ros-humble-tf2-ros \
  ros-humble-xacro \
  ros-humble-pointcloud-to-laserscan \
  ros-humble-rmw-cyclonedds-cpp \
  python3-colcon-common-extensions \
  python3-pip \
  git
```

## Krok 2 — Python deps (tylko dla kamery)

Pomiń jeśli `enable_camera:=false`.

```bash
pip install pyrealsense2 opencv-python numpy
```

Plus jeśli używasz venv (zalecane), aktywuj go przed `pip install`:
```bash
python3 -m venv ~/venvs/g1
source ~/venvs/g1/bin/activate
pip install pyrealsense2 opencv-python numpy
```

`d435i_node.py` wstrzykuje site-packages z `VIRTUAL_ENV` automatycznie,
więc działa zarówno w venv jak i w systemowym Pythonie.

## Krok 3 — clone workspace

```bash
mkdir -p ~/g1_ws/src
cd ~/g1_ws/src
git clone -b courier-deploy https://gitlab.com/iAndy77/j2s.git .
# nasze pakiety są bezpośrednio w src/ (g1_courier_bringup, g1_description, etc.)
```

Plus zależność zewnętrzna (Unitree IDLs dla `/lowstate`):
```bash
cd ~/g1_ws/src
git clone https://github.com/unitreerobotics/unitree_ros2.git
```

## Krok 4 — build

```bash
cd ~/g1_ws
source /opt/ros/humble/setup.bash
colcon build --symlink-install \
  --packages-select \
    unitree_hg unitree_go unitree_api \
    g1_description g1_courier_safety g1_courier_bringup g1_courier_msgs
source install/setup.bash
```

(Buduje tylko minimum potrzebne dla `robot_view`. Cały stack ma więcej
pakietów — pomiń je tutaj, są dla nav2/dock/mission.)

## Krok 5 — DDS network (dev laptop podpięty do robota)

Pomiń jeśli odpalasz na **onboard PC** robota — DDS jest lokalne.

Jeśli **dev laptop** łączy się z robotem przez LAN:

```bash
# znajdź swój interfejs sieciowy (eth0 / wlan0 / enX...):
ip addr show
# powiedzmy że to "eth0"

cat > ~/cyclonedds.xml <<'EOF'
<?xml version="1.0" encoding="UTF-8" ?>
<CycloneDDS xmlns="https://cdds.io/config">
    <Domain id="any">
        <General>
            <Interfaces>
                <NetworkInterface autodetermine="false" name="eth0" priority="default" multicast="default" />
            </Interfaces>
        </General>
    </Domain>
</CycloneDDS>
EOF

# dodaj do .bashrc:
echo 'export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp' >> ~/.bashrc
echo 'export CYCLONEDDS_URI=file://'"$HOME"'/cyclonedds.xml' >> ~/.bashrc
echo 'export ROS_DOMAIN_ID=0' >> ~/.bashrc        # taki sam jak na onboard
source ~/.bashrc
```

Podstaw `eth0` swoim faktycznym interfejsem. `ROS_DOMAIN_ID` musi być
ten sam co na onboard PC robota (zwykle `0`, ale sprawdź `echo $ROS_DOMAIN_ID`
na onboard).

## Krok 6 — sanity test (czy /lowstate dochodzi)

```bash
source ~/g1_ws/install/setup.bash
ros2 topic hz /lowstate
# spodziewane: ~500 Hz
# jeśli 0 — DDS nie dochodzi, sprawdź interfejs sieciowy + ROS_DOMAIN_ID
```

## Krok 7 — uruchomienie

```bash
# sam model robota (default):
ros2 launch g1_courier_bringup robot_view.launch.py

# plus LiDAR (Mid-360):
ros2 launch g1_courier_bringup robot_view.launch.py enable_lidar:=true

# plus kamera D435i (uwaga — musi być pyrealsense2 + opencv-python):
ros2 launch g1_courier_bringup robot_view.launch.py enable_camera:=true

# wszystko:
ros2 launch g1_courier_bringup robot_view.launch.py \
  enable_lidar:=true enable_camera:=true
```

Sukces: okno RViz z animowanym G1 (ruchy robota → RViz). Jeśli model
stoi w T-pose — `/lowstate` nie dochodzi, wróć do Kroku 5/6.

## Co nie działa — diagnoza

| Objaw | Sprawdź | Fix |
|---|---|---|
| `colcon build` failuje na `unitree_hg` | `unitree_ros2/` brak w `src/` | Krok 3 — clone unitree_ros2 |
| `Package 'g1_description' not found` po launch | brak `source install/setup.bash` | `source ~/g1_ws/install/setup.bash` (lub do `~/.bashrc`) |
| `/lowstate` 0 Hz | DDS bridge | Krok 5 — CycloneDDS interface plus ROS_DOMAIN_ID |
| Model w T-pose | `/lowstate` Hz 0 lub `/joint_states` 0 | `ros2 topic hz /joint_states` — jeśli 0, `lowstate_to_joint_states` nie chodzi |
| Czarny ekran RViz | hardware OpenGL bug | `LIBGL_ALWAYS_SOFTWARE=1 ros2 launch ...` |
| `import pyrealsense2` fail | brak w env | Krok 2 + uruchom z aktywowanym venv |
| `Could not load mesh package://g1_description/...` | source przed RViz | upewnij się że `source install/setup.bash` zrobione **w tym samym terminalu** co RViz |
