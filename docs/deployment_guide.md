# Deployment guide — realny robot (`courier-deploy` branch)

End-to-end procedura podniesienia misji courier na fizycznym Unitree G1
z Livox Mid-360 i RealSense D435i.

> Użytkownicy sim: ten guide nie ma zastosowania. Przełącz się na branch
> `courier-sim`.

## Kolejność czytania (pierwszy raz)

Jeśli jeszcze nie deployowałeś tego stacka na realnym robocie, idź
sekcjami w tej kolejności — każda buduje na poprzedniej:

1. [Hardware checklist](#hardware-checklist) — zweryfikuj każdy ptaszek
2. [Instalacja oprogramowania (jednorazowo)](#instalacja-oprogramowania-jednorazowo) — pakiety, repo, sterowniki
3. [Day 1 walkthrough](#day-1-walkthrough) — pierwsze 60 minut z robotem
4. [Budowa mapy (raz na lab)](#budowa-mapy-raz-na-lab) — slam_toolbox + slice Livox
5. [Kalibracja waypointów z mapy](#kalibracja-waypointów-z-mapy) — clicks w RViz
6. [Pierwsze uruchomienie misji](#pierwsze-uruchomienie-misji) — pełen cykl A↔B
7. [Troubleshooting](#troubleshooting) — gdy coś nie działa

Jeśli robiłeś już install + map + kalibrację na poprzedniej wizycie,
przejdź od razu do [Pierwsze uruchomienie misji](#pierwsze-uruchomienie-misji).

## Hardware checklist

- [ ] Unitree G1 (29-DoF) z włączonym `arm_sdk` i gotowym sport API
- [ ] Livox Mid-360 zamontowany na głowie, USB do onboard PC
- [ ] RealSense D435i w torsie, USB3
- [ ] Dwa biurka (table A, table B) — **bez AprilTagów na biurkach**.
  Lokalizacja biurek tylko z mapy + LIDAR_LINE (RANSAC na krawędzi).
- [ ] Tekturowy karton z AprilTagiem `tag36h11` `id=10` na ścianie
  kartonu (rozmiar 0.10 m) — JEDYNY tag używany w systemie. Robot
  używa go tylko do precyzyjnego docka pod chwyt (`pick_box`).
- [ ] WiFi lub ethernet między onboard PC a dev laptopem (do RViz / debug)

## Instalacja oprogramowania (jednorazowo)

### 1. ROS2 Humble + pakiety systemowe

```bash
sudo apt install -y \
  ros-humble-nav2-bringup \
  ros-humble-slam-toolbox \
  ros-humble-pointcloud-to-laserscan \
  ros-humble-py-trees-ros \
  ros-humble-rosidl-generator-dds-idl \
  ros-humble-apriltag-ros \
  ros-humble-apriltag-msgs \
  ros-humble-tf2-geometry-msgs \
  ros-humble-realsense2-camera \
  ros-humble-teleop-twist-keyboard
```

### 2. Workspace + źródła

```bash
mkdir -p ~/g1_courier_ws/src && cd ~/g1_courier_ws/src
git clone https://gitlab.com/iAndy77/j2s.git -b courier-deploy courier
ln -s courier/src/* .

# IDLs Unitree
git clone https://github.com/unitreerobotics/unitree_ros2
touch unitree_ros2/example/COLCON_IGNORE

# URDF G1 + meshes (master branch — NIE unitree_ros2)
git clone https://github.com/unitreerobotics/unitree_ros
# Użyj unitree_ros/robots/g1_description dla robot_state_publisher.

# Sterownik Livox
git clone https://github.com/Livox-SDK/livox_ros_driver2
```

### 3. Build

```bash
cd ~/g1_courier_ws
colcon build --symlink-install
source install/setup.bash
```

### 4. Bridge'e firmware'owe

Firmware Unitree G1 udostępnia te topiki DDS przez onboard SDK:

- `/lowstate` — pozycje stawów, IMU, foot contacts (publish)
- `/lowcmd` — targety stawów z CRC (subscribe; konsumowane przez `arm_sdk`)
- `/cmd_vel` — komenda locomotion (subscribe; konsumowana przez sport API)

Upewnij się że firmware-side bridge chodzi przed odpaleniem tego stacka.
Sprawdzenie:

```bash
ros2 topic hz /lowstate           # spodziewane ~500 Hz
ros2 topic list | grep livox      # /livox/lidar
ros2 topic list | grep camera     # /camera/color/image_raw
```

## Day 1 walkthrough

Konkretna sekwencja na pierwszą sesję w labie. Zakłada że
[hardware](#hardware-checklist) i [install](#instalacja-oprogramowania-jednorazowo)
są zrobione.

### Minuta 0–10: podniesienie platformy

1. Włącz G1, niech wstanie. Zweryfikuj że jest w trybie damping (zerowy
   moment, stawy luźne).
2. Wepnij USB Livox + RealSense do onboard PC.
3. Odpal firmware bridge (proprietary, sport API). Potwierdź:
   ```bash
   ros2 topic hz /lowstate          # ~500 Hz
   ros2 topic list | grep -E 'livox|camera'
   # /livox/lidar  /camera/color/image_raw  /camera/color/camera_info
   ```
4. Z dev laptopa w tej samej sieci:
   ```bash
   export ROS_DOMAIN_ID=<ten sam co robot>
   ros2 topic list   # powinieneś widzieć wszystkie powyższe
   ```

### Minuta 10–25: smoke-test komponentów

Nie odpalaj jeszcze misji — zweryfikuj każdy element osobno. Otwórz 4
terminale na dev laptopie. W każdym sourcuj workspace:
```bash
source ~/g1_courier_ws/install/setup.bash
```

**Terminal 1** — sensory wizualizowane:
```bash
python3 ~/g1_courier_ws/tools/cam_viewer.py
# Sprawdź: obraz przychodzi, ekspozycja OK, widać lab przed robotem.
```

**Terminal 2** — slice LiDAR:
```bash
ros2 launch g1_courier_bringup mapping_real.launch.py
# W innym terminalu:
python3 ~/g1_courier_ws/tools/lidar_viewer.py
# Sprawdź: scan 360° widoczny, ściany/biurka czytelne, brak luk.
```

Jeśli slice lidaru jest pusty albo rzadki, twoje `min_height/max_height`
w `bringup/config/pointcloud_to_laserscan.yaml` nie pasuje do wysokości
montażu Livoxa. Domyślne wartości zakładają Livox na ~1.45 m AGL na
głowie G1.

**Terminal 3** — test ręcznej jazdy:
```bash
ros2 run teleop_twist_keyboard teleop_twist_keyboard
```
Używaj rozważnie — naciskaj w/x z małymi prędkościami (`0.1 m/s`).
Potwierdź że robot przemieszcza się zgodnie z komendą. **Wyłącz
teleop zanim przejdziesz dalej.**

**Terminal 4** — gdy wszystko wygląda OK, zabij `mapping_real.launch.py`
z terminala 2 przez `Ctrl+C`.

### Minuta 25–45: budowa mapy

```bash
ros2 launch g1_courier_bringup mapping_real.launch.py
# W osobnym terminalu:
ros2 run teleop_twist_keyboard teleop_twist_keyboard
ros2 run rviz2 rviz2
# Add → Map → topic /map. Obserwuj jak mapa się wypełnia w trakcie jazdy.
```

Przejedź robotem przez całą topologię laba: front każdego biurka,
każdy korytarz, oba kierunki. Wolna stała prędkość (~0.2 m/s) daje
najczystszą mapę.

Gdy mapa wygląda kompletnie:
```bash
ros2 run nav2_map_server map_saver_cli -f ~/maps/lab
```

Masz teraz `~/maps/lab.pgm` + `~/maps/lab.yaml`.

### Minuta 45–60: kalibracja waypointów

Patrz [Kalibracja waypointów z mapy](#kalibracja-waypointów-z-mapy)
poniżej — clicks w RViz, edycja `waypoints.yaml`, ~10 minut.

Po tym jesteś gotów na
[Pierwsze uruchomienie misji](#pierwsze-uruchomienie-misji).

## Budowa mapy (raz na lab)

```bash
ros2 launch g1_courier_bringup mapping_real.launch.py
# W osobnym terminalu:
ros2 run teleop_twist_keyboard teleop_twist_keyboard
# Przejedź robotem przez cały lab pokrywając oba biurka i obszar tranzytowy.
# Gdy mapa wygląda kompletnie w RViz:
ros2 run nav2_map_server map_saver_cli -f ~/maps/lab
```

Masz teraz `~/maps/lab.pgm` + `~/maps/lab.yaml`.

## Kalibracja waypointów z mapy

`src/g1_courier_mission/config/waypoints.yaml` definiuje predock pose
per-biurko w **world (map) frame**. Po zapisaniu nowej mapy musisz
zaktualizować X/Y/yaw każdego predocka tak, żeby:

- nav2 sprowadzał robota ~50 cm przed krawędzią biurka
- karton z tagiem id=10 leżący na biurku był w polu widzenia `head_cam`
  z tej pozycji (do późniejszego docka APRILTAG)

### Procedura

1. **Otwórz mapę w RViz**:
   ```bash
   ros2 run nav2_map_server map_server --ros-args \
     -p yaml_filename:=$HOME/maps/lab.yaml
   ros2 run rviz2 rviz2
   # Add → Map → topic /map
   ```

2. **Znajdź krawędź każdego biurka w world frame**. Kliknij narzędzie
   "Publish Point" w RViz i kliknij na przedniej krawędzi biurka A.
   RViz loguje `(x, y)` na `/clicked_point`. Zapisz wartości.

3. **Predock pose**:
   - `predock_x = table_edge_x - 0.50` (50 cm przed, żeby dock servo
     miał miejsce na ~13 cm advance bez kolizji)
   - `predock_y = table_edge_y` (centered)
   - `predock_yaw` = orientacja kierująca robota **w stronę** biurka.
     Dla biurka po stronie +X: `0.0`. Dla −X: `3.14159`. Dla −Y:
     `-1.5708`. Itd.

4. **Edytuj waypoints.yaml** nowymi wartościami:
   ```yaml
   tables:
     table_a:
       predock_x: 1.42      # zmierzone: krawędź biurka 1.92 m, robot 0.50 m za
       predock_y: 0.00
       predock_yaw: 0.0
       dock_mode: apriltag
       final_xy_tol_m: 0.03
       final_yaw_tol_rad: 0.05
     table_b:
       predock_x: 4.10
       ...
   ```

5. **Zweryfikuj** ręcznie nawigując do predocka i sprawdzając widok
   `head_cam`:
   ```bash
   ros2 launch g1_courier_bringup real.launch.py map:=$HOME/maps/lab.yaml
   # W RViz, "2D Goal Pose" → kliknij w pozycję predock_a.
   # Po zakończeniu nav, postaw na biurku karton z tagiem id=10.
   # W innym terminalu:
   ros2 topic echo /detections --once
   # Spodziewany tag id=10 z niezerowymi corners i centre blisko (640, 480)/2.
   ```

6. Powtórz dla biurka B.

## Pierwsze uruchomienie misji

```bash
ros2 launch g1_courier_bringup real.launch.py map:=$HOME/maps/lab.yaml
```

Spodziewana sekwencja:

1. AMCL zbiega na zapisaną mapę (ustaw initial pose w RViz jeśli trzeba)
2. Mission BT nawiguje do predocka_a
3. Dock APRILTAG do box tag10 (17 cm) — biurko bez własnego taga
4. `pick_box` wykonuje sekwencję P0..P6 → `grasp_verified=true`
5. Włącza się carry mode (niższe limity prędkości z `safety.yaml`)
6. Nawigacja do predocka_b
7. Dock LIDAR_LINE (kamera zasłonięta przez karton)
8. `place_box` wykonuje P5..ZERO → `release_verified=true`
9. Retreat 0.5 m, swap A/B, kolejny cykl

## Troubleshooting

Każdy wpis: **objaw** → polecenie diagnostyczne → przyczyna → fix.

### `colcon build` failuje na `unitree_hg` not found
```bash
ls src/unitree_ros2/unitree_hg     # powinno istnieć
```
- **Przyczyna**: `unitree_ros2` nie sklonowany, albo brak
  `example/COLCON_IGNORE` powodujący że pakiet example failuje.
- **Fix**: clone wg § "Workspace + źródła";
  `touch src/unitree_ros2/example/COLCON_IGNORE`.

### `ros2 topic list` nie pokazuje `/lowstate` lub `/livox/lidar`
```bash
echo $ROS_DOMAIN_ID                # musi być taki sam jak na robocie
ros2 daemon stop && ros2 daemon start
ros2 topic list
```
- **Przyczyna**: mismatch domain ID, stale daemon, albo firmware bridge
  nie działa.
- **Fix**: ujednolić `ROS_DOMAIN_ID` między robotem a dev laptopem,
  zrestartować daemon.

### AMCL nie zbiega (ikona robota stoi w origin lub dryfuje)
```bash
ros2 topic echo /amcl_pose --once   # sprawdź covariance i pose
ros2 topic hz /scan                 # spodziewane ~10 Hz
ros2 run tf2_tools view_frames      # potwierdź map → odom → base_link
```
- **Przyczyna 1**: brak initial pose.
  **Fix**: w RViz kliknij "2D Pose Estimate", przeciągnij strzałkę
  na mapie w rzeczywistej lokalizacji robota.
- **Przyczyna 2**: scan pusty albo bardzo rzadki.
  **Fix**: sprawdź `pointcloud_to_laserscan.yaml` slice heights pasują
  do wysokości montażu Livoxa; sprawdź że `/livox/lidar` publikuje.
- **Przyczyna 3**: słabe pokrycie mapy.
  **Fix**: nagraj mapę ponownie, dokładniej przejdź przez obszar
  tranzytowy.

### Dock APRILTAG nie zbiega
```bash
ros2 topic echo /detections --once   # tag id 5 albo 7 widoczny?
python3 tools/cam_viewer.py          # wizualne sprawdzenie bbox
```
- **Przyczyna 1**: tag poza FoV kamery z aktualnej pozy predock.
  **Fix**: edytuj `predock_x/y/yaw` w `waypoints.yaml` żeby tag był
  wycentrowany. Użyj cam_viewer do weryfikacji.
- **Przyczyna 2**: obraz D435i nieskorygowany albo `camera_info` ma złe
  intrinsics.
  **Fix**: upewnij się że `realsense2_camera` publikuje `image_rect`;
  zweryfikuj że `camera_info` pasuje do factory values D435i
  (`fx ≈ 615`, `fy ≈ 615`, `cx ≈ 320`, `cy ≈ 240` dla 640×480).
- **Przyczyna 3**: oświetlenie albo motion blur.
  **Fix**: zwiększ ekspozycję, zwolnij dock approach, podnieś rozmiar
  taga.

### `grasp_verified=false` na każdym pick
```bash
ros2 topic echo /lowstate --field motor_state[<arm_idx>].tau_est
```
- **Przyczyna**: `grasp_tau_threshold_nm` w `arm_skills.yaml` nie pasuje
  do realnej masy kartonu + pozy ramion.
- **Fix**: zarejestruj baseline τ przed pickem, zarejestruj τ po lift,
  ustaw threshold na 60% różnicy. Domyślne 1.5 Nm jest dla kartonu
  ~1 kg; skaluj liniowo.

### Dock LIDAR_LINE dryfuje / nigdy się nie ustabilizuje
```bash
python3 tools/lidar_viewer.py
# Doprowadź ręcznie do predocka, obserwuj zieloną linię RANSAC.
```
- **Przyczyna 1**: forward window za szeroki, aligner łapie przeszkody
  obok biurka.
  **Fix**: zwęź `lidar_line.window_*` w `docking.yaml`.
- **Przyczyna 2**: inliers RANSAC za luźne, fituje szum.
  **Fix**: zaostrz `lidar_line.inlier_threshold_m`.
- **Przyczyna 3**: zły znak korekcji yaw (był znany bug — patrz
  CLAUDE.md "Dock LIDAR_LINE timeout").
  **Fix**: zweryfikuj `cmd.angular.z = +kp * yaw_err` w
  `LidarLineAligner`.

### Robot wibruje w carry mode (oscyluje podczas chodzenia)
```bash
ros2 topic echo /cmd_vel --once     # zobacz co arbiter publikuje
```
- **Przyczyna**: limity prędkości carry-mode za wysokie dla aktualnej
  masy kartonu.
- **Fix**: zaostrz `max_vx_carry`, `max_vy_carry`, `max_vyaw_carry` w
  `safety/config/safety.yaml`. Zacznij od `0.2 / 0.1 / 0.3`.

### Mission BT pętli się na tej samej fazie
```bash
ros2 topic echo /mission_status --once
ros2 node info /mission_node
```
- **Przyczyna**: faza ciągle timeoutuje (np. dock A timeout na powrocie
  bo head_cam zasłonięty przez niesiony karton).
- **Fix**: upewnij się że `dock_mode: lidar_line` jest ustawiony na
  biurku **powrotnym** w `waypoints.yaml`. Oba `table_a` i `table_b`
  powinny być `lidar_line` żeby były carry-independent.

### `nav2 controller "Passing new path" ale robot stoi`
```bash
ros2 topic hz /cmd_vel              # 0 Hz oznacza że nic nie dociera do firmware
ros2 topic hz /cmd_vel_nav          # 10 Hz oznacza że nav2 PUBLIKUJE
```
- **Przyczyna**: humble `nav2_bringup` remappuje `cmd_vel` → `cmd_vel_nav`.
  Bez `cmd_vel_arbiter` w trybie merge, output nav2 idzie w nicość.
- **Fix**: potwierdź że `cmd_vel_arbiter` chodzi i subskrybuje
  `/cmd_vel_nav`. `real.launch.py` to robi automatycznie.

### Stare fail-recipes warte pamiętania
Przeczytaj `CLAUDE.md` § "Najczęstsze problemy które mogą wystąpić" —
pełen katalog bugów napotkanych podczas developmentu sim. Większość
sim-specific (mac bridge, MuJoCo zombies), ale kilka ma zastosowanie
też do realnego robota: NavfnPlanner tolerance, sign `dyaw` APRILTAG,
sign yaw LiDAR_LINE.

## Action contracts cheat sheet

Gdy odpalasz akcje ręcznie do debugu, używaj tych payloadów jako
szablonów. Pełen IDL: `src/g1_courier_msgs/{action,srv}/`.

```bash
# /pick_box  — nominalne keyframes P0..P6:
ros2 action send_goal /pick_box g1_courier_msgs/action/PickBox \
  '{box_pose: {header: {frame_id: ""}}, sequence_name: "pick_box", timeout_s: 30.0}' \
  --feedback

# /place_box  — nominalne P5..ZERO:
ros2 action send_goal /place_box g1_courier_msgs/action/PlaceBox \
  '{target_pose: {header: {frame_id: ""}}, sequence_name: "place_box", timeout_s: 30.0}' \
  --feedback

# /dock_to_table APRILTAG (tag10 na kartonie — jedyny tag w systemie):
ros2 action send_goal /dock_to_table g1_courier_msgs/action/DockToTable \
  '{mode: 0, apriltag_id: 10,
    target_pose: {header: {frame_id: "tag_box"}},
    xy_tolerance_m: 0.10, yaw_tolerance_rad: 0.15, timeout_s: 25.0}' \
  --feedback

# /dock_to_table LIDAR_LINE (predock world (4.10, 0, 0)):
ros2 action send_goal /dock_to_table g1_courier_msgs/action/DockToTable \
  '{mode: 1, apriltag_id: 0,
    target_pose: {header: {frame_id: "map"},
                  pose: {position: {x: 4.10, y: 0.0, z: 0.0}}},
    xy_tolerance_m: 0.03, yaw_tolerance_rad: 0.05, timeout_s: 25.0}' \
  --feedback

# /courier/navigate_to_pose (predock_a w (1.42, 0, 0) facing +X):
ros2 action send_goal /courier/navigate_to_pose \
  g1_courier_msgs/action/NavigateToPose \
  '{target_pose: {header: {frame_id: "map"},
                  pose: {position: {x: 1.42, y: 0.0, z: 0.0},
                         orientation: {w: 1.0}}},
    waypoint_name: "predock_a",
    xy_tolerance_m: 0.0, yaw_tolerance_rad: 0.0, timeout_s: 60.0}' \
  --feedback

# /retreat — open-loop wycofanie 0.5 m z prędkością 0.15 m/s:
ros2 action send_goal /retreat g1_courier_msgs/action/Retreat \
  '{distance_m: 0.5, speed_mps: 0.15, timeout_s: 10.0}' \
  --feedback

# /safety/set_carry_mode — włącz limity carry-mode:
ros2 service call /safety/set_carry_mode g1_courier_msgs/srv/SetCarryMode \
  '{carrying: true}'

# /safety/set_freeze — emergency freeze (zerowe cmd_vel):
ros2 service call /safety/set_freeze g1_courier_msgs/srv/SetFreeze \
  '{freeze: true}'
```

Żeby **anulować** akcję w trakcie: `Ctrl+C` w terminalu `send_goal`.
Action server respektuje cancel i czysto się zatrzymuje.

Żeby **wylistować** wszystkie dostępne akcje:
```bash
ros2 action list
ros2 action info /pick_box        # pokazuje server + client(s)
```

## Configi które prawdopodobnie zechcesz tunować

| Plik | Co tunować | Kiedy |
|---|---|---|
| `mission/config/waypoints.yaml` | predock per-biurko | Każda nowa mapa |
| `arm_skills/config/arm_skills.yaml` | `grasp_tau_threshold_nm` | Per masa kartonu |
| `docking/config/docking.yaml` | dock kp_xy/kp_yaw, target_distance | Pierwszy deploy + per oświetlenie |
| `safety/config/safety.yaml` | limity carry-mode v | Per masa kartonu + balans |
| `bringup/config/nav2_params.yaml` | inflation costmapy, footprint | Per zatłoczenie laba |

## Narzędzia diagnostyczne

W osobnym terminalu obok `real.launch.py`:

```bash
python3 ~/g1_courier_ws/tools/cam_viewer.py     # bbox tag + dystans PnP
python3 ~/g1_courier_ws/tools/lidar_viewer.py   # /scan top-down + RANSAC
python3 ~/g1_courier_ws/tools/plan_viz.py       # nav2 plan + AMCL pose
```

## Sim parity

Te same code paths chodzą w sim (branch `courier-sim`). Różnice:

- Sim odpala MuJoCo przez `g1_courier_sim/sim_bridge/`, real używa
  firmware bridge'y Unitree.
- Sim ustawia `kinematic_mode: true` na `pick`/`place` żeby obejść
  jitter PD-via-DDS; real zostaje `false`.
- Sim publikuje `/parcel_state` żeby uwolnić weld constraint przy
  placu; real nie ma takiego topiku (palce fizyczne ogarniają release).
- Sim używa `pupil_apriltags` w procesie bridge'a; real używa
  `apriltag_ros` jako osobnego node'a konsumującego klatki D435i.

Mission BT, action server docka, arm controller, i nav2 stack są
**identyczne** między branchami.
