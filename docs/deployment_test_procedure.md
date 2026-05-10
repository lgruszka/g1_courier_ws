# Procedura testowa — pełne uruchomienie krok po kroku

Tryb testowy: każdy krok ma **checkpoint** (komenda diagnostyczna +
spodziewany output). Przechodzisz dalej **tylko gdy checkpoint pasuje**.
Gdy nie pasuje — patrz "Co jeśli nie działa" pod każdym krokiem.

Cel: dojście od zimnego startu do pełnego cyklu mission A↔B z
weryfikacją każdej warstwy po drodze.

> Ta procedura zakłada że masz **realnego G1 z Livox Mid-360 +
> RealSense D435i + Unitree firmware** i jesteś na branchu
> `courier-deploy`. Dla sim zobacz README brancha `courier-sim`.

## Założenia (przed startem)

- [ ] Hardware checklist z `deployment_guide.md` § "Hardware checklist"
- [ ] Software install zrobiony (apt + workspace + sources)
- [ ] Workspace zbudowany: `colcon build --symlink-install`
- [ ] Mapa nie jest jeszcze zbudowana (Część 3 ją zbuduje) ALBO masz
  poprzednią `~/maps/lab.yaml` z poprzedniej wizyty

## Część 1 — Bring-up sprzętowy (5 min)

### 1.1 Włącz robota

- [ ] Stojący G1, zasilany, w trybie damping (zerowy moment, stawy luźne)
- [ ] Onboard PC odpalony, dostępny przez SSH lub bezpośrednio

### 1.2 Sprawdź sieć między onboard PC a dev laptopem

```bash
# z dev laptopa:
ping <onboard_pc_ip>
```

**Checkpoint**: ping odpowiada. Jeśli nie — sprawdź WiFi/ethernet, IP.

### 1.3 Ustaw ROS_DOMAIN_ID po obu stronach

```bash
# na onboard PC i dev laptopie:
export ROS_DOMAIN_ID=42        # lub inny, byle taki sam
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
```

Dodaj do `~/.bashrc` po obu stronach żeby nie ustawiać przy każdym terminalu.

### 1.4 Odpal firmware Unitree (na onboard PC)

Zgodnie z dokumentacją Unitree dla twojej wersji firmware. Spodziewane:
firmware publikuje `/lowstate`, subskrybuje `/lowcmd` i `/cmd_vel`.

**Checkpoint** (z dev laptopa):
```bash
ros2 topic hz /lowstate
# spodziewane: ~500 Hz
```

**Co jeśli nie działa**:
- 0 Hz → firmware bridge nie chodzi, sprawdź log na onboard PC
- mismatch domain ID → `echo $ROS_DOMAIN_ID` po obu stronach
- daemon stale → `ros2 daemon stop && ros2 daemon start`

### 1.5 Odpal Livox driver (na onboard PC)

```bash
ros2 launch livox_ros_driver2 msg_MID360_launch.py
```

**Checkpoint**:
```bash
ros2 topic hz /livox/lidar
# spodziewane: ~10 Hz
```

**Co jeśli nie działa**:
- 0 Hz → kabel USB Livoxa odpięty, driver nie chodzi
- driver crash → sprawdź IP Livoxa w configu drivera (zwykle 192.168.1.x)

### 1.6 Odpal RealSense driver (na onboard PC)

```bash
ros2 launch realsense2_camera rs_launch.py
```

**Checkpoint**:
```bash
ros2 topic hz /camera/color/image_raw
ros2 topic hz /camera/color/camera_info
# spodziewane: ~30 Hz na image, ~30 Hz na camera_info
```

**Co jeśli nie działa**:
- 0 Hz → USB3 niewłaściwie podłączony albo D435i nie wykryta
  (`lsusb | grep RealSense`)
- camera_info brak → włącz `enable_color:=true` w driverze

## Część 2 — Health check stacka (5 min)

### 2.1 Lista topików

```bash
ros2 topic list | sort
```

**Checkpoint**: widzisz na liście:
- `/lowstate` `/lowcmd`
- `/livox/lidar`
- `/camera/color/image_raw` `/camera/color/camera_info`
- `/cmd_vel` `/odom`
- `/tf` `/tf_static`

### 2.2 TF graph

```bash
ros2 run tf2_tools view_frames
# tworzy frames.pdf w cwd, otwórz np. evince frames.pdf
```

**Checkpoint**: w grafie widać:
- `odom → base_link` (publikuje firmware Unitree)
- `base_link → livox_frame` (publikuje URDF + robot_state_publisher
  ALBO static_transform_publisher)
- `base_link → camera_*` (URDF lub RealSense driver)

**Co jeśli brak**:
- Nie ma `odom → base_link` → firmware bridge nie publikuje TF
- Nie ma `base_link → livox_frame` → trzeba odpalić `robot_state_publisher`
  z URDF G1 LUB ręczny `static_transform_publisher`:
  ```bash
  ros2 run tf2_ros static_transform_publisher \
    --x 0 --y 0 --z 1.45 --frame-id base_link --child-frame-id livox_frame
  ```
  (1.45 m to przykładowa wysokość Livoxa na głowie G1; zmierz fizycznie)

### 2.3 Test motorów (sanity)

```bash
ros2 topic echo /lowstate --field motor_state[12].q --once
# spodziewane: float bliski 0.0 dla zerowej pozy
```

## Część 3 — Mapowanie laboratorium (15-30 min, raz na lab)

Pomijasz tę część jeśli `~/maps/lab.yaml` już istnieje z poprzedniej
wizyty.

### 3.1 Przygotuj katalog

```bash
mkdir -p ~/maps
```

### 3.2 Odpal mapping launch

```bash
# Terminal 1 (dev laptop):
ros2 launch g1_courier_bringup mapping_real.launch.py
```

**Domyślne wartości** (jeśli twój robot ma inny setup, override przez
launch args):
- `cloud_topic:=/utlidar/cloud_livox_360mid` — topic Unitree firmware
- `lidar_frame_id:=utlidar_lidar` — frame_id chmury (sprawdź:
  `ros2 topic echo <cloud_topic> --field header.frame_id --once`)
- `urdf_path:=` — domyślnie `g1_description` package (vendored URDF G1)
- `enable_robot_model:=true|false` — gate dla RSP + adapter
  (set `false` jeśli URDF brak lub robi się minimalny test)
- statyczne TF `base_link → lidar` z xyz=(0, 0, 1.45) m — zmierz
  fizycznie wysokość Livoxa, jeśli inna, edytuj launch lub dodaj
  override (na razie hardkoded — można wynieść do launch arg później)

Przykład z customowymi:
```bash
ros2 launch g1_courier_bringup mapping_real.launch.py \
    cloud_topic:=/livox/lidar \
    lidar_frame_id:=livox_frame
```

**Checkpoint**: w logu widzisz `slam_toolbox` przechodzący do `active`,
brak traceback'ów.

### 3.3 Sprawdź że scan i map publikują

```bash
# Terminal 2:
ros2 topic hz /scan
# spodziewane: ~10 Hz

ros2 topic hz /map
# spodziewane: 0 Hz początkowo, potem ~1 Hz po pierwszych ruchach robota
```

**Co jeśli `/scan` 0 Hz**:
- `/livox/lidar` też 0 Hz → patrz Krok 1.5
- `/livox/lidar` OK, ale `/scan` 0 Hz → brak TF od `base_link` do source
  frame chmury → patrz Krok 2.2

### 3.4 Teleop — jeźdź ostrożnie po lab

```bash
# Terminal 3:
ros2 run teleop_twist_keyboard teleop_twist_keyboard
```

Klawisze: `w/x` przód/tył, `a/d` skręt, `s` stop. Małe prędkości
(`q` zmniejsza, `e` zwiększa). Zacznij od `0.1 m/s`.

**Co robić**: przejedź przez cały lab pokrywając:
- front każdego biurka (z kilku stron jeśli można)
- transit area (gdzie robot będzie chodził między biurkami)
- każdy korytarz, oba kierunki

Wolna stała prędkość (~0.2 m/s) daje najczystszą mapę.

### 3.5 Podgląd mapy w trakcie budowy

```bash
# Terminal 4:
ros2 run rviz2 rviz2
```

W RViz: **Add → By topic → /map → Map**, Fixed Frame: `map`. Plus
**Add → /tf** żeby widzieć pose robota.

**Checkpoint**: w RViz widzisz mapę rosnącą. Ściany ostro czarne.
Robot ikona porusza się synchronizowanie z fizycznym ruchem.

### 3.6 Zapisz mapę (przed Ctrl+C launchu)

```bash
# Terminal 5 (mapping_real.launch.py wciąż chodzi):
ros2 run nav2_map_server map_saver_cli -f ~/maps/lab
```

**Checkpoint**: log kończy się `Map saved successfully`. Sprawdź:
```bash
ls -la ~/maps/
# spodziewane: lab.pgm + lab.yaml
```

**Co jeśli `Failed to save map`**:
- `/map` nie publikuje (Krok 3.3 fail) → robot się nie poruszał (slam_toolbox
  wymaga ≥20 cm ruchu albo ≥11° rotacji)
- katalog `~/maps/` nie istnieje → `mkdir -p ~/maps`

### 3.7 Zatrzymaj mapping launch

Ctrl+C w Terminal 1 (mapping_real), Terminal 3 (teleop), Terminal 4 (rviz).

### 3.8 Obejrzyj mapę

```bash
eog ~/maps/lab.pgm
# lub:
feh ~/maps/lab.pgm
```

Plus z metadanymi w RViz:
```bash
# Terminal 1:
ros2 run nav2_map_server map_server --ros-args \
  -p yaml_filename:=$HOME/maps/lab.yaml -p use_sim_time:=false
# Terminal 2:
ros2 run nav2_lifecycle_manager lifecycle_manager --ros-args \
  -p autostart:=true -p node_names:=['map_server']
# Terminal 3:
ros2 run rviz2 rviz2
# RViz: Add → Map → /map. Fixed Frame: map.
```

**Checkpoint**: w mapie widać:
- oba biurka jako wyraźne czarne kontury (~0.4 × 0.6 m widok z góry)
- ściany ciągłe, bez przerw
- mało "krzaczków" floating w transit area (jeśli dużo szumu — patrz
  CLAUDE.md "Failed to create plan with tolerance" sekcja, mapa może
  wymagać czyszczenia)

## Część 4 — Kalibracja waypointów (10-15 min)

### 4.1 Otwórz mapę w RViz (jak Krok 3.8) i znajdź pozycje biurek

W panelu Tools wybierz **Publish Point** (skrót `P`). Kliknij na
przedniej krawędzi biurka A. RViz w konsoli zaloguje:
```
[INFO] Publishing point ([1.92, 0.0, 0.0])
```

Zapisz wartości. Powtórz dla biurka B.

### 4.2 Oblicz predock pose

Dla każdego biurka:
- `predock_x = krawędź_x − 0.50` (50 cm przed krawędzią)
- `predock_y = krawędź_y` (centered)
- `predock_yaw` = orientacja "robot patrzy na biurko":
  - biurko po +X od robota → `yaw = 0.0`
  - biurko po −X → `yaw = 3.14159`
  - biurko po +Y → `yaw = 1.5708`
  - biurko po −Y → `yaw = -1.5708`

### 4.3 Edytuj waypoints.yaml

```bash
nano src/g1_courier_mission/config/waypoints.yaml
```

Wstaw obliczone wartości:
```yaml
tables:
  table_a:
    predock_x: 1.42
    predock_y: 0.00
    predock_yaw: 0.0
    dock_mode: apriltag
    final_xy_tol_m: 0.03
    final_yaw_tol_rad: 0.05
  table_b:
    predock_x: 4.10
    predock_y: 0.00
    predock_yaw: 0.0
    dock_mode: apriltag
    final_xy_tol_m: 0.03
    final_yaw_tol_rad: 0.05
```

### 4.4 Rebuild mission package

```bash
colcon build --symlink-install --packages-select g1_courier_mission
source install/setup.bash
```

## Część 5 — AMCL + nav2 sanity (10 min)

Test nawigacji **bez mission BT** (manualny goal z RViz).

### 5.1 Postaw karton z AprilTag id=10 na biurku A

Tag ma być na ścianie kartonu, **nie** na górze. Karton centrowany na
biurku A.

### 5.2 Odpal real.launch.py

```bash
ros2 launch g1_courier_bringup real.launch.py map:=$HOME/maps/lab.yaml
```

Czekaj ~10-15 s. Spodziewany log:
- `nav2_lifecycle_manager: Managed nodes are active`
- `mission_node` rzuca `[STAGE START] navigate_to_table_a`

**Wstrzymaj mission BT** żeby testować ręcznie:
```bash
# w innym terminalu:
ros2 service call /safety/set_freeze g1_courier_msgs/srv/SetFreeze '{freeze: true}'
```

To zamraża locomotion. Mission BT nadal woła nav2 ale nic się nie rusza.

### 5.3 Sprawdź AMCL pose

```bash
ros2 topic echo /amcl_pose --field pose.pose.position --once
```

**Checkpoint**: pozycja zgadza się z fizyczną (gdzie robot stoi w lab)
± 50 cm.

**Co jeśli źle**: w RViz **2D Pose Estimate** → przeciągnij strzałkę
na map w faktycznej pozycji robota.

### 5.4 Manualny nav goal — predock_a

```bash
ros2 service call /safety/set_freeze g1_courier_msgs/srv/SetFreeze '{freeze: false}'

ros2 action send_goal /courier/navigate_to_pose \
  g1_courier_msgs/action/NavigateToPose \
  '{target_pose: {header: {frame_id: "map"},
                  pose: {position: {x: 1.42, y: 0.0, z: 0.0},
                         orientation: {w: 1.0}}},
    waypoint_name: "predock_a", xy_tolerance_m: 0.0, yaw_tolerance_rad: 0.0,
    timeout_s: 60.0}' \
  --feedback
```

(Podstaw twoje predock_x/y/yaw. Dla yaw=0 quaternion to `{w: 1.0}`.
Dla yaw=π: `{z: 1.0, w: 0.0}`. Dla yaw=π/2: `{z: 0.7071, w: 0.7071}`.
Dla yaw=−π/2: `{z: -0.7071, w: 0.7071}`.)

**Checkpoint**: robot fizycznie dojeżdża do predock_a (~50 cm od
biurka), kończy `success`.

### 5.5 Sprawdź czy tag10 widoczny z predock_a

```bash
ros2 topic echo /detections --once
```

**Checkpoint**: widzisz `id: 10`, `corners` z niezerowymi wartościami,
`centre` w okolicy `(320, 240)` (czyli centrum obrazu 640×480).

**Co jeśli nie**:
- `id: 10` brak → tag wypadł z FoV. Zmień `predock_x/y/yaw` w
  waypoints.yaml żeby tag był wycentrowany. Podgląd na żywo:
  ```bash
  python3 ~/g1_courier_ws/tools/cam_viewer.py
  ```
- `centre` daleko od (320, 240) → tag w FoV ale z boku. Wyceniuj
  predock_yaw albo predock_y.

### 5.6 Powtórz dla predock_b

Postaw karton na biurku B, manualny nav do predock_b, sprawdź
detekcję tag10.

Plus zatrzymaj launch (Ctrl+C).

## Część 6 — Pojedyncze skille (15-20 min)

Robot przy predock_a, karton z tagiem id=10 na biurku A. Launch
`real.launch.py` chodzi.

### 6.1 Test PICK

```bash
ros2 service call /safety/set_freeze g1_courier_msgs/srv/SetFreeze '{freeze: true}'

ros2 action send_goal /pick_box g1_courier_msgs/action/PickBox \
  '{box_pose: {header: {frame_id: ""}}, sequence_name: "pick_box", timeout_s: 30.0}' \
  --feedback
```

**Checkpoint w trakcie**:
- ramiona wykonują sekwencję P0 → P1 → P2 → P3 (approach) → P4 (grasp)
  → P5 (lift) → P6 (carry_pose)
- czas total ~14 s (7 stages × 2 s)
- result: `grasp_verified: true` lub false

**Co jeśli `grasp_verified=false`**:
- ręcznie zmierz `tau_est` baseline przed pickem i po lift:
  ```bash
  ros2 topic echo /lowstate --field motor_state[15].tau_est --once
  ```
- jeśli różnica < 1.5 Nm → karton za lekki dla domyślnego threshold.
  Edytuj `arm_skills.yaml`:
  ```yaml
  pick_action_server:
    ros__parameters:
      grasp_tau_threshold_nm: 0.5    # niższy
  ```
- albo wyłącz weryfikację dla testu:
  ```yaml
  require_grasp_verified: false
  ```

### 6.2 Test PLACE (po pick)

Robot trzyma karton (po success picka). Test odłożenia (zostań przy
biurku A na razie):

```bash
ros2 service call /safety/set_carry_mode g1_courier_msgs/srv/SetCarryMode '{carrying: true}'

ros2 action send_goal /place_box g1_courier_msgs/action/PlaceBox \
  '{target_pose: {header: {frame_id: ""}}, sequence_name: "place_box", timeout_s: 30.0}' \
  --feedback
```

**Checkpoint**: ramiona wykonują P5 (snap) → P5 (hold 2s) → P4 → P3 →
P2 → ZERO. Karton ląduje na biurku. Wynik `release_verified: true`.

### 6.3 Test DOCK APRILTAG (do tag10)

Cofnij robota ręcznie ~30 cm od kartonu na biurku A (żeby tag był w FoV ale dock musi się zbiec):

```bash
ros2 service call /safety/set_freeze g1_courier_msgs/srv/SetFreeze '{freeze: false}'

ros2 action send_goal /dock_to_table g1_courier_msgs/action/DockToTable \
  '{mode: 0, apriltag_id: 10,
    target_pose: {header: {frame_id: "tag_box"}},
    xy_tolerance_m: 0.10, yaw_tolerance_rad: 0.15, timeout_s: 25.0}' \
  --feedback
```

**Checkpoint**: dock zbiega do ~17 cm od taga, kończy `success` z
`final_xy_error_m < 0.10`.

**Co jeśli timeout / "no detection"**:
- tag wypadł z FoV → cofnij robota dalej, postaw bliżej osi
- intrinsics D435i nie zgadzają się → sprawdź `/camera/color/camera_info`
  z `fx ≈ 615`, `fy ≈ 615`, `cx ≈ 320`, `cy ≈ 240`

### 6.4 Test DOCK LIDAR_LINE (z paczką w rękach)

Gdy robot trzyma karton (carry mode aktywny), tag10 niewidoczny dla
head_cam (zasłonięty). Wtedy działamy LIDAR_LINE:

```bash
# Robot z paczką, ~50 cm od biurka B (manualnie zaprowadź teleopem):
ros2 action send_goal /dock_to_table g1_courier_msgs/action/DockToTable \
  '{mode: 1, apriltag_id: 0,
    target_pose: {header: {frame_id: "map"},
                  pose: {position: {x: 4.10, y: 0.0, z: 0.0}}},
    xy_tolerance_m: 0.03, yaw_tolerance_rad: 0.05, timeout_s: 25.0}' \
  --feedback
```

**Checkpoint**: lidar_viewer (jeśli odpalony) pokazuje czerwoną linię
RANSAC fittowaną do krawędzi biurka. Robot zbiega prostopadle.

```bash
python3 ~/g1_courier_ws/tools/lidar_viewer.py
```

### 6.5 Test carry_mode + freeze services

```bash
# włącz carry-mode caps:
ros2 service call /safety/set_carry_mode g1_courier_msgs/srv/SetCarryMode '{carrying: true}'
# spodziewane: max_vx ~0.3 zamiast 0.6

# wyłącz:
ros2 service call /safety/set_carry_mode g1_courier_msgs/srv/SetCarryMode '{carrying: false}'

# emergency freeze:
ros2 service call /safety/set_freeze g1_courier_msgs/srv/SetFreeze '{freeze: true}'
# spodziewane: cmd_vel = zero niezależnie od source

# wyłącz freeze:
ros2 service call /safety/set_freeze g1_courier_msgs/srv/SetFreeze '{freeze: false}'
```

## Część 7 — Pełen mission cycle (20 min)

Po success Części 6, można odpalić pełen cykl A↔B.

### 7.1 Setup początkowy

- karton z tagiem id=10 na biurku A
- robot w okolicach origin (0, 0) na mapie, ramiona w pozycji rest
- `~/maps/lab.yaml` skalibrowane (Część 4)

### 7.2 Test pojedynczego cyklu (max_cycles=1)

```bash
ros2 launch g1_courier_bringup real.launch.py \
  map:=$HOME/maps/lab.yaml \
  max_cycles:=1
```

**Spodziewana sekwencja log**:
1. AMCL converge na map (~5 s)
2. `[STAGE START] navigate_to_table_a` → `OK`
3. `[STAGE START] dock_to_box_table_a` → `OK`
4. `[STAGE START] pick_at_table_a` → `OK, grasp_verified=true`
5. `[STAGE START] retreat_after_pick_table_a` → `OK`
6. carry_mode=True
7. `[STAGE START] navigate_to_table_b` → `OK`
8. `[STAGE START] dock_to_table_b` (LIDAR_LINE) → `OK`
9. `[STAGE START] place_at_table_b` → `OK, release_verified=true`
10. `[STAGE START] retreat_after_place_table_b` → `OK`
11. carry_mode=False
12. (drugi pół cyklu — A jako miejsce, B jako pickup)
13. `Mission complete after 1 cycles`

### 7.3 Monitorowanie w trakcie

Cztery dodatkowe terminale:

```bash
# T1 — log mission status:
watch -n 1 'ros2 topic echo /mission_status --once 2>/dev/null'

# T2 — kamera z tag overlay:
python3 ~/g1_courier_ws/tools/cam_viewer.py

# T3 — lidar 2D top-down + RANSAC:
python3 ~/g1_courier_ws/tools/lidar_viewer.py

# T4 — nav2 plan + AMCL pose (refresh /tmp/nav_plan.png):
python3 ~/g1_courier_ws/tools/plan_viz.py &
feh --reload 1 /tmp/nav_plan.png
```

### 7.4 Pełen loop (gdy single cycle OK)

```bash
ros2 launch g1_courier_bringup real.launch.py map:=$HOME/maps/lab.yaml
# bez max_cycles = pętla nieskończona
```

Cykl: ~60-90 s. 100 cykli ≈ 1.5-2 godziny ciągłej pracy.

## Część 8 — Co jeśli coś nie działa

Pełen katalog symptomów + diagnostyk + fixów:
[`docs/deployment_guide.md`](deployment_guide.md) § "Troubleshooting".

Najczęstsze:

| Symptom | Diagnostyka | Fix |
|---|---|---|
| `/livox/lidar` 0 Hz | `lsusb \| grep Livox` | kabel USB, restart drivera |
| `/scan` 0 Hz | TF graph (Krok 2.2) | static_transform_publisher base_link → livox_frame |
| AMCL drift | RViz pose vs prawda | manual 2D Pose Estimate, lub re-record map |
| Dock APRILTAG timeout | `cam_viewer.py` | predock_x/y/yaw, ekspozycja D435i |
| `grasp_verified=false` | `tau_est` przed/po | tune `grasp_tau_threshold_nm` |
| Mission BT pętla na fazie | `/mission_status` | dock_mode lidar_line dla return leg |
| nav2 controller stoi | `ros2 topic hz /cmd_vel` | SetRemap cmd_vel → cmd_vel_nav |

## Skrót dla powtórnej wizyty

Gdy mapa + waypoints już skalibrowane (z poprzedniej wizyty), pomiń
Części 3 i 4. Procedura skrócona:

1. Część 1 (bring-up)
2. Część 2 (health check)
3. Część 5.4 (manualny nav do predock_a) — sanity że AMCL+nav działają
4. Część 7 (pełen cykl)

Czas: ~15 min od zimnego startu do pierwszego cyklu.
