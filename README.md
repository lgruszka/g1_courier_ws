# g1_courier_ws

Stack ROS2 dla humanoida Unitree G1 — przenoszenie kartonu między dwoma
biurkami oznaczonymi AprilTagami.

> **Tu jesteś**: branch `courier-deploy` — kod dla **realnego Unitree G1**.
> Jeśli chcesz najpierw bawić się symulatorem (rekomendowane),
> `git checkout courier-sim` i przeczytaj README tamtego brancha.

## TL;DR — pierwszy raz w zespole?

1. **Wybierz branch** (patrz [Branche](#branche) niżej):
   - Masz tylko laptopa → `courier-sim`
   - Jesteś w labie z prawdziwym G1 → `courier-deploy` (ten)
2. **Rzuć okiem na diagram architektury** ([tutaj](#diagram-architektury))
   żeby wiedzieć która warstwa za co odpowiada. Na razie nie musisz
   rozumieć wszystkiego dogłębnie.
3. **Build workspace'u** ([5 minut](#build-i-source)).
4. **Odpal pełną misję** ([Pełne uruchomienie misji](#pełne-uruchomienie-misji)) —
   jeden launch podnosi całość.
5. **Pierwsze uruchomienie krok po kroku** (od zimnego startu po pełen
   cykl A↔B, każdy krok z checkpointem diagnostycznym):
   [docs/deployment_test_procedure.md](docs/deployment_test_procedure.md).
6. **Gdy coś się zepsuje**:
   - [Pojedyncze skille — debug](#pojedyncze-skille--debug) — odpal jedną akcję, zobacz co się dzieje
   - [Podgląd diagnostyczny](#podgląd-diagnostyczny) — wizualizacja kamery, lidaru, nav
   - [docs/deployment_guide.md](docs/deployment_guide.md) — pełen troubleshooting
7. **Zanim zmienisz kod**, przeczytaj
   [docs/ARCHITECTURE.md](docs/ARCHITECTURE.md) — to jest źródło prawdy
   dla decyzji projektowych i konwencji.

## Branche

Repo zespołu (`gitlab.com/iAndy77/j2s`) ma trzy orphan branche o
niezależnych historiach. Wybierz pasujący do twojego targetu — wszystkie
dzielą warstwy mission/skills/platform, różnią się tym co działa pod
spodem.

| Branch | Co to | Kiedy używać |
|---|---|---|
| **`courier-sim`** | Ubuntu native MuJoCo bridge + pełen stack misji | Codzienna praca, bez fizycznego robota |
| **`courier-deploy`** | Realny Unitree G1 + Livox Mid-360 + RealSense D435i | Sesje labowe z prawdziwym robotem |
| `courier-sim-legacy-mac` | Frozen snapshot oryginalnego setupu mac MuJoCo + Linux Parallels | Tylko historia — nie utrzymywany |

Przełączanie branchy:
```bash
cd ~/g1_courier_ws/src/courier
git fetch
git checkout courier-sim       # albo z powrotem na courier-deploy
cd ../..
rm -rf build install log        # wyczyść artefakty z poprzedniego brancha
colcon build --symlink-install
source install/setup.bash
```

## Wymagania sprzętowe (ten branch)

Dla sim — patrz README brancha `courier-sim`, ta sekcja nie ma
zastosowania.

- Unitree G1 (29-DoF) z włączonym `arm_sdk` i gotowym sport API
- Livox Mid-360 zamontowany na głowie, USB do onboard PC
- RealSense D435i w torsie, USB3
- Dwa biurka (table A, table B) — **bez AprilTagów na biurkach**.
  Lokalizacja biurek tylko z mapy + LiDAR line fit (RANSAC na krawędzi).
- Tekturowy karton z AprilTagiem `tag36h11` `id=10` na ścianie kartonu
  (rozmiar 0.10 m) — to JEDYNY tag używany w systemie. Służy tylko do
  precyzyjnego docka pod chwyt (`pick_box`).
- Zapisana mapa 2D w `~/maps/lab.yaml` (zbudowana raz przez
  `mapping_real.launch.py`)
- Skalibrowane waypoints w `src/g1_courier_mission/config/waypoints.yaml`

Pełen checklist sprzętowy + procedura kalibracji:
[docs/deployment_guide.md](docs/deployment_guide.md).

## Build i source

```bash
cd ~/g1_courier_ws
colcon build --symlink-install
source install/setup.bash
# Dodaj `source ~/g1_courier_ws/install/setup.bash` do ~/.bashrc, żeby
# w nowych powłokach nie sourcować ręcznie.
```

Najczęstsze powody błędu buildu:

- Brakujące pakiety apt → patrz
  [Wymagane zależności zewnętrzne](#wymagane-zależności-zewnętrzne)
- Brakujący `unitree_hg` / `unitree_api` → sklonuj `unitree_ros2` do
  `src/` (deployment guide § "Workspace + sources")

## Pełne uruchomienie misji

Jeden launch podnosi cały stack: nav2, AMCL, detector AprilTag, konwerter
LiDAR-do-laserscan, action serwery dock / pick / place / retreat,
mission BT.

```bash
ros2 launch g1_courier_bringup real.launch.py map:=$HOME/maps/lab.yaml
```

Co się dzieje, krok po kroku:

1. AMCL ładuje `lab.yaml`, czeka na initial pose (jeśli nie zbiega
   automatycznie, ustaw "2D Pose Estimate" z RViz).
2. Mission BT nawiguje do predocka biurka A.
3. Dock APRILTAG → box tag10 (17 cm) — biurko nie ma własnego taga,
   robot zbiega bezpośrednio do kartonu.
4. `pick_box` wykonuje keyframes P0..P6, grasp verifier potwierdza
   skok τ.
5. Włącza się carry mode — `cmd_vel_arbiter` obniża limity prędkości.
6. Nawigacja do predocka biurka B.
7. Dock LIDAR_LINE (head_cam zasłonięta przez niesiony karton).
8. `place_box` wykonuje P5..ZERO, release verifier potwierdza spadek τ.
9. Retreat 1.0 m, swap A↔B, kolejny cykl.

Czyste zatrzymanie: `Ctrl+C` w terminalu launch.

> **Co obserwować**: obok odpal RViz (`ros2 run rviz2 rviz2`) z displayami
> Map, AMCL Pose, Nav2 Plan, TF. Plus
> [podglądy diagnostyczne](#podgląd-diagnostyczny) niżej.

### Limit cykli do testów

Domyślnie BT pętli się w nieskończoność. Żeby uruchomić tylko N cykli:

```bash
ros2 launch g1_courier_bringup real.launch.py \
  map:=$HOME/maps/lab.yaml \
  max_cycles:=3
```

(`max_cycles` to parameter `mission_node`; 0 = nieskończoność.)

## Pojedyncze skille — debug

Gdy chcesz przetestować jeden action server w izolacji. Przydatne do:

- Sprawdzenia zmiany kalibracji bez pełnego cyklu misji
- Reprodukcji buga który pojawia się tylko w jednej fazie
- Onboardingu — wyczucia kontraktów action po kolei

Wszystkie przykłady niżej zakładają że `real.launch.py` chodzi w innym
terminalu, albo przynajmniej działa odpowiedni action server.

### Pick

```bash
# Nominalne keyframes P0..P6 (bez offsetu z mierzonej pozycji boxa).
ros2 action send_goal /pick_box g1_courier_msgs/action/PickBox \
  '{box_pose: {header: {frame_id: ""}}, sequence_name: "pick_box", timeout_s: 30.0}' \
  --feedback
```

Zobaczysz progresję faz `wait_for_state → approach → grasp → lift → verify`.
Result zawiera `grasp_verified: true|false`.

### Place

```bash
ros2 action send_goal /place_box g1_courier_msgs/action/PlaceBox \
  '{target_pose: {header: {frame_id: ""}}, sequence_name: "place_box", timeout_s: 30.0}' \
  --feedback
```

### Dock do kartonu (tryb AprilTag, tag10)

W systemie używamy AprilTaga **tylko na kartonie** (id=10). Biurka nie
mają tagów — pozycja biurek bierze się z mapy plus LIDAR_LINE.

```bash
ros2 action send_goal /dock_to_table g1_courier_msgs/action/DockToTable \
  '{mode: 0, apriltag_id: 10,
    target_pose: {header: {frame_id: "tag_box"}},
    xy_tolerance_m: 0.10, yaw_tolerance_rad: 0.15, timeout_s: 25.0}' \
  --feedback
```

`mode: 0` to `MODE_APRILTAG`, `1` to `MODE_LIDAR_LINE`, `2` to `MODE_AMCL_ONLY`
(patrz `src/g1_courier_msgs/action/DockToTable.action`).

### Dock do biurka (tryb LiDAR line — gdy niesiesz karton)

```bash
ros2 action send_goal /dock_to_table g1_courier_msgs/action/DockToTable \
  '{mode: 1, apriltag_id: 0,
    target_pose: {header: {frame_id: "map"},
                  pose: {position: {x: 4.10, y: 0.0, z: 0.0}}},
    xy_tolerance_m: 0.03, yaw_tolerance_rad: 0.05, timeout_s: 25.0}' \
  --feedback
```

### Nawigacja do pozy 2D

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

(`xy_tolerance_m: 0.0` znaczy "użyj domyślnej z `nav2_params.yaml`".)

### Retreat (open-loop wycofanie)

```bash
ros2 action send_goal /retreat g1_courier_msgs/action/Retreat \
  '{distance_m: 0.5, speed_mps: 0.15, timeout_s: 10.0}' \
  --feedback
```

### Carry mode (services na cmd_vel_arbiter)

```bash
# Włącz limity carry-mode:
ros2 service call /safety/set_carry_mode g1_courier_msgs/srv/SetCarryMode \
  '{carrying: true}'

# Freeze locomotion (publikuj zerowe cmd_vel niezależnie od upstreamu):
ros2 service call /safety/set_freeze g1_courier_msgs/srv/SetFreeze \
  '{freeze: true}'
```

### Stan mission BT

```bash
# Latched mission status (licznik cykli, aktualna faza):
ros2 topic echo /mission_status --once

# Live BT tick logs (BT loguje przejścia między fazami):
ros2 node info /mission_node
```

## Podgląd diagnostyczny

Odpalaj obok `real.launch.py` (każdy w swoim terminalu):

```bash
python3 tools/cam_viewer.py
# Okno z /head_cam image + bbox AprilTagów + dystans z PnP.
# Przydatne do: sprawdzania ekspozycji, ostrości, zasięgu detection.

python3 tools/lidar_viewer.py
# Top-down 2D scan + RANSAC line fit (algorytm jak dock LIDAR_LINE).
# Przydatne do: sprawdzania wysokości slice'a pcl-to-laserscan, alignmentu docka.

python3 tools/plan_viz.py
# Zapisuje /tmp/nav_plan.png przy każdym update'cie planu nav2.
# Podgląd: feh --reload 1 /tmp/nav_plan.png
# Przydatne do: konwergencji AMCL i jakości ścieżki nav2 vs costmap.
```

Skróty klawiszowe (`q`/`s`) opisane w `tools/README.md`.

### RViz z modelem 3D robota

Preset z gotowymi displays (RobotModel z URDF, Map, /scan, nav2 plan,
AMCL pose, TF):

```bash
rviz2 -d $(ros2 pkg prefix g1_courier_bringup)/share/g1_courier_bringup/rviz/courier.rviz
```

Działa zarówno z `real.launch.py` (real robot) jak i `phase1_full.launch.py`
(sim) — oba publikują `/robot_description` przez `robot_state_publisher`
plus `/joint_states` przez `lowstate_to_joint_states`. Wymaga sklonowanego
`unitree_ros` w `src/` (URDF G1).

W RViz: **Fixed Frame: map** (default w presecie). Przełącz na `base_link`
jeśli chcesz local view robota podczas docka.

## Budowanie mapy (raz na lab)

```bash
ros2 launch g1_courier_bringup mapping_real.launch.py
# W innym terminalu:
ros2 run teleop_twist_keyboard teleop_twist_keyboard
# Przejedź robotem przez cały lab pokrywając oba biurka i obszar tranzytowy.
# Gdy mapa wygląda kompletnie:
ros2 run nav2_map_server map_saver_cli -f ~/maps/lab
```

Następnie zaktualizuj `src/g1_courier_mission/config/waypoints.yaml`
zgodnie z nową mapą — patrz deployment guide § "Calibrate waypoints
from map" (procedura w RViz).

## Diagram architektury

```
┌─────────────────────────────────────────────────────────────┐
│  Mission layer       g1_courier_mission                     │
│  - py_trees_ros Behavior Tree                               │
│  - blackboard: cycle_count, current_target, box_held        │
│  - woła skille jako akcje ROS2, retry on failure            │
└──────────────┬──────────────────────────────────────────────┘
               │ akcje ROS2 (g1_courier_msgs)
┌──────────────┴──────────────────────────────────────────────┐
│  Skills layer                                               │
│  ┌─────────────────────┐ ┌────────────────────────────────┐ │
│  │ NavigateToPose      │ │ DockToTable                    │ │
│  │ wrapper nav2        │ │ MODE_APRILTAG / LIDAR / AMCL   │ │
│  └─────────────────────┘ └────────────────────────────────┘ │
│  ┌─────────────────────┐ ┌────────────────────────────────┐ │
│  │ PickBox / PlaceBox  │ │ Retreat (open-loop wycofanie)  │ │
│  └─────────────────────┘ └────────────────────────────────┘ │
└──────────────┬──────────────────────────────────────────────┘
               │ /cmd_vel_*, /arm_sdk, /lowstate, TF, /scan
┌──────────────┴──────────────────────────────────────────────┐
│  Platform layer                                             │
│  ┌─────────────────────┐ ┌────────────────────────────────┐ │
│  │ Nav2 stack          │ │ Lokalizacja                    │ │
│  │ planner / controller│ │ slam_toolbox (mapowanie)       │ │
│  └─────────────────────┘ │ nav2_amcl (działanie)          │ │
│  ┌─────────────────────┐ └────────────────────────────────┘ │
│  │ cmd_vel_arbiter     │ ┌────────────────────────────────┐ │
│  │ - priorytety        │ │ Sensory                        │ │
│  │ - limity carry-mode │ │ Mid360 LiDAR + RealSense D435i │ │
│  │ - freeze + e-stop   │ │ apriltag_ros                   │ │
│  └─────────────────────┘ │ pointcloud_to_laserscan        │ │
│                          └────────────────────────────────┘ │
└─────────────────────────────────────────────────────────────┘
                              │
                              ↓ /lowstate, /livox/lidar, /camera/...
                       Firmware bridges Unitree +
                       sterowniki Livox + RealSense
```

Reguła wertykalna: każda warstwa rozmawia **w dół** przez akcje /
topiki ROS2 zdefiniowane w `g1_courier_msgs`. Skill nigdy nie woła
innego skilla — kompozycja jest w mission BT.

## Pakiety

| Pakiet | Typ | Rola |
|---|---|---|
| `g1_courier_msgs` | ament_cmake | Definicje action / srv / msg — API systemu |
| `g1_courier_arm_skills` | ament_python | Action serwery `PickBox` i `PlaceBox`, parametryczny arm controller |
| `g1_courier_docking` | ament_python | Action server `DockToTable` z trybami AprilTag / LiDAR / AMCL |
| `g1_courier_mission` | ament_python | Mission node (Behavior Tree), `NavigateToPose` proxy, `Retreat` |
| `g1_courier_safety` | ament_python | `cmd_vel` arbiter z priorytetami, carry mode, freeze, e-stop |
| `g1_courier_bringup` | ament_python | Launch i configi (nav2, slam_toolbox, AMCL, AprilTag, ...) |

## Przepływ misji (BT wykonuje to)

```
loop forever:
  set_carry_mode(off)
  navigate_to_pose(predock_table_A)        # nav2 + AMCL
  dock_to_box(mode=APRILTAG, tag=10)       # 6-DoF visual servo do tag10 na kartonie
  pick_box(box_pose_from_tag)              # parametryczna trajektoria ramion
  verify_grasp                             # próg τ
  set_carry_mode(on)                       # niższe vx/vyaw, mniejsze kroki

  navigate_to_pose(predock_table_B)        # nav2 + AMCL only
  dock_to_table(mode=LIDAR_LINE)           # kamera zasłonięta — RANSAC krawędź
  place_box(target_pose_from_lidar)        # parametryczne odłożenie
  verify_release

  set_carry_mode(off)
  retreat(1.0 m)
  swap A <-> B
```

BT ma retry policy na dock i pick. Nieudany `verify_grasp` powtarza
dock + pick. Nieudany `verify_release` eskaluje do abort.

## Jak rozwiązaliśmy zasłanianie kamery

Gdy robot niesie karton, kamera frontowa jest w większości zasłonięta —
detection AprilTaga staje się zawodny. Stack rozwiązuje to przez
**niekorzystanie z AprilTag do globalnej lokalizacji**. Globalnie
zawsze AMCL na 2D laser scan z LiDARu. AprilTag używamy tylko do
finalnego doprecyzowania zbliżenia do biurka pickup (bo pick wymaga
tolerancji ±2-3 cm). Do biurka place podchodzimy dwoma tańszymi
środkami łącznie:

1. AMCL pose, dokładny do ~±5 cm w dobrze zmapowanym środowisku.
2. LiDAR line fitting na krawędzi biurka (`MODE_LIDAR_LINE`), korygujący
   resztkowy lateral i yaw error bezpośrednio z scan, niezależnie od
   stanu kamery.

Action dock przyjmuje argument `mode`, więc mission BT wybiera per-table
jaki poziom doprecyzowania jest potrzebny.

## Koordynacja loco↔arms

- `cmd_vel_arbiter` udostępnia service `freeze`. Przed każdą akcją ramion
  mission node ustawia freeze=true, arbiter publikuje zerowe prędkości,
  arm action czeka aż lowstate zaraportuje prędkość ciała poniżej progu
  zanim wyśle pierwszy setpoint do ramion.
- `cmd_vel_arbiter` udostępnia service `carry_mode`. Z trzymanym
  kartonem ogranicza `max_vx`, `max_vy`, `max_vyaw` do niższych
  wartości (config).
- Po `place_box` ramiona są sprowadzane do zera z weight ramped 0,
  zwracając kontrolę FSM-owi.

## Configi które prawdopodobnie zechcesz tunować

| Plik | Co tunować | Kiedy |
|---|---|---|
| `mission/config/waypoints.yaml` | `predock_x/y/yaw` per-biurko | Każda nowa mapa |
| `arm_skills/config/arm_skills.yaml` | `grasp_tau_threshold_nm` | Per masa kartonu |
| `docking/config/docking.yaml` | `kp_xy`, `kp_yaw`, `target_distance` | Pierwszy deploy + per oświetlenie |
| `safety/config/safety.yaml` | `max_v*_carry` | Per masa kartonu + balans |
| `bringup/config/nav2_params.yaml` | inflation costmapy, footprint | Per zatłoczenie laba |

Why-and-how dla każdego jest w
[deployment guide](docs/deployment_guide.md) § "Configs that you may
need to tune".

## Co jest zaimplementowane

- Wszystkie interfejsy action / service / msg (`g1_courier_msgs`).
- Parametryczny arm controller z CRC, biblioteką keyframes (`P0..P6`
  z kalibracji realnego G1), weight ramping, hookiem grasp verifier,
  sentinelem kinematic-mode (`mode==99`) dla sim.
- Action serwery `PickBox` i `PlaceBox` z grasp verifierem.
- Action server `DockToTable` ze wszystkimi trzema trybami:
  - `MODE_APRILTAG` — 6-DoF PnP visual servo do tag10 na kartonie (tylko ten jeden tag)
  - `MODE_LIDAR_LINE` — RANSAC line fit na 2D scan, perpendicular alignment
    (gdy niesiony karton zasłania head_cam)
  - `MODE_AMCL_ONLY` — zaufanie AMCL (fallback)
- `cmd_vel_arbiter` z priorytetami (dock → retreat → nav → /cmd_vel),
  limitami carry-mode, freeze service, e-stop latch.
- Mission BT z cyklem: `pickup_at_a → transfer_b → pickup_b →
  transfer_a` z logami `[STAGE START]`/`[STAGE END]`.
- nav2 stack: AMCL OmniMotionModel + NavfnPlanner A\* + RotationShim →
  RegulatedPurePursuit + costmaps z warstwami obstacle/inflation.

## Wymagane zależności zewnętrzne

apt (ROS2 Humble):
```
ros-humble-nav2-bringup
ros-humble-slam-toolbox
ros-humble-pointcloud-to-laserscan
ros-humble-py-trees-ros
ros-humble-rosidl-generator-dds-idl
ros-humble-apriltag-ros
ros-humble-apriltag-msgs
ros-humble-tf2-geometry-msgs
ros-humble-realsense2-camera
ros-humble-teleop-twist-keyboard
```

ROS2 source (clone do `src/`):
```
unitree_ros2          IDLs unitree_hg / unitree_api / unitree_go
                      https://github.com/unitreerobotics/unitree_ros2
unitree_ros           URDF g1_description + meshes (master branch)
                      https://github.com/unitreerobotics/unitree_ros
livox_ros_driver2     Sterownik Livox Mid-360
                      https://github.com/Livox-SDK/livox_ros_driver2
```

## Indeks dokumentacji

- [docs/ARCHITECTURE.md](docs/ARCHITECTURE.md) — reguły projektowe,
  granice warstw, anti-patterns. **Czytaj zanim zmienisz core kod.**
- [docs/deployment_guide.md](docs/deployment_guide.md) — pełen setup
  sprzętowy, procedura kalibracji, troubleshooting.
- [docs/deployment_test_procedure.md](docs/deployment_test_procedure.md) —
  procedura testowa krok po kroku (od zimnego startu do pełnego cyklu
  A↔B), każdy krok z checkpointem diagnostycznym.
- [tools/README.md](tools/README.md) — szczegóły podglądów diagnostycznych.
- [docs/phases/](docs/phases/) — historyczna kronika faz developmentu
  (skupiona na sim; przydatna do zrozumienia *dlaczego* obecny design
  wygląda jak wygląda).
