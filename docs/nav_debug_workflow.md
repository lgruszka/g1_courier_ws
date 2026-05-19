# Nav debug workflow — kolejność testów + parametrów do tunowania

Pragmatyczny zestaw kroków do debugowania nawigacji na realnym robocie.
Idziesz od najprostszego (czy zegar OK) do najbardziej złożonego
(tuning controller'a). Plus każda faza ma **stop point** — nie idź
dalej dopóki ✅.

## Pliki które będziesz edytować

| Plik | Co tu jest |
|---|---|
| `src/g1_courier_bringup/config/nav2_params.yaml` | AMCL, planner, controller, costmaps, goal checker |
| `src/g1_courier_safety/config/safety.yaml` | `cmd_vel_arbiter` floors (min_v*_threshold) |
| `src/g1_courier_bringup/config/pointcloud_to_laserscan.yaml` | Z slice dla `/scan` |
| `src/g1_courier_bringup/launch/real.launch.py` | static TF, `enable_mission` |

Po edycji każdego z tych plików:
```bash
colcon build --symlink-install --packages-select g1_courier_bringup g1_courier_safety
source install/setup.bash
# restart real.launch.py
```

---

# Faza 0 — Pre-flight (3 min)

```bash
# 0.1 — zegar
timedatectl | grep -E '(NTP|synchronized)'
# musi: active + yes; inaczej:
sudo timedatectl set-ntp true   # czekaj 5 s

# 0.2 — zombie kill
pkill -9 -f mission_node
pkill -9 -f component_container

# 0.3 — odpal stack bez mission BT
source /home/neo/j2s/install/setup.bash
ros2 launch g1_courier_bringup real.launch.py \
  map:=$HOME/maps/scenarios_g1_map_flipped/<mapa>.yaml \
  enable_mission:=false
# czekaj na: "Managed nodes are active"

# 0.4 — RViz (drugi terminal)
source /home/neo/j2s/install/setup.bash
LIBGL_ALWAYS_SOFTWARE=1 rviz2 \
  -d $(ros2 pkg prefix g1_courier_bringup)/share/g1_courier_bringup/rviz/courier.rviz
```

**STOP** — czekaj na "Managed nodes are active" zanim pójdziesz dalej.

---

# Faza 1 — Topic sanity (5 min)

Sprawdzenie czy podstawowe dane płyną. **Bez tego nawigacja nie ma szans**.

```bash
# 1.1 — Livox + LaserScan
ros2 topic hz /livox/lidar                              # ~10 Hz
ros2 topic hz /scan --qos-reliability best_effort       # ~10 Hz
ros2 topic echo /scan --qos-reliability best_effort --once --field header.frame_id
# musi: base_link (po pointcloud_to_laserscan)

# 1.2 — Firmware
ros2 topic hz /lowstate                                  # ~500 Hz
ros2 topic hz /dog_odom                                  # ~50 Hz

# 1.3 — AMCL aktywne
ros2 lifecycle get /amcl                                 # active [3]
ros2 lifecycle get /map_server                           # active [3]
ros2 lifecycle get /controller_server                    # active [3]
ros2 lifecycle get /planner_server                       # active [3]
ros2 lifecycle get /bt_navigator                         # active [3]

# 1.4 — TF tree
ros2 run tf2_ros tf2_echo odom base_link                # zwraca Translation
ros2 run tf2_ros tf2_echo base_link livox_frame         # z=1.45, quat zależnie od roll
```

**STOP** — każdy z 1.1-1.4 musi dać sensowny wynik. Jeśli któryś pada:
- `/scan` 0 Hz → patrz problem QoS lub TF (osobny temat)
- `/dog_odom` 0 Hz → firmware nie chodzi
- `/amcl` not active → lifecycle problem
- `tf2_echo` timeout → brak TF chain

---

# Faza 2 — Bridge sanity (izolowany cmd_vel test, 3 min)

**Cel**: sprawdzić czy bridge mapuje cmd_vel poprawnie na ruch robota.
Bez AMCL/nav — tylko ręczne cmd_vel.

⚠️ **Trzymaj pilota E-stop w dłoni**. Robot się ruszy.

```bash
# 2.1 — vx do przodu
ros2 topic pub --once /cmd_vel geometry_msgs/Twist \
  '{linear: {x: 0.15, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.0}}'
# Robot fizycznie:
#   ✅ idzie do przodu (zgodnie z osią X robota)
#   ❌ idzie do tyłu → bridge invertuje X
#   ❌ idzie w bok → mapping X↔Y przekręcony

# 2.2 — vy w bok lewo
ros2 topic pub --once /cmd_vel geometry_msgs/Twist \
  '{linear: {x: 0.0, y: 0.15, z: 0.0}}'
# ✅ fizycznie w lewo

# 2.3 — wz obrót lewo
ros2 topic pub --once /cmd_vel geometry_msgs/Twist \
  '{angular: {x: 0.0, y: 0.0, z: 0.3}}'
# ✅ fizycznie skręca w lewo (counterclockwise from above)
```

**STOP** — wszystkie trzy muszą się zgadzać z fizyką. Jeśli któryś
przekręcony → problem w `unitree_cmd_vel_bridge_node` lub firmware,
**nie** w nav2. Patch wymaga edycji bridge code, **nie** parameter
tweak. Zatrzymaj plus daj znać Łukaszowi.

---

# Faza 3 — AMCL sanity (10 min)

Pełna procedura w `docs/amcl_sanity_test.md`. Skrót:

```
T1 — Set Initial Pose (skan pokrywa ściany ±20 cm) ✅
T2 — Yaw pasuje do fizycznego przodu robota ✅
T3 — Particles skupione (~30 cm po 15 s stania) ✅
T4 — Popchnij robota 30 cm, model RViz podąża ✅
T5 — Covariance < 0.05 (ros2 topic echo /amcl_pose --once --field pose.covariance) ✅
```

**STOP** — jeśli któryś z T1-T5 ❌, fix tam **przed** dalszymi
krokami. Bo controller dostaje pose z AMCL i jeśli to złe, kontroler
wysyła sensowne komendy ale efekt fizyczny jest losowy.

Najczęstsze fixy:
- T1/T2 fail → ponownie 2D Pose Estimate, dokładniej
- T3 fail → `min_particles: 1000, max_particles: 3000` w `amcl` (więcej particles)
- T4 fail → `/dog_odom` problem (Faza 1.2)
- T5 fail (covariance wysoka) → daj AMCL więcej obserwacji — popchnij robota plus poczekaj

---

# Faza 4 — Pierwszy nav goal (5 min)

Krótki test 1 m do przodu w GUI lub RViz.

```bash
# T4.1 — krótki nav z RViz (lub operator_gui):
# RViz pasek narzędzi → 2D Goal Pose → klik 1 m przed, przeciągnij yaw=0 (ten sam co robot)
```

W trakcie patrz **3 terminale**:

```bash
# T4.2 — co controller wysyła:
ros2 topic echo /cmd_vel_nav

# T4.3 — co arbiter wypuszcza (po floor):
ros2 topic echo /cmd_vel

# T4.4 — log BT (recovery, errors):
ros2 launch ... 2>&1 | grep -iE 'error|abort|recover|extrapolation|reject|wait|backup'
```

Plus w RViz dodaj displays:
- **Path** `/plan` — global plan (czerwona linia)
- **Path** `/local_plan` — local plan (zielona linia)
- **Costmap** `/local_costmap/costmap` (Transient Local QoS)
- **Costmap** `/global_costmap/costmap` (Transient Local QoS)

## Kryterium sukcesu Fazy 4

| Sygnał | Spodziewane |
|---|---|
| `/plan` | sensowna prosta linia od robota do celu |
| `/cmd_vel_nav` | vx ~0.2 m/s (nie 0, nie >0.5) gdy robot jedzie |
| `/cmd_vel` | identyczne lub similar do `/cmd_vel_nav` |
| Robot fizycznie | jedzie w kierunku celu plus zatrzymuje się ~30 cm |
| Costmaps | nie pokazują "ducha" obstacle gdzie nic nie ma |

---

# Faza 5 — Tuning gdy Faza 4 pada (rzędu 10-15 min na iterację)

**Najczęstsze problemy z Fazy 4 + fixy** w kolejności prawdopodobieństwa:

## 5.1 — Robot dostaje "skoki" w bok mimo prostego nav goal

**Objaw**: `/cmd_vel_nav` ma vy=0.03, `/cmd_vel` ma vy=0.12.

**Przyczyna**: `min_vy_threshold` floor w arbiter. Małe korekcje są
skalowane do floor → robot dostaje znacznie większy ruch niż controller
chciał.

**Fix**: edytuj `src/g1_courier_safety/config/safety.yaml`:
```yaml
cmd_vel_arbiter:
  ros__parameters:
    min_vx_threshold: 0.12      # zostaw — dock potrzebuje
    min_vy_threshold: 0.0       # ← zmień z 0.12 na 0.0
    min_vyaw_threshold: 0.0
```

Plus rebuild + restart.

## 5.2 — Robot jedzie za szybko / chaotyczne ruchy

**Objaw**: `/cmd_vel_nav` ma vx=0.7 m/s, robot się "rwie".

**Przyczyna**: `desired_linear_vel: 0.8` w controller jest za duże dla biped G1.

**Fix**: edytuj `src/g1_courier_bringup/config/nav2_params.yaml`:
```yaml
controller_server:
  ros__parameters:
    FollowPath:
      desired_linear_vel: 0.25    # ← zmień z 0.8 na 0.25
```

## 5.3 — `Extrapolation Error` / `Unable to transform robot pose`

**Objaw**: w logu `Extrapolation Error looking up target frame: ...
Lookup would require extrapolation into the future/past`.

**Przyczyna**: TF tolerance niska, AMCL publikuje rzadko, Livox skew.

**Fix**: edytuj `nav2_params.yaml`:
```yaml
amcl:
  ros__parameters:
    transform_tolerance: 3.0           # zmień z 0.5 na 3.0

controller_server:
  ros__parameters:
    FollowPath:
      transform_tolerance: 3.0         # zmień z 0.2 na 3.0

global_costmap:
  global_costmap:
    ros__parameters:
      transform_tolerance: 3.0         # zmień z 0.2

local_costmap:
  local_costmap:
    ros__parameters:
      transform_tolerance: 3.0         # zmień z 0.5
```

## 5.4 — Robot kręci się w miejscu zamiast jechać

**Objaw**: `/cmd_vel` ma wz≠0, vx~0. Robot obraca się ale nie postępuje.

**Przyczyna**: `angular_dist_threshold: 0.785` (45°). Gdy plan idzie
>45° od aktualnego yaw, RotationShim odpala in-place rotation.

**Fix opcja A**: zwiększ próg żeby rzadziej odpalał shim:
```yaml
FollowPath:
  angular_dist_threshold: 1.57    # zmień z 0.785 (45°) na 1.57 (90°)
```

**Fix opcja B**: zwolnij rotation żeby nie oscylował:
```yaml
FollowPath:
  rotate_to_heading_angular_vel: 0.4    # zmień z 0.8 na 0.4
```

## 5.5 — Costmaps pokazują "ducha" obstacle

**Objaw**: w RViz `/local_costmap/costmap` widać czerwone obszary tam
gdzie nic fizycznie nie ma. Robot omija ducha.

**Przyczyna**: punkty skanu z podłogi/sufitu mimo Z slice, lub stary
TF nie czyści costmap.

**Fix opcja A**: zawęź slice w `pointcloud_to_laserscan.yaml`:
```yaml
pointcloud_to_laserscan:
  ros__parameters:
    min_height: -0.10       # zawęź wokół poziomu pelvis
    max_height: 0.20
```

**Fix opcja B**: zwiększ `raytrace_max_range` w nav2_params (costmap
agresywniej czyści):
```yaml
local_costmap:
  local_costmap:
    ros__parameters:
      obstacle_layer:
        scan:
          raytrace_max_range: 10.0   # default 3.0
          obstacle_max_range: 5.0
```

## 5.6 — Robot dojeżdża ale przejeżdża cel o >30 cm

**Objaw**: robot fizycznie zatrzymuje się znacznie za celem, ale
`bt_navigator: Goal succeeded`.

**Przyczyna**: `xy_goal_tolerance: 0.10` ale controller hamuje za późno
przez `min_approach_linear_velocity`.

**Fix**:
```yaml
goal_checker:
  xy_goal_tolerance: 0.20         # poluzuj (akceptuje 20 cm)
  yaw_goal_tolerance: 0.30

FollowPath:
  min_approach_linear_velocity: 0.05    # default 0.05, zmniejsz do 0.03
  approach_velocity_scaling_dist: 1.0   # zacznij hamować 1 m przed celem
```

---

# Topiki do monitorowania w trakcie testów

Wszystkie w **trzecim terminalu** podczas Fazy 4-5:

```bash
source /home/neo/j2s/install/setup.bash

# Live status:
ros2 topic echo /cmd_vel_nav     # controller output (przed arbiter)
ros2 topic echo /cmd_vel         # arbiter output (do bridge)
ros2 topic echo /amcl_pose       # AMCL pose update rate
ros2 topic echo /behavior_tree_log    # BT decisions

# Hz monitoring:
ros2 topic hz /scan --qos-reliability best_effort
ros2 topic hz /amcl_pose
ros2 topic hz /cmd_vel
ros2 topic hz /tf

# TF live:
ros2 run tf2_ros tf2_echo map base_link    # widoczne aktualizacje
```

W RViz **przez cały czas**:
- LaserScan `/scan` (Best Effort QoS)
- Map `/map`
- AMCL Pose `/amcl_pose`
- ParticleCloud `/particle_cloud`
- Path `/plan` + `/local_plan`
- Costmap `/local_costmap/costmap`

---

# Mini-cheatsheet dla edycji

Wszystkie zmiany w yaml + rebuild + restart:
```bash
# po edycji któregokolwiek z config files:
cd /home/neo/j2s
colcon build --symlink-install \
  --packages-select g1_courier_bringup g1_courier_safety
source install/setup.bash
# restart real.launch.py
```

Plus **symlink-install** znaczy że yaml zmiany w `src/.../config/*.yaml`
**nie** wymagają rebuild — wystarczy restart launchu. Plus bezpieczniej
zawsze rebuild po edycji, bo gwarantuje refresh.

---

# Strategia debugowania — przykład sesji

Załóżmy że robot na nav goal jedzie do przodu plus skręca w bok zamiast
prosto:

1. Faza 1 — wszystko OK (scan, lowstate, AMCL active)
2. Faza 2 — bridge cmd_vel test OK (direct vx=0.15 → idzie prosto)
3. Faza 3 — AMCL T1-T5 wszystkie ✅
4. Faza 4 — `/cmd_vel_nav` pokazuje vy=0.04 m/s, `/cmd_vel` pokazuje vy=0.12 m/s

   → **5.1 hit**: `min_vy_threshold` floor

5. Edytuj `safety.yaml`: `min_vy_threshold: 0.0`
6. Rebuild + restart
7. Powtórz Fazę 4

Jeśli teraz `/cmd_vel` ≈ `/cmd_vel_nav` plus robot jedzie prosto → ✅

Jeśli nadal coś dziwnego → wróć do logu BT (`/behavior_tree_log`) plus
patrz kolejny scenariusz 5.x.
