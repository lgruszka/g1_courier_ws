# Faza 1.5 — AMCL + nav2 z static map (real navigate_to_pose)

## Cel

Zastąpić `kinematic_nav_node` (P-controller bez planning'u) realnym nav2 stack'iem: **AMCL** lokalizuje robota na mapie z Fazy 1.4, **NavfnPlanner (A\*)** planuje globalną ścieżkę omijając biurka, **RegulatedPurePursuitController** (owinięty w `RotationShimController`) wykonuje plan publikując `/cmd_vel`. Mission BT widzi ten sam kontrakt action `/courier/navigate_to_pose` co przedtem — adapter `nav2_navigate_proxy` translatuje na nav2's `/navigate_to_pose`.

**Świadoma decyzja**: zostajemy przy kinematic mocap movement (sub-faza 1.3 deferred) — z perspektywy AMCL/nav2 ślizg jest nierozróżnialny od walkingu, oba dostarczają `/odom`.

## Co osiągnięto

| Komponent | Stan przed | Stan po |
|---|---|---|
| `phase1_full.launch.py` | nie istniał | nowy launch: nav2 stack via `bringup_launch.py` + AMCL + sim_cmd_vel_bridge (publish_map_to_odom=False) + cmd_vel_arbiter + arm/dock/retreat + mission BT |
| `nav2_params.yaml` | minimum scaffold | pełna config: AMCL OmniMotionModel + set_initial_pose, NavfnPlanner A\*, RotationShim → RegulatedPurePursuit, costmaps z obstacle_layer (2D LaserScan), stub configs dla collision_monitor + docking_server |
| `nav2_navigate_proxy` | nie istniał | adapter `g1_courier_msgs/NavigateToPose` (mission BT) → `nav2_msgs/NavigateToPose` (nav2) |
| `cmd_vel_arbiter` | publishes /cmd_vel always | parameter `enable_publish: bool` (default True). Plus subscribes `/cmd_vel_nav` od nav2 controller_server (jazzy bringup remapuje cmd_vel → cmd_vel_nav) |
| `_phase_transfer` w `mission_node.py` | 2 nav goale (`transit_via_to_*` + `navigate_to_*_predock`) — workaround dla kinematic_nav | **1 nav goal** — A\* z inflation 0.40 sam planuje detour wokół biurek |
| `waypoints.yaml` table_a | dock_mode: apriltag | dock_mode: lidar_line — return leg (B→A) carryuje paczkę z B, head_cam zasłonięty, RANSAC line fit działa parcel-independent |
| Mapa `~/maps/lab.pgm` | zawierała ~200 izolowanych szumowych pikseli z lidar self-reflection podczas mappingu | wyczyszczona connected-component filter (size<5 → free), backup w `lab.pgm.backup` |
| Pełen mission cycle (4 fazy: pickup_a + transfer_b + pickup_b + transfer_a) | działał z fake nav i fake dock | działa end-to-end z real nav2 + real dock + real arm w MuJoCo |

## Architektura — przepływ /cmd_vel + nav2 stack

```
mac MuJoCo (10.211.55.2)             Linux (10.211.55.11)
─────────────────────────             ────────────────────────
unitree_mujoco bridge                 phase1_full.launch.py
  ├─ MuJoCo physics (1 ms step)        │
  ├─ rt/scan ─────────────────────────┐│
  ├─ rt/detections ───────────────────┐│
  ├─ rt/lowstate ─────────────────────┐│
  ├─ rt/cmd_vel ◄─────────┐           ││
  └─ kinematic mocap movement (Faza 1.1)
                          │           ││
                          │     ┌─────▼▼──────────┐
                          │     │ AMCL            │
                          │     │  /map (static)   │
                          │     │  + /scan         │
                          │     │  → map→odom TF   │
                          │     └──────────────────┘
                          │     ┌─────▼────────────┐
                          │     │ NavfnPlanner A\* │
                          │     │  + global_costmap│
                          │     │  → /plan         │
                          │     └──────────────────┘
                          │     ┌─────▼────────────┐
                          │     │ RotationShim     │
                          │     │  → RegulatedPure │
                          │     │  → /cmd_vel_nav  │
                          │     └─────────┬────────┘
                          │     ┌─────────▼────────┐
                          │     │ cmd_vel_arbiter   │
                          │     │ merges:           │
                          │     │  /cmd_vel_nav     │
                          │     │  /cmd_vel_dock    │
                          │     │  /cmd_vel_retreat │
                          │     └─────────┬────────┘
                          └──────/cmd_vel─┤  +→ sim_cmd_vel_bridge (Linux /odom)
                                          │  +→ rt/cmd_vel (mac mocap movement)
                                          │
                       ┌────────────────────┐
                       │ mission_node BT    │
                       │  /courier/nav      │
                       │   ↓                │
                       │ nav2_navigate_proxy│
                       │   ↓                │
                       │ /navigate_to_pose  │
                       │   ↓                │
                       │ bt_navigator       │
                       │   ↓                │
                       │ planner + controller│
                       └────────────────────┘
```

## Workflow user'a

```bash
# Terminal 1 — mac MuJoCo (z scene plus AprilTag plus LiDAR plus parcel)
cd ~/code/unitree_mujoco/simulate_python
mjpython unitree_mujoco.py

# Terminal 2 — Linux phase1_full
cd ~/g1_courier_ws
source install/setup.bash
ros2 launch g1_courier_bringup phase1_full.launch.py
# (opcjonalnie: map:=$HOME/maps/other.yaml)

# Terminal 3 — visualizer trasy nav2 (debug)
python3 /tmp/plan_viz.py
feh --reload 1 /tmp/nav_plan.png    # lub eog /tmp/nav_plan.png + F5
```

Spodziewany przebieg jednego cyklu (~80–100 s):
1. `navigate_to_table_a` (0,0)→(0.5, 0): ~5 s, brak detouru
2. `dock_to_table_a` LIDAR_LINE: ~10 s konwergencji RANSAC
3. `pick_at_table_a`: ~17 s sekwencja P0..P6
4. `transfer_to_table_b` (0.7, 0)→(3.0, 0): ~25 s, łuk przez +Y wokół biurka A
5. `dock_to_table_b` + `place_at_table_b` + `retreat`: ~25 s
6. `pickup_at_table_b`: ~20 s nav + dock + pick
7. `transfer_to_table_a` (3.0, 0)→(0.5, 0): ~25 s, łuk przez +Y w drugą stronę
8. `dock_to_table_a` LIDAR_LINE (carry): ~10 s — paczka nie blokuje pelvis lidar
9. `place_at_table_a` + `retreat`: ~17 s

## Lessons learned

### Jazzy nav2 bringup remapuje `cmd_vel → cmd_vel_nav` — arbiter musi merge'ować

W `nav2_bringup/launch/navigation_launch.py` w jazzy linie 137, 181, 215 mają
`remappings = remappings + [('cmd_vel', 'cmd_vel_nav')]` zaaplikowane do
`controller_server`, `smoother_server`, `bt_navigator` itd. Czyli nav2
**nie publikuje `/cmd_vel` bezpośrednio** — wszystko ląduje na `/cmd_vel_nav`.

Kierunek myślenia "nav2 publishes /cmd_vel directly, dock/retreat też publish /cmd_vel directly, arbiter w trybie service-only" zawodzi: `/cmd_vel_nav` ląduje w czarnej dziurze (nikt nie subskrybuje), robot fizycznie się nie rusza mimo `controller_server: Passing new path to controller`.

**Fix**: arbiter zostaje w trybie merging, subskrybuje `/cmd_vel_nav` (nav2) + `/cmd_vel_dock` + `/cmd_vel_retreat`, publikuje `/cmd_vel`. To było jego oryginalne zadanie. Dock i retreat zostają z domyślnymi topics (`/cmd_vel_dock`, `/cmd_vel_retreat`) — nie nadpisuje się ich w launch.

Diagnostyka: `ros2 topic hz /cmd_vel` — jeśli 0 plus jednocześnie `/cmd_vel_nav` ma data, to ten bug.

### `collision_monitor` plus `docking_server` wymagają stub config

Jazzy `bringup_launch.py` **zawsze** odpala `collision_monitor` plus `docking_server` (`opennav_docking::DockingServer`) — nie ma toggle. Plus oba mają aggressive lifecycle validation:

- `collision_monitor`: bez `polygons:` plus `observation_sources:` lifecycle configure aborti z `observation_sources not initialized`
- `docking_server`: bez `dock_plugins:` aborti z `Charging dock plugins not given!`
- Pusta lista `[]` plus pusty dict `{}` w YAML rozbija parameter parser z `Expected float/int/str/bool/bytes, got tuple ()`

**Fix**: stub config z jednym disabled item odpowiedniego typu. Patrz `nav2_params.yaml` linie 188-274. Nodes konfigurują się czysto, bond z lifecycle managerem łapie się, ale sub-systemy są wyłączone — nasz `g1_courier_docking.dock_action_server` nie jest opennav_docking, plus arbiter routuje cmd_vel z dala od collision_monitor.

### Carry-mode aktywny na obu legs — oba dock muszą być parcel-independent

Mission cycle ma 4 fazy (`build_tree` w `mission_node.py`):

```
_phase_pickup(table_a)     # NO carry → dock A → pick
_phase_transfer(table_b)   # CARRY parcel z A → dock B → place + retreat
_phase_pickup(table_b)     # NO carry → dock B → pick
_phase_transfer(table_a)   # CARRY parcel z B → dock A → place + retreat
```

Obie fazy `_phase_transfer` carryują paczkę. Head_cam (czołowa kamera w torso) jest zasłonięty przez paczkę w obu przypadkach. Pierwotnie tylko `table_b` miał `dock_mode: lidar_line` (komentarz w waypoints.yaml o tym wspominał). `table_a` z `apriltag` zawodzi na return leg (B→A): APRILTAG aligner nie widzi tag id=5, oscyluje yaw szukając go, timeout 30 s plus mission BT z `memory=True` restartuje cycle od początku → nieskończona pętla.

**Fix**: oba tables na `dock_mode: lidar_line`. RANSAC pelvis-mounted 360° lidara (Faza 1.2) działa parcel-independent. Pierwsza wizyta przy table_a (no carry) też działa — lidar widzi krawędź biurka z dowolnej z dwóch sytuacji.

### Pure-pursuit oscyluje przy in-place yaw — RotationShim wrapper

`RegulatedPurePursuitController` ma `lookahead_dist` (0.6 m) — jeśli robot musi zmienić yaw o duży kąt (np. 180° po retreat), nie robi czystego in-place rotation, robi małe loopy `lookahead × velocity`. Plus pre-pursuit `regulated_linear_scaling_min_radius` jeszcze spowalnia. Efekt: robot dryptuje (xy goal w obrębie tolerance ale yaw nie) przez kilkadziesiąt sekund.

**Fix**: owinąć w `nav2_rotation_shim_controller::RotationShimController`. Kluczowe parameters:
- `angular_dist_threshold: 0.785` (45°) — jeśli path heading vs current yaw > to, rotate first
- `rotate_to_goal_heading: true` — robi też in-place rotation przy końcu trasy
- `rotate_to_heading_angular_vel: 0.8` — matche'uje arbiter `max_vyaw_normal`
- `primary_controller`: `RegulatedPurePursuitController` (przekazuje sterowanie po rotation)

Plus znane ograniczenie: nawet z RotationShim widać **resztkowy mikro-oscylacja** (małe ruchy lewo-prawo) na samym końcu trasy zanim robot ustabilizuje się przed dock. Prawdopodobnie shim oscyluje wokół `yaw_goal_tolerance` (0.35 rad) z `rotate_to_heading_angular_vel: 0.8` rad/s i 50 Hz controller frequency — dyskretne kroki velocity dają overshoot. Nie blokuje funkcjonalności (dock i tak rozpoznaje krawędź biurka), ale jest kosmetycznie nieprzyjemne. Future tuning: `rotate_to_heading_angular_vel` ↓ do 0.4 rad/s, lub `yaw_goal_tolerance` ↑ do 0.50.

### Map noise z lidar self-reflection podczas mappingu

`~/maps/lab.pgm` z Fazy 1.4 zawierało wiele pojedynczych pikseli `occupied` (wartość 0) wokół (0, 0) plus rozsiane po pomieszczeniu. To self-reflection lidara z robota podczas teleop (lidar widział własne tylne nogi, plecy, pelvis itp.). Plus inflation 0.20-0.40 wokół takich pikseli powoduje że planner uznaje start (0, 0) jako lethal — `Failed to create plan with tolerance of: 0.250000` mimo że visually mapa wygląda na pustą.

**Fix**: connected-component filter na PGM:
```python
from scipy.ndimage import label
occupied = (arr < 100).astype(np.uint8)
labels, n = label(occupied)
sizes = np.bincount(labels.flatten())
small = np.where((sizes > 0) & (sizes < 5))[0]
for lbl in small:
    arr[labels == lbl] = 254  # set to free
```

Real obstacles (ściany, biurka) tworzą connected components o setkach pikseli. Szum to izolowane 1-3-pikselowe grupy. Próg `size < 5` jest bezpieczny.

W naszym przypadku oczyszczenie usunęło 178 pikseli, plus mapy ścian plus biurek pozostały intact.

### Stripping dlaczego planner zawodzi krótkie nav (start ≈ goal)

`NavfnPlanner.tolerance: 0.50` powodował że plan z (0, 0) do (0.5, 0) zwracał empty path bo "start jest już w obrębie tolerancji od goala" — controller dostawał pustą path, próbowal jej follow'ować, robot stał. Bez logu błędu.

**Fix**: `tolerance: 0.25` (połowa najkrótszego nav distance ~0.5 m). Plus `use_astar: true` — Dijkstra w cluttered grid czasami wraca z `Failed to create plan` przy bardzo wąskich korytarzach których A\* by znalazł.

## Co zostało odłożone

- **Resztkowy mikro-oscylacja przy końcu nav** — RotationShim z domyślnymi parameters wystarcza dla mission BT, ale można dostroić (`rotate_to_heading_angular_vel` ↓ albo `yaw_goal_tolerance` ↑) jeśli kosmetyka stanie się problemem.
- **Real walking controller (sub-faza 1.3)** — nadal deferred, kinematic mocap wystarcza.
- **Multi-cycle stress test** (`max_cycles > 1`) — Faza 1.6 zweryfikuje stabilność po 10+ cyklach.
- **AMCL drift diagnostic** — w naszym sim `/odom` jest idealny, AMCL praktycznie nie ma czego korygować. Real robot będzie wymagał re-tuningu `alpha[1..5]` z ground truth z mac.
- **Visual servo accuracy benchmark** — Faza 2 (Isaac Sim cloud GPU) zwaliduje APRILTAG na foto-realistycznym oświetleniu, dla teraz wystarcza.

## Pliki dotknięte

### Linux side

| Plik | Co się zmieniło |
|---|---|
| `src/g1_courier_bringup/launch/phase1_full.launch.py` | **Nowy launch** — nav2 stack via `bringup_launch.py`, AMCL z static map, sim_cmd_vel_bridge (publish_map_to_odom=False), cmd_vel_arbiter (publish=True), nav2_navigate_proxy adapter, arm/dock/retreat, mission BT z TimerAction(5s) żeby nav2 lifecycle zdążył dojść do `active`. |
| `src/g1_courier_bringup/config/nav2_params.yaml` | **Nowy plik** — pełen config: AMCL (OmniMotionModel, set_initial_pose), NavfnPlanner (A\*, tolerance 0.25), RotationShim → RegulatedPurePursuit (yaw_goal_tolerance 0.35), local + global costmap (footprint 0.18×0.20, inflation 0.40, obstacle_layer LaserScan), stub configs dla collision_monitor + docking_server + velocity_smoother + waypoint_follower + behavior_server. |
| `src/g1_courier_sim/g1_courier_sim/nav2_navigate_proxy.py` | **Nowy adapter** — accept `g1_courier_msgs/NavigateToPose` na `/courier/navigate_to_pose`, forward jako `nav2_msgs/NavigateToPose` na `/navigate_to_pose`. Translatuje `result.error_code → success/message`. |
| `src/g1_courier_sim/g1_courier_sim/sim_cmd_vel_bridge_node.py` | (już z Fazy 1.4) parameter `publish_map_to_odom: bool` — phase1_full ustawia `False` żeby AMCL publikował map→odom. |
| `src/g1_courier_sim/setup.py` | Dodany entry point `nav2_navigate_proxy`. |
| `src/g1_courier_safety/g1_courier_safety/cmd_vel_arbiter.py` | Parameter `enable_publish: bool` (default True). Pierwotnie próbowano `False` (service-only) — nie działało bo nav2 publikuje na `/cmd_vel_nav`. Param zostawiony do future use cases. |
| `src/g1_courier_mission/g1_courier_mission/mission_node.py` | `_phase_transfer` ma teraz **1 nav goal** zamiast 2. Usunięte `_TRANSIT_VIA_*` constants — A\* z inflation 0.40 sam planuje detour. |
| `src/g1_courier_mission/config/waypoints.yaml` | `table_a.dock_mode`: apriltag → **lidar_line** (carry-occluded head_cam na return leg). Tolerancje 0.06/0.07 jak table_b. |
| `~/maps/lab.pgm` | Wyczyszczone 178 izolowanych szumowych pikseli (connected-component, size<5). Backup `lab.pgm.backup`. |
| `CLAUDE.md` | Faza 1.5 ✅ w "Stan obecny", sub-faza 1.5 description, plus 5 nowych wpisów w "Najczęstsze problemy". |

### Mac side

Brak zmian — mac bridge z Faz 1.0..1.2 (DDS publish + kinematic mocap movement + LiDAR raycasting + AprilTag detection) działa bez modyfikacji.
