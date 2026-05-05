# Faza 1.4 — Mapa LiDARem (slam_toolbox) z kinematic mocap

## Cel

Zbudować 2D occupancy grid mapę mac MuJoCo scene'u (8×6 m pomieszczenie, dwa biurka, przeszkody) na podstawie real LaserScan z mac (Faza 1.2). Mapa zostaje zapisana jako `~/maps/lab.{pgm,yaml}` plus jest podstawą dla Fazy 1.5 (AMCL + nav2 z static map).

**Świadoma decyzja**: pomijamy walking controller (sub-faza 1.3 deferred). Zamiast real chodu używamy kinematic mocap movement — robot welded ślizga się zgodnie z `/cmd_vel`. Plus z perspektywy slam_toolbox/AMCL nie ma różnicy między walking a kinematic ślizgiem — oba dostarczają `/odom` plus `/scan`, slam jest agnostic do źródła ruchu.

## Co osiągnięto

| Komponent | Stan przed | Stan po |
|---|---|---|
| `mapping.launch.py` | scaffold z `pointcloud_to_laserscan` (zakładał 3D Livox) | rewrite: sim_cmd_vel_bridge + static TF + slam_toolbox + nav2_lifecycle_manager |
| `slam_toolbox_mapping.yaml` | minimum config | rozszerzona: solver Ceres, max_laser_range 8 m, scan matcher params, loop closure params |
| `sim_cmd_vel_bridge_node` | always publishes static map→odom | parameter `publish_map_to_odom: bool` (default True; mapping launch ustawia False żeby slam_toolbox publikował map→odom) |
| Mac `rt/scan` header.stamp | sim_t (sec ≈ 335) | Linux epoch (sec ≈ 1778019xxx) — zgodne z TF buffer |
| `/map` topic | nie istniał | publikuje OccupancyGrid ~1 Hz po slam activate |
| Zapisana mapa | brak | `~/maps/lab.pgm` + `~/maps/lab.yaml` (static map dla Fazy 1.5) |

## Architektura — przepływ /scan + /map

```
mac MuJoCo (10.211.55.2)             Linux (10.211.55.11)
─────────────────────────             ────────────────────────
unitree_mujoco bridge                 mapping.launch.py
  ├─ MuJoCo physics (1 ms step)        │
  ├─ LidarScanner (Faza 1.2)           ├─ sim_cmd_vel_bridge_node
  └─ DDS publish rt/scan ──────────────┤    publish_map_to_odom: False
       (header.stamp = time.time(),    │    publishes /odom + odom→base_link TF
        Linux epoch)                   │
                                       ├─ static_transform_publisher × 2
                                       │    base_link → pelvis (identity)
                                       │    pelvis → lidar_link (offset -0.393)
                                       │
                                       ├─ slam_toolbox (lifecycle node)
                                       │    subscribe /scan
                                       │    publish /map + map→odom TF
                                       │
                                       └─ nav2_lifecycle_manager
                                            autostart: True
                                            node_names: [slam_toolbox]
                                            (auto configure + activate)

User w innym terminalu:
  teleop_twist_keyboard publishes /cmd_vel
  → sim_cmd_vel_bridge integruje + mac mocap też integruje (oba zsynchronizowane)
  → robot fizycznie się przesuwa w mac scene
  → kamera + lidar widzą zmieniającą się scenę
  → slam matche scans, builduje mapę
```

## Workflow user'a

```bash
# Terminal 1 — mac MuJoCo
cd ~/code/unitree_mujoco/simulate_python
mjpython unitree_mujoco.py

# Terminal 2 — mapping stack (Linux)
cd ~/g1_courier_ws
source install/setup.bash
ros2 launch g1_courier_bringup mapping.launch.py

# Terminal 3 — manual teleop driving
ros2 run teleop_twist_keyboard teleop_twist_keyboard
# Klawisze: i/, (forward/back), j/l (strafe), J/L (rotate), q/z (speed up/down)
# Wolno (~0.2 m/s), żeby slam zdążył dopasować skany

# Terminal 4 — sanity check + zapis mapy
ros2 topic hz /scan          # ~7 Hz z mac
ros2 topic hz /map           # ~1 Hz po pierwszym ruchu
ros2 topic echo /map --field info --once   # width/height/resolution sanity

# Po objechaniu sceny:
ros2 run nav2_map_server map_saver_cli -f ~/maps/lab
# Powstaje ~/maps/lab.pgm + ~/maps/lab.yaml

xdg-open ~/maps/lab.pgm   # weryfikacja wzrokowa
```

## Lessons learned

### slam_toolbox to lifecycle node — wymaga lifecycle managera

W ROS2 Jazzy `async_slam_toolbox_node` jest **managed** lifecycle node. Plus po starcie sam siedzi w stanie `unconfigured` — nie subskrybuje `/scan`, nie publikuje `/map`. Plus widoczne tylko `/parameter_events` plus `/slam_toolbox/transition_event` (lifecycle pubs).

Diagnostyka: `ros2 node info /slam_toolbox` — jeśli widać tylko `parameter_events` i lifecycle services, slam jest unconfigured.

**Fix**: dodać `nav2_lifecycle_manager` z `autostart: True` plus `node_names: ['slam_toolbox']` w launch'u. Plus po starcie manager auto-przejdzie nodes przez `configure → activate`.

### Mac LaserScan timestamp musi być w Linux epoch (nie sim_t)

slam_toolbox **wymaga** TF lookup dla każdego scanu (żeby projection do map frame). Plus mac initialnie publikował `header.stamp` w sim_t (sim seconds od startu unitree_mujoco, np. `sec=335`). Plus Linux TF buffer ma transforms ze stampami w epoch (`sec ≈ 1778019xxx`). Plus slam wywołuje `tf_buffer.lookupTransform(lidar_link, base_link, scan.header.stamp)` z stampem 335 — buffer odpowiada "extrapolation into the past beyond available transforms" — scan odrzucony **cicho**.

Diagnostyka:
```bash
ros2 topic echo /scan --field header.stamp --once
# jeśli sec jest mały (kilkaset), to ten problem
```

**Fix**: mac używa `time.time()` (Linux epoch) zamiast `sim_t` w `header.stamp` jedynie dla `rt/scan`. Pozostałe topics (`rt/lowstate`, `rt/detections`) zostają w sim_t — dock APRILTAG i arm_controller nie używają TF lookup.

Plus jeśli kiedyś będzie potrzeba pełnej time synchronization (multi-sensor fusion), proper rozwiązanie: mac publikuje `/clock` (`rosgraph_msgs/Clock`) z sim_t plus wszystkie Linux nodes z `use_sim_time: True`. Większy refactor — odłożone do chwili kiedy faktycznie potrzebne.

### `sim_cmd_vel_bridge_node` musi być świadomy SLAM stack'a

Default sim_cmd_vel_bridge publikuje **static** `map → odom` jako identity transform (przydatne w Phase 0 gdzie nie ma SLAM). Plus z slam_toolbox aktywnym, slam **też** publikuje `map → odom` — plus dwa publishery na ten sam transform łamią TF tree.

**Fix**: dodać parameter `publish_map_to_odom: bool` (default True dla zachowania compat). W mapping launch ustawić `False` żeby slam_toolbox był jedynym publisherem.

### Parallels VM rviz2 GLSL bug

RViz Map display crashuje w Parallels VM z błędem `active samplers with a different type refer to the same texture image unit`. Plus to **driver-level** problem virtio-GPU + RViz custom shaders. Plus `/map` topic ma wszystkie dane poprawnie — to tylko viewer crashy.

**Workarounds**:
- `LIBGL_ALWAYS_SOFTWARE=1 rviz2` — software OpenGL (1-5 FPS, ale działa)
- Zapisać mapę przez `map_saver_cli` plus obejrzeć PGM externally (`xdg-open ~/maps/lab.pgm`)

Plus dla naszego workflow zapis + external viewer wystarcza — RViz nie jest critical dla mapping pipeline.

## Co zostało odłożone

- **`/clock` + `use_sim_time: True`** dla pełnej time sync — niepotrzebne dla obecnych Linux nodes (slam, dock, arm). Plus jeśli Faza 1.5 (AMCL + nav2 + multi-sensor fusion) tego wymaga, zaimplementuję wtedy.
- **Real walking controller (sub-faza 1.3)** — nasz kinematic mocap workaround jest funkcjonalnie wystarczający dla 1.4-1.6.
- **Lokalizacja w trybie localization** (slam_toolbox z istniejącą mapą) — opcja na przyszłość, dla teraz Faza 1.5 użyje AMCL na zapisanej mapie.

## Pliki dotknięte

### Linux side

| Plik | Co się zmieniło |
|---|---|
| `src/g1_courier_sim/g1_courier_sim/sim_cmd_vel_bridge_node.py` | Nowy parameter `publish_map_to_odom: bool` (default True). Gdy False, nie publikuje static map→odom — pozwala SLAM przejąć ten transform. |
| `src/g1_courier_bringup/launch/mapping.launch.py` | Pełen rewrite: sim_cmd_vel_bridge (publish_map_to_odom=False) + 2 static TF (base_link→pelvis, pelvis→lidar_link) + slam_toolbox + nav2_lifecycle_manager. Usunięte `pointcloud_to_laserscan` (nie potrzebne — mac publikuje LaserScan bezpośrednio). |
| `src/g1_courier_bringup/config/slam_toolbox_mapping.yaml` | Rozszerzona config: solver Ceres, max_laser_range 8.0 (mac scene 8×6), scan matcher tuning, loop closure params. |
| `CLAUDE.md` | Faza 1.4 ✅ w "Stan obecny", sub-faza 1.4 description, plus 2 nowe wpisy w "Najczęstsze problemy" (lifecycle manager required, scan stamp Linux time). |

### Mac side (poza repo)

| Plik | Co się zmieniło |
|---|---|
| `~/code/unitree_mujoco/simulate_python/unitree_sdk2py_bridge.py` | `MaybeStepLidar()` używa `time.time()` zamiast `sim_t` dla `header.stamp` w `rt/scan`. Reszta topics (rt/lowstate, rt/detections) bez zmian. |
