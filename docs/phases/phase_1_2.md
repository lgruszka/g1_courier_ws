# Faza 1.2 — Real LiDAR sensor w MuJoCo + dock LIDAR_LINE end-to-end

## Cel

W Fazie 1.1+ dock B działał w trybie `amcl_only` (trywialny passthrough), bo nie mieliśmy real LiDAR'a w mac scene'ie. Plus `sim_lidar_publisher_node` po Linux side generował syntetyczny scan oparty na hardcoded geometrii biurka — niezsynchronizowany z mac MuJoCo physical scene.

Faza 1.2: real 2D LaserScan jako sensor w mac scene XML (programmatic ray-cast przez `mj_ray()`), bridge eksportuje przez DDS na `rt/scan`. Plus po fixie:
- `sim_lidar_publisher_node` retired (zostaje wyłączony w launchu)
- Linux `dock_action_server` MODE_LIDAR_LINE konsumuje real `/scan` z mac
- Mission BT cofa dock B do `dock_mode: lidar_line` (zamiast diagnostic `amcl_only`)
- Pełen mission BT cycle z **realnymi** dock APRILTAG (biurko A) i dock LIDAR_LINE (biurko B)

## Co osiągnięto

| Komponent | Stan przed | Stan po |
|---|---|---|
| LiDAR sensor w mac scene XML | brak | `lidar_site` w pelvis body (g1_29dof.xml) z pos `(0, 0, -0.393)` → world `(0, 0, 0.4)` |
| LaserScan publisher po macu | brak | `LidarScanner` klasa w `unitree_sdk2py_bridge.py` — 360 promieni co 1° przez `mj_ray()`, throttled @ 10 Hz sim time |
| IDL `sensor_msgs/LaserScan` po macu | brak | Hand-written w `~/code/idl_local/sensor_msgs/msg/dds_/_LaserScan_.py` (bez idlc gen, reuse `Header_` i `Time_` z unitree_sdk2py) |
| Static TF `pelvis → lidar_link` | brak | `static_transform_publisher` w `phase1_smoke.launch.py` z translacją `(0, 0, -0.393)` |
| `sim_lidar_publisher_node` w launch | aktywny (idle) | retired |
| Linux `LidarLineAligner` filtr forward window | broken dla mac scan convention (widzi tylko prawą połowę) | wrap `a → a - 2π` jeśli `a > π`, signed comparison przeciw `[-π/6, +π/6]` |
| Linux `LidarLineAligner` yaw correction | znak `-kp * yaw_err` wzmacniał drift | znak `+kp * yaw_err` tłumi |
| `dock_action_server` MODE_LIDAR_LINE end-to-end | timeout (`amcl_only` workaround) | konwerguje ~12 s do tolerancji 6 cm / 0.07 rad na biurko B |
| Mission BT pełen cycle | dock B trywialny | pełen real cycle: APRILTAG (A) + LIDAR_LINE (B) z fizycznym ruchem |

## Architektura — przepływ /scan

```
mac MuJoCo (10.211.55.2)             Linux (10.211.55.11)
─────────────────────────             ────────────────────────
unitree_mujoco bridge                 dock_action_server
  ├─ MuJoCo physics (1 ms step)         │
  ├─ LidarScanner (360 mj_ray's @ 10Hz)│  goal /dock_to_table mode=LIDAR_LINE
  └─ DDS publish rt/scan ──────────────► subscribe /scan
       │ (sensor_msgs/LaserScan)         │
       │  frame_id="lidar_link"          ▼
       │  angles 0..2π                LidarLineAligner.step(scan):
       │  ranges[360]                    1. _scan_to_points (forward window ±30°)
       │  10 Hz                          2. RANSAC line fit
       └─────────────────────────►       3. yaw_err = atan2(b, a)
                                         4. cmd.linear.x = kp_xy * (dist - 0.55)
                                         5. cmd.angular.z = kp_yaw * yaw_err
                                            │
                                            ▼
                                       publish /cmd_vel_dock
                                            │
                                       cmd_vel_arbiter → /cmd_vel
                                            │
                                       mac mocap integruje, fizyczny ruch
```

## Algorytm LIDAR_LINE krok po kroku

### 1. Subscribe i fresh data check

`dock_action_server` w `_run_lidar`:
- Subskrybuje `/scan` (`sensor_msgs/LaserScan`) z `qos_profile_sensor_data`
- Każdy tick (50 ms = 20 Hz) w pętli czyta `self._latest_scan` (stale data OK — mac publikuje 10 Hz, dock loop 20 Hz, używa najświeższy)

### 2. _scan_to_points — filtr i konwersja na (x, y) w robot frame

Dla każdego promienia w `scan.ranges`:
1. Wylicz aktualny kąt `a` (od `scan.angle_min`, plus `i * scan.angle_increment`)
2. **Wrap unsigned 0..2π do signed -π..π** (mac convention vs aligner forward window expectation):
   ```python
   a_signed = a if a <= math.pi else a - 2.0 * math.pi
   ```
3. Filter:
   - `math.isfinite(r)` (drop NaN/inf)
   - `lo_r < r < hi_r` (range window, default 0.3..2.5 m)
   - `self.angle_min <= a_signed <= self.angle_max` (forward window ±π/6 = ±30°)
4. Konwertuj na `(x, y) = (r * cos(a_signed), r * sin(a_signed))`

Dla biurka B na world `(4.0, 0)` widzianego z robota na `(2.96, 0.01)` po zsynchronizowanym mac mocap — front face na ~0.64 m forward, points concentrate w ±30° forward window, ~60 valid points przed RANSAC.

### 3. RANSAC line fit

Standardowy 2-point RANSAC, 80 iteracji, threshold inlier 0.02 m, minimum 8 inliers. Zwraca line w postaci `(a, b, c)` taką że `a*x + b*y + c = 0` plus `(a, b)` to unit normal.

### 4. Disambiguacja kierunku normal

`(a, b)` z RANSAC może wskazywać "od linii do origin" lub "od origin do linii". Konwencja aligner: `(a, b)` punktuje **OD origin DO linii**, więc `c < 0` (origin na "minus" stronie normal'a). Plus jeśli `c > 0`, flip: `a, b, c = -a, -b, -c`. Po flip `distance_to_line = -c`.

### 5. Compute residual w robot frame

```python
dx = distance_to_line - self.target_distance      # forward error
yaw_err = math.atan2(b, a)                         # heading error
```

Dla biurka B prosto przed robotem (linia perpendicular do robot's +X), `(a, b) = (1, 0)` plus `yaw_err = 0`. Plus dla biurka rotated o `θ` względem robot frame, `(a, b) = (cos θ, sin θ)` plus `yaw_err = θ`.

### 6. P-regulator

```python
cmd.linear.x = clamp(kp_xy * dx, -max_vx, max_vx)   # forward correction
cmd.angular.z = clamp(kp_yaw * yaw_err, -max_vyaw, max_vyaw)  # rotation correction
```

Kluczowa konwencja znaku: `cmd.angular.z` ma **ten sam znak** co `yaw_err` żeby tłumić drift (gdy line normal jest "po prawej", yaw_err < 0 → cmd CW negative → robot rotuje CW → line normal wraca do osi forward → yaw_err → 0). Plus brak minusa w mnożeniu.

### 7. Settle detection

`xy_err = abs(dx)` plus `yaw_err = abs(atan2(b, a))`. Plus jeśli oba pod tolerancją 5 ticków pod rząd (250 ms) → sukces.

## Lessons learned

### Mac scan convention (0..2π) vs aligner forward window (-π..π)

`LidarLineAligner` był pierwotnie napisany dla typowego ROS LaserScan z `angle_min=-π`, `angle_max=+π` (signed). Plus mac publikuje **unsigned** 0..2π (mathematical convention). Plus filter `self.angle_min <= a <= self.angle_max` (gdzie `angle_min=-π/6`) match'uje tylko `a ∈ [0, π/6]` z mac scan — czyli right half forward window. Plus aligner widzi 30 promieni zamiast oczekiwanych 60, line fit asymmetric, RANSAC niestabilny.

**Fix**: zwrócić `a_signed = a - 2π` jeśli `a > π`. Plus po wrap aligner widzi pełny ±30° forward window.

**Diagnostyka**: gdy aligner publikuje ~zero cmd_vel mimo że robot fizycznie nie konwerguje, to jest typowy objaw "za mało points dla RANSAC plus line fit fluktuuje". Plus dwa testy: (a) ile points przepuszcza filter (`len(pts)`), (b) czy aligner publikuje `searching` feedback (RANSAC fail).

### LIDAR_LINE yaw_err sign — to samo co APRILTAG dyaw

W Fazie 1.1 fixowaliśmy znak `dyaw` w APRILTAG aligner — tutaj **identyczny** problem w LIDAR_LINE. Plus `cmd.angular.z = -kp_yaw * yaw_err` wzmacnia każdą perturbację: gdy yaw_err<0 (linia "po prawej"), `cmd CCW positive` → mac integruje yaw → robot rotuje CCW → linia jeszcze bardziej w prawo → yaw_err jeszcze bardziej negative → runaway.

**Fix**: `cmd.angular.z = +kp_yaw * yaw_err` (znak ten sam co yaw_err żeby tłumić). Plus weryfikacja przez log: `cur_yaw` powinien dryfować w **przeciwnym** kierunku do `eyaw` plus konwergować do 0.

### Mac sticky kinematic_mode + scan rate niezależny od dock loop rate

Mac scan publikuje 7.2 Hz wall-clock (sim biega 72% realtime z renderem head_cam + raycastem + mocap drive). Plus dock loop 20 Hz. Plus dock używa `_latest_scan` (stale data nie reset), więc 1.5x różnica rate'u nie blokuje. Plus 100-150 ms latency pomiędzy fresh scan a dock cmd jest tolerowalna.

### LaserScan IDL hand-write (bez idlc)

Mac strona zrobiła `~/code/idl_local/sensor_msgs/msg/dds_/_LaserScan_.py` ręcznie zamiast `idlc -l py LaserScan.idl` plus reuse `Header_` plus `Time_` z `unitree_sdk2py`. Plus dziwne field (CDR padding lub float32 vs float64) byłyby trudne do złapania bez exhaustive testu, ale w tym wypadku pole-by-pole kontrola dała wire-bit-compatible result. Plus wartości z 4-ray test po macu zgadzały się z Linux subscriber'em w pierwszym podejściu.

## Co zostało odłożone

- **Self-hits przez nogi przy `lidar_site` w pelvis** — promieni szukające w kierunku ±90° (ku bokom) trafiają w nogi w odległości <0.10 m → mac filter `range_min=0.10` zwraca `range_max+1` jako "discard" marker. Plus aligner filter range `0.3..2.5` ignoruje te promienie. Plus jeśli kiedykolwiek RANSAC ma problem z brakami w 360° skanie (np. dla mapowania całego pomieszczenia), opcja: przesunąć `lidar_site` do `torso_link` body (wyżej w robocie, tylko ramiona zwisają).
- **Mac scan rate 7.2 Hz** — ograniczone przez sim CPU load. Plus jeśli kiedyś chcielibyśmy szybszy dock (lub mapowanie LiDAR'em w Fazie 1.4), opcje: zmniejszyć `LIDAR_NUM_RAYS` (np. 180 zamiast 360), rate-limit head_cam render, lub speedup mac side workers.
- **Filtered scan dla obstacle layer** — w przyszłej Fazie 1.5 (real Nav2 + AMCL + costmap) potrzebny będzie filter scan dla self-body removal plus floor/ceiling.

## Pliki dotknięte

### Linux side

| Plik | Co się zmieniło |
|---|---|
| `src/g1_courier_docking/g1_courier_docking/dock_action_server.py` | `LidarLineAligner.step` — flip znaku `cmd.angular.z = +kp * yaw_err`. `_scan_to_points` — wrap unsigned `a` do signed `a_signed` przed angle filter. |
| `src/g1_courier_bringup/launch/phase1_smoke.launch.py` | Usunięty `sim_lidar_publisher_node` Node. Dodany `static_transform_publisher pelvis → lidar_link` z translacją `(0, 0, -0.393)`. Header docstring aktualizacja. |
| `src/g1_courier_mission/config/waypoints.yaml` | `table_b.dock_mode: amcl_only → lidar_line`. Komentarz o RANSAC line fit. |

### Mac side (poza repo)

| Plik | Co się zmieniło |
|---|---|
| `~/code/unitree_mujoco/unitree_robots/g1/g1_29dof.xml` | `<site name="lidar_site">` w pelvis body z pos `(0, 0, -0.393)`. |
| `~/code/idl_local/sensor_msgs/msg/dds_/_LaserScan_.py` | Hand-written IDL (bez idlc), reuse Header/Time z unitree_sdk2py. |
| `~/code/unitree_mujoco/simulate_python/unitree_sdk2py_bridge.py` | Klasa `LidarScanner` (mj_ray × 360), `MaybeStepLidar()` w głównej pętli physics z 10 Hz throttling przez sim_t, ChannelPublisher `rt/scan`. |
