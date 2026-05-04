# Faza 1.1 — AprilTag w MuJoCo + dock APRILTAG end-to-end

## Cel

Zastąpić `fake_dock_action_server` w `phase1_smoke.launch.py` prawdziwym `dock_action_server` w trybie MODE_APRILTAG. Robot widzi AprilTag id=5 na biurku A przez kamerę MuJoCo, servo loop publikuje `/cmd_vel_dock`, mac mocap przesuwa robota fizycznie aż do tolerancji 3 cm / 0.05 rad od targetu (0.55 m od tagu).

## Co osiągnięto

| Komponent | Stan przed | Stan po |
|---|---|---|
| AprilTag textures + biurka w mac `scene.xml` | brak | tag36h11 id5, id7 na biurkach A (1.5 m) i B (4 m), tag size 0.16 m |
| Kamera w MuJoCo | brak | `head_cam` w `torso_link`, fovy 60°, 640×480, ~10–15 Hz render |
| Detection na macu | brak | `pupil_apriltags` + DDS publish `rt/detections` jako `apriltag_msgs/AprilTagDetectionArray` |
| Linux pose extraction | broken (`det.pose` nie istnieje w `apriltag_msgs/AprilTagDetection` z ros-jazzy) | `cv2.solvePnPGeneric` + `SOLVEPNP_IPPE_SQUARE` z dwustopniową disambiguacją + flip-fallback dla degeneracji |
| Camera intrinsics na Linuxie | brak | fallback z parametru `apriltag.{fx,fy,cx,cy}` w `docking.yaml`; `CameraInfo` subscriber gotowy gdy mac kiedyś doda `rt/camera_info` |
| Mac kinematic mocap movement | brak | bridge subscribuje `rt/cmd_vel`, integruje `pelvis_anchor` `mocap_pos` i `mocap_quat` co physics step (1 ms) |
| Dock A APRILTAG end-to-end | timeout / fake | real, zbiega w ~7 s do tolerancji 3 cm / 0.05 rad |
| Mission BT pełen cycle | wykolejony na dock A | przechodzi pełny: pickup A → transfer B → pickup B → transfer A z retreat |

## Architektura — co z czym gada

```
mac MuJoCo (10.211.55.2)             Linux (10.211.55.11)
─────────────────────────             ────────────────────────
unitree_mujoco bridge                 mission_node (BT)
  ├─ MuJoCo physics (1 ms step)         │
  ├─ head_cam Renderer (10-15 Hz)       │  goal /dock_to_table
  ├─ pupil_apriltags detector  ──┐      ▼
  └─ kinematic mocap integrator  │   dock_action_server
       ↑                         │      │  publish /cmd_vel_dock
       │ rt/cmd_vel              │      ▼
       │                         │   cmd_vel_arbiter
       │                         │      │  publish /cmd_vel
       │                         │      ▼
       └─────────────────────────┴── DDS rt/cmd_vel
                                 │
                                 │  rt/detections
                                 └────────────────► dock_action_server
```

## Algorytm dokowania krok po kroku (MODE_APRILTAG)

### 1. Mission BT decyduje "dock to A"

`mission_node` ma w drzewie `DockTo("dock_to_table_a", mode=APRILTAG, tag_id=5, xy_tol=0.03, yaw_tol=0.05, timeout_s=30)`. Gdy poprzedni krok (`navigate_to_table_a`) zwrócił SUCCESS, BT przechodzi do tej pozycji i wywołuje `_ActionBehaviour.initialise()`. To buduje `DockToTable.Goal` i wysyła go przez `ActionClient.send_goal_async('/dock_to_table', goal)`.

### 2. Action goal trafia do `dock_action_server`

W `_execute()` action server bierze busy-lock (jeden goal na raz), czyta `request.mode`. Dla `MODE_APRILTAG` wchodzi w `_run_apriltag(goal_handle, request, result, deadline)`. Tworzy obiekt `AprilTagAligner` z parametrami z `docking.yaml`:

- `kp_xy = 0.6`, `kp_yaw = 1.2` — wzmocnienia P-regulatora
- `max_vx = 0.15`, `max_vy = 0.10`, `max_vyaw = 0.4` m/s — sufity prędkości
- `target_xy = 0.55` m — docelowa odległość od tagu

Pętla servo działa z częstotliwością `control_rate_hz = 20` Hz (`period = 50 ms`).

### 3. Pobranie najnowszej detekcji

Każdy tick pętli wywołuje `_extract_tag_residual(tag_id=5, goal_pose)`. Pod spodem:

- z `_tag_lock` czyta `self._latest_tags` (zapisywane przez callback `_on_tag` z subskrypcji `rt/detections`)
- iteruje `msg.detections`, szuka tej z `id == 5`
- wyciąga 4 punkty `corners` (piksele w obrazie)

Jeśli detekcji brak (lub tag5 nie w polu widzenia) — zwraca `(None, None, None)`, aligner publikuje `Twist()` (zero) i pętla idzie dalej.

### 4. PnP — pozycja i orientacja tagu względem kamery

Trzy macierze wchodzą do `cv2.solvePnPGeneric`:

**`obj_pts`** (4×3) — gdzie cztery rogi tagu znajdują się w **lokalnym układzie tagu** (X w prawo, Y w górę, Z prostopadle z płaszczyzny):

```
(-s,  s, 0)   TL          s = tag_size_m / 2 = 0.08 m
( s,  s, 0)   TR
( s, -s, 0)   BR
(-s, -s, 0)   BL
```

**`img_pts`** (4×2) — te same rogi, ale w pikselach obrazu. `pupil_apriltags` zwraca corners w kolejności CCW od lewego górnego rogu obrazu, więc reorderujemy `[corners[3], corners[2], corners[1], corners[0]]` żeby pasowało do konwencji IPPE_SQUARE.

**`K`** (3×3) — intrinsics kamery (z fallback: `fx = fy = 415.69`, `cx = 320`, `cy = 240`, dla fovy=60° i 640×480).

`solvePnPGeneric` ze flagą `SOLVEPNP_IPPE_SQUARE` rozwiązuje równanie projekcji 2D→3D — dla każdego piksela wie, że jest projekcją konkretnego rogu w lokalnej tag-frame. Zwraca dwa kandydaty `(rvec, tvec)` (planar markery mają 2-krotną dwuznaczność dla normal: tag może "patrzeć na kamerę" lub "od kamery" — oba dają tę samą projekcję pikselową).

### 5. Disambiguacja kandydatów

Tag fizyczny wisi na biurku stroną z teksturą do robota — czyli **lokalny +Z tagu wskazuje w stronę kamery**, czyli w przybliżeniu w kierunku `-Z_camera`. Matematycznie: `R[2,2] < 0`, gdzie `R[:, 2]` to lokalna oś Z tagu wyrażona w układzie kamery.

Pętla po kandydatach: weź pierwszy z `R[2,2] < 0`. Jeśli żaden nie pasuje (degeneracyjny przypadek idealnego facing-on, gdzie oba kandydaci mają `R[2,2] ≈ +1`), bierzemy pierwszego i ręcznie obracamy go o 180° wokół tag's X: `R = R @ diag(1, -1, -1)`. To wymusza `R[2,2] < 0`.

Po tym kroku mamy `tvec = (x_c, y_c, z_c)` — pozycję środka tagu w **OpenCV optical frame** (X w prawo, Y w dół, Z do przodu kamery).

### 6. Mapowanie do układu robota (REP-103)

OpenCV optical frame ≠ ROS robot frame. W naszej geometrii kamera jest sztywno zamontowana w torsie i patrzy do przodu, więc mapowanie redukuje się do permutacji osi:

```
robot X (do przodu) ←→ camera Z (forward)         dx_robot = z_c - target_distance
robot Y (w lewo)    ←→ -camera X (right negated)  dy_robot = -x_c
robot yaw (CCW)     ←→ rotacja wokół -camera Y    dyaw = atan2(R[0,2], -R[2,2])
```

Dla typowego stanu początkowego (tag czołowo na 1.24 m, środek obrazu): `tvec = (0, 0.13, 1.24)`, `R[2,2] = -1`, `R[0,2] = 0`. Stąd:

- `dx_robot = 1.24 - 0.55 = +0.69 m` → robot ma jechać do przodu o 0.69 m
- `dy_robot = -0 ≈ 0` → bez przesunięcia bocznego
- `dyaw ≈ 0` → bez korekty kąta

Znak `dyaw` jest tu krytyczny. Konwencja: **dodatnie `dyaw` znaczy "robot ma się obrócić CCW"**. Jeśli mac mocap zacznie dryfować w yaw o `+θ` (CCW), tag w nowym układzie kamery ma `R[2,2] = -cos θ`, `R[0,2] = -sin θ`, więc `dyaw = atan2(-sin θ, cos θ) = -θ` — ujemne, czyli korekta CW, czyli powrót do poprawnej orientacji.

### 7. P-regulator w aligner

`AprilTagAligner.step(dx, dy, dyaw)` produkuje `Twist`:

```python
cmd.linear.x  = clamp(kp_xy  * dx,   -max_vx,   +max_vx)
cmd.linear.y  = clamp(kp_xy  * dy,   -max_vy,   +max_vy)
cmd.angular.z = clamp(kp_yaw * dyaw, -max_vyaw, +max_vyaw)
```

Dla naszego stanu początkowego: `linear.x = 0.6 * 0.69 = 0.41 → clamp do 0.15 m/s`. To jest "robot ma jechać do przodu z prędkością 0.15 m/s".

Plus aligner zwraca też `AlignError(xy_m = sqrt(dx² + dy²), yaw_rad = |dyaw|)` — to idzie do feedback BT.

### 8. Publish i routing przez arbiter

`dock_action_server` woła `self._cmd_pub.publish(cmd)` na topic `/cmd_vel_dock`.

`cmd_vel_arbiter` ma tę topicę zasubskrybowaną. Co tick (50 ms) arbiter w `_select()` sprawdza priorytety: e-stop > freeze > **dock** > retreat > nav. Jeśli `_cmd_vel_dock` ma świeży komunikat (młodszy niż `cmd_timeout_s = 0.4 s`), arbiter używa go. Aplikuje sufity (`max_vx_normal = 0.6 m/s` w trybie bez parcela) i publikuje na `/cmd_vel`.

DDS bridge między Linuxem a Macem przekazuje `/cmd_vel` jako `rt/cmd_vel` na drugą stronę.

### 9. Mac integruje pozycję mocap

Bridge na macu w callback `_on_cmd_vel` zapisuje `(vx, vy, vyaw)` w robot body frame. W głównej pętli physics (przed każdym `mj_step`, `dt = 0.001 s`):

```
yaw   += vyaw * dt
x_w   += (cos(yaw) * vx - sin(yaw) * vy) * dt
y_w   += (sin(yaw) * vx + cos(yaw) * vy) * dt
mocap_pos[pelvis_anchor]  = (x_w, y_w, 0.793)
mocap_quat[pelvis_anchor] = quat_z(yaw)
```

MuJoCo widzi że `pelvis_anchor` (mocap body) zmieniło pozycję, plus weld pin "ciągnie" całe ciało robota razem z anchorem. Robot "ślizga się" po podłodze.

Camera `head_cam` jest sztywno zamontowana w `torso_link`, więc renderowany obraz **automatycznie** pokazuje tag z bliższej odległości po następnym ticku rendera.

### 10. Pętla domyka się

Następny tick `dock_action_server` (50 ms później):

- mac wyrenderował nowy obraz z bliższej odległości
- `pupil_apriltags` znalazło tag5 (corners są dalej od centrum, bo tag jest większy w obrazie)
- Bridge publikuje nowe `rt/detections` z aktualnymi corners
- Linux dock dostał świeży `_latest_tags`
- `_extract_tag_residual` zwraca nowy `dx_robot ≈ 0.61 m` (zmalał z 0.69)
- Aligner publikuje nowy cmd_vel, ale wciąż clamped do 0.15 m/s

I tak co tick, aż `dx_robot` spadnie poniżej tolerancji.

### 11. Detekcja konwergencji

Po wyliczeniu `err` aligner sprawdza:

```python
if err.xy_m <= request.xy_tolerance_m  and  err.yaw_rad <= request.yaw_tolerance_rad:
    within_tol_count += 1
    if within_tol_count >= settle_samples:  # 5 consecutive
        # SUCCESS
else:
    within_tol_count = 0
```

`settle_samples = 5` znaczy: pięć kolejnych ticków (5 × 50 ms = 250 ms) musi być w tolerancji. To filtruje przypadki gdy robot przeskoczy target i wróci — pojedyncze "cudowne" trafienie nie liczy się jako konwergencja.

W naszym scenariuszu: `dx_robot` maleje liniowo (clamped 0.15 m/s, więc 7.5 mm na tick). Po przejechaniu od 0.69 m do 0.03 m mija ~ (0.66 / 0.15) = 4.4 s. Plus 250 ms settle. Cały dock A w realnych logach: ~7.5 s (różnica to czas na rozkręcenie mocap'u na początku plus zwalnianie P-regulatora gdy `dx` jest mały).

### 12. Result wraca do mission BT

```python
result.success = True
result.message = 'apriltag dock converged'
result.final_xy_error_m = err.xy_m
result.final_yaw_error_rad = err.yaw_rad
goal_handle.succeed()
```

`_publish_zero()` w `finally` upewnia się że robot zatrzymuje się (zero cmd_vel). Mission BT dostaje SUCCESS i przechodzi do następnej pozycji w sekwencji (`Pick`).

### Skrót różnic dla pozostałych trybów

- **MODE_LIDAR_LINE**: zamiast PnP z corners robi `LidarLineAligner.step(scan)` — RANSAC dopasowuje prostą do skanu LaserScan, wyznacza prostopadłą odległość i kąt do tej linii. Reszta (clamp, publish, arbiter, mocap) identyczna.
- **MODE_AMCL_ONLY**: trywialne — `_run_amcl_only()` publikuje zero cmd_vel i natychmiast zwraca `success=True`. Używane gdy nav2+AMCL już dowiozły robota wystarczająco blisko.

## Lessons learned

### IDL deserialization mismatch między macem a Linuxem

W pierwszej iteracji mac side wygenerował Python bindings dla `apriltag_msgs/AprilTagDetection` z ręcznie skopiowanego `.idl`, ale kolejność pól lub typ `Point` (float32 vs float64) nie zgadzały się z ros-jazzy `apriltag_msgs`. Na Linuxie `ros2 topic echo` i `rclpy` subscriber dostawały **deterministyczne** dane, ale `corners[0].x = 0.0`, `homography[8] = 25.79` (zamiast `≈1.0`) — kompletnie nieczytelne. Rozwiązanie: mac side regenerował IDL ze świeżego `/opt/ros/jazzy/share/apriltag_msgs/msg/*.idl` skopiowanego przez Parallels Shared Folders, plus zachował konwencję `Point.x/y` jako `float64`. Po fix wartości natychmiast stały się sensible.

**Diagnostyka**: gdy widzisz `centre` z sensownymi wartościami (np. (320, 240)) ale corners z dziwnymi wartościami zawierającymi zera lub niepasującymi do pixel range — to mismatch field offset. Type hash warningi `Failed to parse type hash for topic 'rt/detections' ... USER_DATA '(null)'` są **kosmetyczne** i nie odróżniają zdrowej deserializacji od popsutej.

### IPPE_SQUARE 2-fold ambiguity dla planar markerów

`SOLVEPNP_IPPE_SQUARE` zwraca dwa kandydaty pose — odpowiadające dwóm interpretacjom normalnej tagu (do kamery / od kamery). W przypadku **idealnego facing-on** (tag czołowo na osi optycznej) **oba kandydaci** mają `R[2,2] ≈ +1` (tag "patrzy od kamery") — naturalne z perspektywy degeneracji projekcji. Wtedy disambiguacja przez "weź ten z `R[2,2] < 0`" zawodzi.

Rozwiązanie: jeśli żaden kandydat nie pasuje, weź pierwszego i zastosuj manualny flip o 180° wokół tag's X: `R = R @ diag(1, -1, -1)`. To odwraca normal i zachowuje tvec. Po tym kroku `R[2,2]` jest na pewno `< 0`.

### Sign convention `dyaw` musi pasować do aligner intent

Pierwsza wersja formuły zwracała `dyaw = atan2(-R[0,2], -R[2,2])`. To daje "kąt tagu w układzie kamery" — czyli **kierunek przesuniecia normal'a**. Ale aligner robi `angular.z = kp_yaw * dyaw`, czyli oczekuje **kierunku korekty kąta robota** (CCW positive).

Te dwie semantyki mają **przeciwny znak** dla scenariusza "robot dryfuje CCW i trzeba go skorygować CW". Stara formuła wzmacniała każdą perturbację zamiast tłumić — robot ustawiał się "pod skosem" i timeoutował.

Fix: `dyaw = atan2(R[0,2], -R[2,2])` (oba znaki flipped). Test standalone z czterema scenariuszami (facing-on, drift CCW, drift CW, tag rotated) potwierdził poprawne znaki.

**Lekcja generalna**: matematyka pose extraction w optical frame jest bezdyskusyjna, ale przekład na "control law residual" wymaga dokładnego śledzenia konwencji znaków. Najprostsze testy: standalone PnP na zsyntetyzowanych corners z znanym ground-truth + sprawdzenie 4 scenariuszy (facing-on, drift L/R, tag rotated L/R).

### Tag size na obu stronach musi być zsynchronizowany

Mac użył `tag plate size = 0.16 m`, mój Linux config zakładał `0.15 m`. Skutek: PnP zwracał błędną odległość (skalowanie). Niewielka rozbieżność (0.16 vs 0.15 = 6.7%), ale propaguje się przez cały servo loop. Synchronizacja przez `apriltag.tag_size_m` w `docking.yaml`.

## Co zostało odłożone

- **Real nav A↔B**: `fake_navigate_proxy` jest instant, robot fizycznie zostaje w okolicy biurka A. To bramka dla testowania dock B prawdziwie z mac kamerą. Najprostszy fix: kinematic nav node z prostym P-controllerem do target waypoint, publikujący `/cmd_vel`.
- **Dock B real (LIDAR_LINE)**: wymaga LiDAR sensora w mac scene XML (Faza 1.2). Aktualnie `dock_mode: amcl_only` w `waypoints.yaml` jako trywialny passthrough dla diagnostycznego cyklu.
- **Drift Linux TF vs mac mocap**: oba integratory startują z (0,0,0) i konsumują ten sam `/cmd_vel`, ale różne `dt` i kolejność ticków może spowodować rozjazd po wielu cyklach. Niewidoczne w obecnej Fazie 1.1, dopadnie w 1.5 (gdy AMCL będzie konsumował Linux TF).
- **`rt/camera_info` z mac side**: aktualnie nie publikowane, używamy fallback intrinsics z config. Przy zmianie kamery (innej fovy lub rozdzielczości) trzeba by aktualizować yaml. Production: mac powinien publikować latched CameraInfo raz przy starcie.

## Pliki dotknięte

### Linux side

| Plik | Co się zmieniło |
|---|---|
| `src/g1_courier_docking/g1_courier_docking/dock_action_server.py` | Pełny refactor `_extract_tag_residual` z `cv2.solvePnPGeneric`. Subskrypcja `CameraInfo` z `RELIABLE+TRANSIENT_LOCAL`. Helper `_seed_intrinsics_from_params` dla fallback. Sign fix dla `dyaw`. |
| `src/g1_courier_docking/config/docking.yaml` | `apriltag.tag_size_m: 0.16`, `camera_info_topic: /camera_info`, `apriltag.{fx,fy,cx,cy}` jako fallback dla intrinsics. |
| `src/g1_courier_docking/package.xml` | `python3-opencv`, `python3-numpy` exec_depend. |
| `src/g1_courier_bringup/launch/phase1_smoke.launch.py` | Real `dock_action_server` zamiast `_fake('fake_dock_action_server')`. Plus `sim_lidar_publisher_node` dla biurka B (fixture). |
| `src/g1_courier_mission/config/waypoints.yaml` | `dock_mode: amcl_only` dla biurka B (diagnostyczny passthrough do czasu Fazy 1.2). |

### Mac side (poza repo)

| Plik | Co się zmieniło |
|---|---|
| `~/code/unitree_mujoco/unitree_robots/g1/scene.xml` | Biurka A (1.5 m) i B (4 m) jako `<body>` z `<geom box>`. AprilTag tag36h11 id5 i id7 jako `<geom>` z teksturą. Pelvis weld pin (z Fazy 1.0). |
| `~/code/unitree_mujoco/unitree_robots/g1/g1_29dof.xml` | `<camera name="head_cam">` w torsie z fovy=60°. |
| `~/code/unitree_mujoco/unitree_robots/g1/tags/tag36h11_id*.png` | Wygenerowane PNG-i z `moms-apriltag` lub `pupil-apriltags`. |
| `~/code/unitree_mujoco/simulate_python/unitree_sdk2py_bridge.py` | Render kamery (`mujoco.Renderer`), detector `pupil_apriltags`, DDS publisher `rt/detections`. Subscriber `rt/cmd_vel`, integrator mocap `pelvis_anchor` (kinematic mocap movement). |
| `~/code/idl_local/` | Wygenerowane Python bindings dla `apriltag_msgs/AprilTagDetectionArray`, `geometry_msgs/Twist`, `geometry_msgs/Vector3` (z ros-jazzy IDL przez `idlc -l py`). |
