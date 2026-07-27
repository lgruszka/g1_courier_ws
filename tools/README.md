# Narzędzia diagnostyczne

Standalone helpery Pythona subskrybujące topiki ROS2. Działają
identycznie z sim i z realnym robotem.

## `cam_viewer.py`

Live preview `head_cam` z bbox AprilTagów i overlay'em dystansu z PnP.

```bash
source ~/g1_courier_ws/install/setup.bash
python3 tools/cam_viewer.py
```

Subskrybuje:
- `/head_cam/image_raw` (sensor_msgs/Image, rgb8)
- `/detections` (apriltag_msgs/AprilTagDetectionArray)
- `/camera_info` (sensor_msgs/CameraInfo, transient_local)

Per detekcja uruchamia `cv2.solvePnPGeneric` (ten sam algorytm co w
`dock_action_server`) i wyświetla `z_c` (głębokość wzdłuż osi optycznej),
pełen dystans 3D, plus offset `(x, y)` w cam frame.

Klawisze: `q` quit, `s` snapshot do `/tmp/cam_snap.png`.

## `lidar_viewer.py`

Live top-down view 2D z `/scan` z RANSAC line fit (dopasowany do
algorytmu `dock_action_server.LidarLineAligner`).

```bash
python3 tools/lidar_viewer.py
```

Wizualizuje:
- Wszystkie punkty scanu (szary = poza forward window, niebieski =
  w stożku ±30° aligner'a, czerwony = inliers RANSAC)
- Stożek forward window (`angle_min .. angle_max`)
- Linia `target_distance` (gdzie zbiega dock_to_table)
- Linia RANSAC fit (zielona) — co dock_action_server widzi jako
  "krawędź biurka"

Klawisze: `q` quit, `s` snapshot do `/tmp/lidar_snap.png`.

## `plan_viz.py`

Renderuje globalny plan nav2 + AMCL pose + inflation costmapy do
`/tmp/nav_plan.png` przy każdym update'cie planu. Używaj z viewerem
auto-reload:

```bash
python3 tools/plan_viz.py &
feh --reload 1 /tmp/nav_plan.png   # albo eog + ręczne F5
```

Subskrybuje `/map`, `/global_costmap/costmap`, `/plan`, `/amcl_pose`.

## `run_mapping_session.sh`

End-to-end orchestrator FAST-LIO mapping. Odpala FAST-LIO, czeka aż
przejedziesz scenę + Enter, wywołuje `/map_save`, generuje wiele
wariantów PGM, otwiera picker GUI.

```bash
# T1 — Livox driver (osobno):
ros2 launch livox_ros_driver2 msg_MID360_launch.py

# T2 — orchestrator:
./tools/run_mapping_session.sh
```

Env vars: `PCD_OUT`, `SCENARIOS_DIR`, `FLIP_Y` (1 dla Mid-360 upside-down).

## `pcd_variant_grid.py`

Batch generator wariantów mapy z jednego PCD (~30 kombinacji
Z-slice × resolution × min_points). Tworzy `manifest.json` z metadanymi
do `map_picker.py`.

```bash
python3 tools/pcd_variant_grid.py ~/maps/last_session.pcd ~/maps/scenarios --flip-y
```

## `map_picker.py`

PyQt5 GUI: lista wariantów + preview PGM + Save as production
(`~/maps/lab.yaml` + `lab.pgm` — odbierane przez `real.launch.py`).

```bash
python3 tools/map_picker.py ~/maps/scenarios
```

Wymaga `manifest.json` (generowane przez `pcd_variant_grid.py`).

## `scan_height_tuner.py`

**Live tuner** dla `pointcloud_to_laserscan` `min_height/max_height`.
Zmiana wartości leci do node'a przez `ros2 param set` — widoczna
**natychmiast** w RViz LaserScan display. Bez restartu launchu.

```bash
# T1: real.launch.py odpalone
# T2: rviz2 z LaserScan /scan
# T3: tuner:
python3 tools/scan_height_tuner.py
```

Slidery + 5 preset buttons + status (Hz, valid pts %). Po znalezieniu
dobrego slice → **Save to yaml**.

Wymaga: `python3-pyqt5 python3-yaml`.

## `standstill_diag.py` — dryf odometrii na POSTOJU

Mierzy równolegle trzy tory przy **stojącym** robocie i porównuje je:
`/dog_odom` (surowa odometria firmware) vs TF `odom→base_footprint`
(po `trans_scale` i ZUPT) vs `/amcl_pose` (pozycja na mapie).

```bash
python3 tools/standstill_diag.py 60      # okno 60 s, robot MUSI STAC
```

Do czego służy: firmware bipeda **całkuje kinematykę nóg także gdy robot stoi**
(zmierzone ~2.6 mm/s), co wleczyło AMCL i pozycja uciekała z mapy. Ta sonda
rozdziela winę: czy pełza źródło, czy dokłada nasz relay, czy gubi AMCL.
Raportuje też **rozpiętość kołysania** (peak-to-peak) — bez tego łatwo obwinić
sway, który jest oscylacyjny i się znosi.

Wartość progu `odom_still_dist` (ZUPT) dobiera się z tego pomiaru: ~3× szum
stania, ~4× mniej niż dystans najwolniejszego chodu w oknie 1 s.
Kontekst i liczby: `docs/etap_a_utwardzenie_nav.md` §4.1.

## `scale_verdict.py` — skala odometrii POD RUCHEM

Startuje sam, gdy pojawi się ruch, i porównuje **trzy tory jednocześnie**:
`/dog_odom` (co zgłasza firmware) vs TF `odom→base_footprint` (co widzi AMCL jako
ruch) vs `/amcl_pose` (gdzie robot faktycznie jest — zakotwiczone skanem).

```bash
python3 tools/scale_verdict.py 200     # czeka do 200 s na cel
```

Wypisuje **skalę wymaganą = amcl/raw** (to powinno siedzieć w `trans_scale`) oraz
**dryf `map→odom` na metr drogi** — ta druga liczba jest miarą tego, co widać
w Foxglove jako „skan ucieka od mapy".

Iloraz netto jest poprawny nawet gdy trasa nie jest prosta: jeśli wszystkie przyrosty
są mnożone przez `k`, to i wektor wynikowy. Sonda ostrzega, gdy ścieżka jest znacznie
dłuższa od netto (trasa zawracała) — wtedy iloraz jest słabą miarą.

**Nie ustawiaj `Pose estimate` w trakcie pomiaru** — teleportuje AMCL, a sonda liczy
na niego jako prawdę odniesienia. Sonda wykrywa takie skoki (> 0.30 m) i przycina okno
albo unieważnia przebieg. Wykrywa też nieciągłości `/dog_odom` (skok > 5 cm między
próbkami przy ~1 kHz = reset odometrii firmware).

Kluczowy kontekst: **`trans_scale` to stała chodu, nie robota** — po zmianie toru
`cmd_vel` trzeba ją przemierzyć (`docs/etap_a_utwardzenie_nav.md` §4.6).

## `bt_trace.py` — dlaczego nav2 przerwał cel

Subskrybuje `/behavior_tree_log` (nav2 publikuje tam **każde** przejście statusu
węzła drzewa) i wysyła cel **niemożliwy do zaplanowania** (poza mapą), więc
**robot się nie rusza** — planner pada na pierwszym ticku i żadna ścieżka nie
powstaje.

```bash
python3 tools/bt_trace.py 45            # okno 45 s
python3 tools/bt_trace.py 45 60 60      # wlasny cel (x, y) w ramce map
```

Wypisuje osi czasu przejść z nazwami węzłów, zaznacza każde `FAILURE` i mówi
wprost, **czy drzewo w ogóle sięgnęło po recovery**. To narzędzie rozstrzygnęło,
że stockowe drzewo pomija `Spin`/`Wait`/`BackUp` przez bramki
`WouldA*RecoveryHelp` — z samych logów kontenera tego nie widać
(`docs/etap_a_utwardzenie_nav.md` §4.2).

Uwaga: `ReactiveFallback` re-tickuje dzieci co ~10 ms, więc `GoalUpdated`
generuje ~100 zdarzeń/s (45 s ≈ 10 tys. linii). Filtruj po nazwie węzła.

## `cpu_probe.py` — CPU per proces (bez `pidstat`/`htop`)

Sonda oparta wyłącznie na `/proc`, bo Jetson nie ma zainstalowanego
`pidstat`/`htop`. Liczy delty `utime+stime`, sortuje malejąco i **rozbija
najcięższy proces na wątki** — bez tego nav2 jest nieprzejrzysty, bo cały siedzi
w jednym `component_container_isolated`.

```bash
python3 tools/cpu_probe.py 20           # na HOŚCIE robota, nie w kontenerze
```

Nie wymaga ROS-a. Uruchamiać na maszynie, gdzie żyją procesy (host widzi też
procesy kontenera). Pomiary bazowe: `docs/etap_a_utwardzenie_nav.md` §4.4.

## `fastlio_pcd_to_pgm.py`

Single conversion PCD → PGM (manual, dla ad-hoc). Patrz
`pcd_variant_grid.py` dla batch generowania.
