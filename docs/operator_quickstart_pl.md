# Quickstart dla operatora — workflow nawigacji G1

Prosta instrukcja od mapowania do operowania robotem. Zakłada że
robot odpalony, Livox driver chodzi, workspace zbudowany.

## Co masz nowego do dyspozycji

1. **`./tools/run_mapping_session.sh`** — automatyczne mapowanie od A do Z.
   Wcześniej trzeba było odpalić FAST-LIO, wywołać service, znaleźć PCD,
   uruchomić skrypt konwersji. Teraz jedna komenda.

2. **`tools/scan_height_tuner.py`** — live tuner `pointcloud_to_laserscan`
   `min_height/max_height`. Slider zmienia params **bez restartu** —
   widzisz efekt w RViz natychmiast. Save gdy znajdziesz dobre.

3. **`tools/map_picker.py`** — PyQt6 GUI do przeglądu wariantów map
   (~30 z `pcd_variant_grid`). Wybierasz najlepszą wzrokowo, klik →
   zapis jako produkcyjna `~/maps/lab.yaml`.

4. **`tools/pcd_variant_grid.py`** — generuje 30+ wariantów PGM
   z jednego PCD (Z-slice × resolution × density). Plus `manifest.json`.

## Jednorazowy setup

```bash
sudo apt install python3-pyqt6 python3-yaml
cd ~/g1_courier_ws
git pull
colcon build --symlink-install
source install/setup.bash
```

## Workflow: od zera do nawigacji

### Krok 1 — zmapuj salę

```bash
# T1 (osobny terminal — Livox driver):
ros2 launch livox_ros_driver2 msg_MID360_launch.py

# T2 — orchestrator:
cd ~/g1_courier_ws
source install/setup.bash
./tools/run_mapping_session.sh
```

Skrypt poprowadzi:
1. Sprawdzi `/livox/lidar` Hz
2. Odpali FAST-LIO
3. Wyświetli "Press Enter when done" — w tym czasie przejedź robotem
   teleopem (≤ 0.15 m/s) po całej scenie, wróć do startu
4. Po Enter wywołuje `/map_save` → PCD plik
5. Ubija FAST-LIO
6. Generuje ~30 wariantów PGM (z flip-y dla Mid-360 upside-down)
7. Otwiera **map_picker GUI**: wybierasz najlepszy wariant wzrokowo,
   klik **Save as production map** → `~/maps/lab.yaml`

### Krok 2 — odpal nav (operator mode)

```bash
# T1:
ros2 launch g1_courier_bringup real.launch.py \
  map:=$HOME/maps/lab.yaml \
  enable_mission:=false

# T2 — RViz:
source ~/g1_courier_ws/install/setup.bash
LIBGL_ALWAYS_SOFTWARE=1 rviz2 \
  -d $(ros2 pkg prefix g1_courier_bringup)/share/g1_courier_bringup/rviz/courier.rviz

# T3 — operator GUI:
ros2 run g1_courier_bringup operator_gui
```

### Krok 3 — jeśli `/scan` nie pokrywa się z mapą — użyj tunera

Najczęstsza przyczyna nieudanej lokalizacji AMCL. Odpal tuner **w trakcie
działającego stacka**:

```bash
python3 ~/g1_courier_ws/tools/scan_height_tuner.py
```

- Slider `min_height` / `max_height`
- Patrzysz w RViz LaserScan display — kontur scan vs ściany mapy
- Quick presets: `podłoga`, `niskie`, `biurka`, `ściany`, `wszystko`
- Status (Hz + valid %) pokazuje czy slice ma sense
- Gdy skan pokrywa ściany → **Save to yaml**

Live update przez `ros2 param set` — bez restartu.

### Krok 4 — pierwsze nav goal

W operator_gui:
1. **Set Initial Pose** — wpisz aktualną pozycję robota → klik
2. **Navigate** → wybierz waypoint z dropdown lub wpisz x/y/yaw → **GO**

W RViz: powinieneś widzieć `/plan` (czerwona linia), robot fizycznie
jedzie do celu.

## Co najczęściej psuje się

| Objaw | Najpierw sprawdź |
|---|---|
| `frame map does not exist` | `Set Initial Pose` (krok 4) |
| `/scan` pusty | QoS — `ros2 topic echo /scan --qos-reliability best_effort` |
| `/scan` ma dane ale nie pasuje do mapy | tuner (krok 3) |
| Robot dziwnie się porusza | `ros2 topic echo /cmd_vel_nav` vs `/cmd_vel` — czy floor arbiter nie skacze |
| Mission BT wystrzela goale mimo `enable_mission:=false` | zombie: `pkill -9 -f mission_node` |
| Czarny ekran RViz | `LIBGL_ALWAYS_SOFTWARE=1 rviz2 ...` |
| `Extrapolation Error` w logu | zegar: `sudo timedatectl set-ntp true` |

Pełne procedury debugowania:
- `docs/amcl_sanity_test.md` — 7-step AMCL verification
- `docs/nav_debug_workflow.md` — kolejność testów + tabela params

## Mission BT (autonomiczny cykl)

Gdy nav działa stabilnie z manualnymi celami:

```bash
ros2 launch g1_courier_bringup real.launch.py \
  map:=$HOME/maps/lab.yaml \
  enable_mission:=true
```

Mission BT zacznie cykl pickup A → place B → pickup B → place A z
`src/g1_courier_mission/config/waypoints.yaml`. Zedytuj waypoints
zgodnie z faktyczną pozycją table_a/table_b w twojej sali.
