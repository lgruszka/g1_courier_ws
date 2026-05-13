# Operator GUI — krótka instrukcja

GUI w PyQt6 do wystawiania zadań na **realnym G1** w trybie operator
(`enable_mission:=false`). Klikasz przycisk → action goal leci do
serwera → status w panelu. Zastępuje wpisywanie `ros2 action send_goal`
plus jest bezpieczniejsze (single-flight, E-stop, cancel).

## Wymagania jednorazowe

```bash
# Ubuntu 22.04 (zalecane dla zespołu):
pip install PyQt6 PyYAML

# Ubuntu 24.04 (lokalny dev z PEP 668):
sudo apt install python3-pyqt6 python3-yaml
```

## Uruchomienie

**Krok 0 — pre-flight** (zawsze sprawdź):
```bash
timedatectl | grep -E '(NTP|synchronized)'
# musi być NTP active + synchronized yes; jeśli nie:
sudo timedatectl set-ntp true   # czekaj 5 s
```

**Krok 1 — terminal 1 — stack bez mission BT**:
```bash
cd /home/neo/j2s
source install/setup.bash
ros2 launch g1_courier_bringup real.launch.py \
  map:=$HOME/maps/lab.yaml \
  enable_mission:=false
```
Czekaj ~15 s aż `lifecycle_manager_navigation: Managed nodes are active`.

**Krok 2 — terminal 2 — GUI**:
```bash
source /home/neo/j2s/install/setup.bash
ros2 run g1_courier_bringup operator_gui
```

**Krok 3 (opcjonalny) — terminal 3 — RViz dla podglądu mapy + skanu**:
```bash
source /home/neo/j2s/install/setup.bash
rviz2 -d $(ros2 pkg prefix g1_courier_bringup)/share/g1_courier_bringup/rviz/courier.rviz
```

## Pierwszy uruchom — sanity check

W GUI lewy panel **Status**:
- `AMCL pose` — pokazuje aktualną pozę robota na mapie (live update)
- `/scan Hz` — zielony, ~10 Hz
- `/lowstate Hz` — zielony, ~500 Hz

Jeśli któryś czerwony / „—" — patrz [Co psuje się najczęściej](#co-psuje-się-najczęściej).

## Workflow operatora

### A. Kalibracja AMCL (raz na start)

1. **Set Initial Pose** w lewym panelu — wpisz **x/y/yaw** tam gdzie
   robot fizycznie stoi w lab
2. Klik **Set Initial Pose**
3. W logu: `info /initialpose: x=... y=... yaw=...°`
4. AMCL skacze do tej pozycji. Live `AMCL pose` aktualizuje się.

### B. Pojechanie do punktu

**Opcja A — z dropdown** (gdy waypoint zdefiniowany w `waypoints.yaml`):
1. **Navigate** → dropdown **Waypoint** → wybierz `table_a` lub `table_b`
2. Pola x/y/yaw auto-fill
3. Klik **GO →**
4. Log: `→ navigate: ...` → feedback `d=X.XXm phase=navigating` → `OK arrived at ...`

**Opcja B — ad-hoc**:
1. Wpisz x/y/yaw ręcznie
2. Klik **GO →**

### C. Dock do kartonu (apriltag)

Wymaga: karton z AprilTag id=10 widoczny przez kamerę D435i.

1. **Dock** → radio **APRILTAG (pickup)**
2. **tag_id:** 10 (default), **target z:** 0.30 (m od tagu)
3. **xy_tol:** 0.10, **yaw_tol:** 0.15
4. Klik **DOCK →**
5. Robot servuje wizualnie do tagu

### D. Dock do biurka (lidar line)

Używaj **przy odkładaniu paczki** (kamera okryta).

1. **Dock** → radio **LIDAR_LINE (place)**
2. **xy_tol:** 0.03, **yaw_tol:** 0.05 (precyzyjniej niż apriltag)
3. Klik **DOCK →**
4. Robot dopasowuje się do krawędzi biurka (RANSAC line fit z `/scan`)

### E. Pickup / Place

1. Po pomyślnym dock APRILTAG: klik **PICK box** (Arm skills)
2. Po pomyślnym dock LIDAR_LINE: klik **PLACE box**
3. Każda sekwencja ~10-20 s, feedback `phase=approach/grasp/lift...`

### F. Retreat

1. **dist:** 0.5 m (default), **speed:** 0.12 m/s
2. Klik **RETREAT ←**
3. Robot cofa się open-loop przez podaną odległość

## Bezpieczeństwo

### ⛔ E-STOP

Duży czerwony przycisk **lewy panel**. Klik:
1. **Cancel** aktywnej akcji
2. **Zero cmd_vel** publikacja x5 (twardy stop)
3. **`/safety/set_freeze freeze=true`** (jeśli service dostępny)

Po E-stopie robot stoi, kolejne goale **odrzucane**. Żeby odblokować:
**Unfreeze** (przycisk pod E-stop).

### Cancel active goal (lżejsze niż E-stop)

Pomarańczowy przycisk **Cancel active goal**. Anuluje aktualny goal
ale **nie** freezuje. Robot zatrzymuje się natychmiast (controller
dropuje cmd_vel), kolejne goale są przyjmowane.

### Single-flight

GUI wysyła **tylko jeden goal naraz**. Jeśli klikniesz drugi gdy
pierwszy w toku — log: `BUSY — odrzucam <action>`. Najpierw **Cancel**
lub poczekaj na SUCCESS/FAIL.

## Typowa sekwencja pickup→place

```
1. Set Initial Pose          (kalibracja AMCL)
2. Navigate → table_a        (jedź do biurka A)
3. Dock → APRILTAG id=10     (dock do paczki)
4. PICK box                  (chwyć)
5. Retreat 0.5 m             (odejdź)
6. Navigate → table_b        (jedź do B)
7. Dock → LIDAR_LINE         (dock do krawędzi B)
8. PLACE box                 (odłóż)
9. Retreat 0.5 m             (odejdź)
```

Każdy krok czekasz na `OK` w logu zanim klikniesz następny.

## Co psuje się najczęściej

| Objaw | Przyczyna | Fix |
|---|---|---|
| `BUSY — odrzucam X` | aktywny goal w toku | poczekaj lub **Cancel active goal** |
| `server niedostępny` | action server nie wystartował | `ros2 action list` — sprawdź czy action jest |
| `RejECTED` po wysłaniu nav | brak TF `map` lub AMCL nieaktywne | `Set Initial Pose` + sprawdź `ros2 lifecycle get /amcl` |
| Mission BT wystrzela goale mimo `enable_mission:=false` | zombie mission_node | `pkill -9 -f mission_node` + restart launchu |
| GUI nie startuje, `No module named 'PyQt6'` | brak pakietu | `pip install PyQt6` lub `sudo apt install python3-pyqt6` |
| `/scan Hz: 0` | Livox driver / Unitree firmware nie publikuje | `ros2 topic hz /livox/lidar` na onboard PC |
| AMCL pose nie aktualizuje się | `/amcl_pose` 0 Hz | `ros2 lifecycle get /amcl` musi być `active [3]` |
| `Extrapolation into the future` w logu launchu | clock skew Livox vs ROS | `transform_tolerance` w `nav2_params.yaml` ↑ do 1.5 |

## Czego GUI **nie robi**

- Nie zastępuje RViz — mapa, skan, costmaps, TF tree zostają w RViz
- Nie modyfikuje nav2 params na żywo (`xy_goal_tolerance` itp. z `nav2_params.yaml`)
- Nie wystawia mission BT — `enable_mission:=false` znaczy że robot
  **wykonuje tylko to** co klikniesz
- Nie ma sekwencji ("nav → dock → pick → retreat" jako jeden klik) —
  na razie wystawiasz krok po kroku ręcznie. Sekwencer jako v3.

## Powrót do trybu automatycznego (mission BT)

Wyłącz operator GUI (Ctrl+C), zatrzymaj launch, odpal z `enable_mission:=true`:
```bash
ros2 launch g1_courier_bringup real.launch.py \
  map:=$HOME/maps/lab.yaml \
  enable_mission:=true
```
Mission BT odpala się ~5 s po starcie, jedzie cykl A↔B autonomicznie
(z `waypoints.yaml` + hardcoded sekwencji w `mission_node.py`).
