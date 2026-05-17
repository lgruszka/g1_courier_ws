# AMCL sanity test — krok po kroku

Procedura testowa AMCL na realnym G1. Po każdym kroku jest **kryterium
sukcesu**. Jeśli krok pada — patrz "Co jeśli pada" pod nim. Przechodzisz
dalej tylko gdy poprzedni jest ✅.

Cel: ustalić **czy AMCL ma poprawną pozę robota na mapie** plus czy
wszystkie TF się zgadzają, **zanim** wystawimy nav goal.

## 0. Pre-flight

```bash
# zegar:
timedatectl | grep -E '(NTP|synchronized)'
# musi: active + yes; inaczej: sudo timedatectl set-ntp true

# zombie kill (na wszelki):
pkill -9 -f mission_node
pkill -9 -f component_container

# odpal stack bez mission BT:
source /home/neo/j2s/install/setup.bash
ros2 launch g1_courier_bringup real.launch.py \
  map:=$HOME/maps/scenarios_g1_map_flipped/<wybrana_mapa>.yaml \
  enable_mission:=false

# w drugim terminalu — RViz z presetem:
source /home/neo/j2s/install/setup.bash
LIBGL_ALWAYS_SOFTWARE=1 rviz2 \
  -d $(ros2 pkg prefix g1_courier_bringup)/share/g1_courier_bringup/rviz/courier.rviz
```

Czekaj aż w logu pojawi się: `lifecycle_manager_navigation: Managed nodes are active`.

W RViz upewnij się że masz włączone (panel Displays):
- ✅ **Map** (`/map`)
- ✅ **LaserScan** (`/scan`, Reliability: **Best Effort**)
- ✅ **RobotModel** (`/robot_description`)
- ✅ **TF**
- ✅ **AMCL Pose** (`/amcl_pose`)

Plus dodaj manualnie (Add → By topic):
- **ParticleCloud** display → topic `/particle_cloud`

---

## Test 1 — Set Initial Pose

**Cel**: AMCL wie gdzie robot fizycznie stoi.

1. **Postaw robota w znanym punkcie sali**. Najlepiej w rogu — łatwo
   zmierzyć pozycję i orientację. Załóżmy że stoi w punkcie A
   (np. lewy-dolny róg sali, patrzy w +X mapy).
2. W RViz pasek narzędzi → kliknij **2D Pose Estimate** (zielona strzałka)
3. Na mapie **kliknij** dokładnie tam gdzie robot fizycznie jest
4. **Przeciągnij** strzałkę w kierunku w którym robot fizycznie patrzy
   (czyli jego "przód")
5. Puść

**Kryterium sukcesu**:
- Model robota w RViz **skacze** do tej pozycji
- Skan LiDAR-a (kolorowe punkty) **pokrywa się** ze ścianami mapy
  z dokładnością ~10-20 cm

**Co jeśli pada**:
- Skan przesunięty równomiernie o ~50 cm w jednym kierunku
  → niedokładne kliknięcie. Powtórz 2D Pose Estimate, dokładniej.
- Skan **odwrócony lustrzanie** vs mapa → mapa lub static TF źle
  skonfigurowane (vide: notatka o `--flip-y` w pcd_variant_grid + roll=π
  w real.launch.py). Sprawdź czy oba są spójne.
- Skan **obrócony** (np. o 90°) → ponowne 2D Pose Estimate z innym yaw.

---

## Test 2 — Orientacja AMCL Pose pasuje do fizycznego przodu

**Cel**: yaw w AMCL = fizyczny yaw robota.

1. W RViz znajdź czerwoną/zieloną **strzałkę AMCL Pose** (~ 1 m długości,
   wychodzi z robota w kierunku "przodu" według AMCL)
2. Spójrz na **fizycznego robota** — gdzie jest jego front
   (np. tam gdzie kamera RealSense, tam gdzie "patrzy")
3. Porównaj — czy strzałka AMCL w RViz wskazuje **ten sam kierunek**?

**Kryterium sukcesu**:
- Strzałka AMCL ≈ fizyczny przód robota (tolerancja ±20°)

**Co jeśli pada**:
- Strzałka AMCL wskazuje **przeciwnie** niż robot fizycznie patrzy
  → przy Set Initial Pose przeciągnąłeś strzałkę w złym kierunku.
  Powtórz Test 1 z **zwróceniem uwagi** na yaw.
- Strzałka **obrócona o 90°** → przeoczona oś. Powtórz Test 1.
- **Mimo poprawnego klikania strzałka jest mirror** → po fixie flip-y mapy +
  roll=π w TF, układ powinien być spójny; jeśli wciąż mirror, jedno
  z dwóch jest źle. Skontaktuj się z Łukaszem.

---

## Test 3 — Particle cloud zbieżność (statyczna)

**Cel**: AMCL ma "pewną" pozę, nie zgaduje.

1. Robot stoi (nie ruszaj go)
2. W RViz spójrz na **ParticleCloud** — chmura strzałek wokół robota
3. Patrz przez **15-20 sekund** plus poczekaj aż AMCL ustabilizuje się

**Kryterium sukcesu**:
- Chmura particles jest **skupiona** w promieniu ~20-30 cm wokół modelu
  robota
- Wszystkie strzałki particles wskazują **podobny** kierunek (yaw)

**Co jeśli pada**:
- Particles **rozjeżdżone po pół mapy** → AMCL nie wie gdzie jest robot.
  Powtórz Test 1, plus po Set Initial Pose obróć lekko robotem żeby
  AMCL miał "świeży skan z różnych kierunków".
- Particles **w jednej kupie ale w złym miejscu** (np. obok robota
  zamiast pod nim) → Initial Pose dał AMCL fałszywą pewność. Powtórz
  Test 1 dokładniej.

---

## Test 4 — Particle convergence po ruchu (dynamic)

**Cel**: AMCL aktualizuje się gdy robot się rusza.

1. Robot stoi, AMCL stabilne (po Teście 3)
2. **Popchnij robota ręcznie** o ~30 cm do przodu (delikatnie, by nie
   uszkodzić)
3. Obserwuj w RViz:
   - Model robota powinien **podążać** za fizycznym ruchem
   - Strzałka AMCL Pose podąża
   - Particle cloud "wydłuża się" w kierunku ruchu, potem skupia

**Kryterium sukcesu**:
- Model w RViz dojeżdża **w tym samym miejscu** co fizyczny robot
  (tolerancja ~15 cm)

**Co jeśli pada**:
- Model w RViz idzie w **przeciwnym kierunku** niż popchnięcie
  → `/dog_odom` (firmware odom) ma odwrócony znak. Sprawdź
  `ros2 topic echo /dog_odom` podczas ruchu — czy delta zgadza się z fizycznym ruchem.
- Model **nie rusza się** mimo popchnięcia → `/dog_odom` nie publikuje
  lub odom_tf_relay padł. Sprawdź `ros2 topic hz /dog_odom`.
- Model idzie ale **z innym yaw** niż robot fizycznie obracał
  → znowu yaw flip. Wróć do Testu 2.

---

## Test 5 — Covariance AMCL

**Cel**: numerycznie potwierdzić zbieżność.

```bash
ros2 topic echo /amcl_pose --once --field pose.covariance
```

Wartości to macierz 6×6 (36 liczb). Patrz na **przekątną**:
- Pozycja 0 (indeks `0`) = **XX covariance**
- Pozycja 7 (indeks `7`) = **YY covariance**
- Pozycja 35 (indeks `35`) = **yaw_yaw covariance**

**Kryterium sukcesu**:
- XX < 0.05
- YY < 0.05
- yaw_yaw < 0.05

(Wartości w m² dla XX/YY, rad² dla yaw_yaw. 0.05 m² → odchylenie ~22 cm.)

**Co jeśli pada**:
- XX/YY > 0.2 → particles nie zbiegły, wróć do Testu 3 plus ruszaj
  robotem żeby zebrać więcej obserwacji
- yaw_yaw > 0.2 → orientacja niepewna, sprawdź Test 2

---

## Test 6 — Krótki nav goal

**Cel**: AMCL + nav2 + controller współpracują, robot fizycznie jedzie.

1. Włącz fizycznie **uwagę na robota** plus miej **pilota Unitree** w ręku
   z palcem na E-stopie
2. Upewnij się że **ścieżka jest pusta** (1-2 m wolnej przestrzeni przed
   robotem)
3. W RViz pasek narzędzi → **2D Goal Pose**
4. Kliknij na mapie **1 m przed robotem**, przeciągnij strzałkę w jego
   aktualnym kierunku, puść

**Kryterium sukcesu**:
- W RViz pojawia się **czerwona linia `/plan`** od robota do celu (prosta,
  bez dziwnych zakrętów)
- Robot **fizycznie się rusza w kierunku celu**
- Robot **zatrzymuje się** ~30 cm od celu
- W logu: `bt_navigator: Goal succeeded`

**Co jeśli pada**:
- `/plan` **nie pojawia się** → `Failed to create plan` — cel w obstacle.
  Daj cel dalej od ścian.
- Robot **stoi mimo planu** → `/cmd_vel` 0 Hz lub bridge problem.
  Sprawdź `ros2 topic hz /cmd_vel`.
- Robot **jedzie w losowy kierunek** → AMCL ma yaw flip lub bridge
  cmd_vel przekręcony. Wróć do Testu 1+2, lub:
  ```bash
  ros2 topic pub --once /cmd_vel geometry_msgs/Twist \
    '{linear: {x: 0.15}}'
  # robot fizycznie powinien jechać do przodu
  # jeśli idzie w tył/bok — bridge problem
  ```
- Robot **kręci się w kółko** → cmd_vel angular dominuje. Patrz log
  `transformPoseInTargetFrame` errors plus zob. `transform_tolerance`
  w nav2_params.yaml.
- Robot dojeżdża ale **przeskakuje cel** o >50 cm → controller jest
  agresywny, ale AMCL/cmd_vel działa. To dostrojenie.

---

## Test 7 — Powtarzalność

Jeśli Test 6 ✅, powtórz dla:
- Cel 1 m w **prawo** (cel `y=+1.0`)
- Cel 1 m w **lewo** (cel `y=-1.0`)
- Cel 1 m w **tył** (z `yaw` o 180°)

Każdy powinien zakończyć się sukcesem. Jeśli pewne kierunki działają a
inne nie → typowo yaw/frame mapping issue.

---

## Co notować + wysłać Łukaszowi gdy coś nie działa

Dla każdego nieudanego testu zapisz:
1. **Numer testu** (np. "Test 4 pada")
2. **Co fizycznie obserwujesz** (np. "robot popchnięty w przód, model RViz idzie w bok")
3. **Wynik `/amcl_pose`** (covariance + position)
4. **Screenshot RViz** z particles + scan + map
5. **5-10 ostatnich linii logu** real.launch.py z `transformPose|amcl|controller`

---

## Quick summary "wszystko OK" criteria

| Test | Sukces |
|---|---|
| 1. Set Initial Pose | Skan pokrywa ściany mapy ±20 cm |
| 2. Yaw orientation | Strzałka AMCL = fizyczny przód robota |
| 3. Static convergence | Particles skupione ~30 cm wokół modelu |
| 4. Dynamic update | Model w RViz podąża za fizycznym ruchem |
| 5. Covariance | XX, YY, yaw_yaw < 0.05 |
| 6. Krótki nav | /plan sensowny, robot dojeżdża |
| 7. Powtarzalność | 1 m w 4 kierunki działa |

Jeśli wszystkie 7 ✅ → AMCL działa stabilnie, można testować dłuższe
trasy, dokowanie, mission BT.
