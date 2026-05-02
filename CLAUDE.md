# CLAUDE.md — guidance for the next Claude Code session

Ten plik jest automatycznie ładowany jako kontekst przy starcie sesji w katalogu `g1_courier_ws/`. Trzymaj go zwięzłym; szczegóły idą do `README.md` i `docs/ARCHITECTURE.md`.

## TL;DR

- **Projekt**: humanoid Unitree G1 ma cyklicznie przenosić karton między biurkami A i B oznaczonymi AprilTagami.
- **Stack**: ROS2 Humble + nav2 + slam_toolbox/AMCL + AprilTag visual servo + parametryczny arm controller na `arm_sdk`. Mission jako Behavior Tree (`py_trees_ros`).
- **Faza**: **sim-first**. Realny robot pojawi się dopiero po stabilnym przejściu pełnego cyklu w symulacji.
- **Symulator**: strategia trzy-fazowa — (0) no-sim z fixturami, (1) MuJoCo (`unitree_mujoco`) jako *główne* środowisko testów, (2) Isaac Sim na cloud GPU jako finalna walidacja foto-realizmu. Szczegóły w sekcji "Decyzja: symulator".
- **Workspace**: scaffolded (~60 plików, 6 paczek). Buduje się, ale nie był jeszcze odpalony przeciw realnemu/symulowanemu robotowi.

## Pierwsze kroki dla nowej sesji

Przeczytaj w tej kolejności:

1. `README.md` — high-level co i dlaczego.
2. `docs/ARCHITECTURE.md` — **źródło prawdy dla decyzji projektowych**. Każda zmiana w kodzie ma być z nim zgodna; jeżeli nie jest, najpierw aktualizujesz dokument, potem kod.
3. `src/g1_courier_msgs/action/*.action` — kontrakty między warstwami. To jest API systemu.
4. `src/g1_courier_arm_skills/g1_courier_arm_skills/arm_controller.py` — najważniejszy logiczny blok manipulacji.
5. `src/g1_courier_docking/g1_courier_docking/dock_action_server.py` — dock action z trzema trybami.
6. `src/g1_courier_mission/g1_courier_mission/mission_node.py` + `behaviors.py` — Behavior Tree misji.

## Stan obecny

| Komponent | Status |
|---|---|
| Definicje `g1_courier_msgs` (action/srv/msg) | ✅ pełne |
| Arm controller (parametric, CRC, weight ramping, grasp verifier) | ✅ pełne, IK hook to-do |
| `pick_box` / `place_box` action servery | ✅ pełne |
| `dock_to_table` action server, tryb APRILTAG | 🟡 servo loop OK, ale `_extract_tag_residual` polega na `det.pose.pose.pose.position` którego `apriltag_msgs/AprilTagDetection` z `ros-jazzy-apriltag-ros` NIE ma (tylko homography + corners). Pełna integracja wymaga (a) TF z osobnego pose estimation node'a, lub (b) solvePnP w naszym kodzie z homography + camera_info — robione w Fazie 1.1 |
| `dock_to_table` tryb LIDAR_LINE | ✅ RANSAC line fit + perpendicular alignment, walidowane na fixture (1.8 mm xy / 1.7° yaw convergence) |
| `dock_to_table` tryb AMCL_ONLY | ✅ pełne (trywialne) |
| `cmd_vel_arbiter` z carry mode + freeze + e-stop | ✅ pełne |
| `navigate_proxy` → nav2 | ✅ pełne |
| `retreat` action server | ✅ pełne |
| Mission BT (cykl A↔B 4-fazowy) | ✅ pełne; cycle counter + MissionStatus publishing + max_cycles parameter |
| Launch full + mapping | ✅ scaffold |
| Configi nav2 / slam_toolbox / amcl / apriltag | ✅ scaffold, **wszystkie wartości oznaczone `TODO_TUNE` do dostrojenia w symie** |
| `sim_cmd_vel_bridge_node` (kinematic, Faza 0 no-sim) | ✅ pełne, smoke test 4-fazowego cyklu BT przeszedł |
| `sim_lowstate_publisher_node` (idle LowState fixture, Faza 0) | ✅ |
| Fake action serwery (`fake_navigate_proxy`/`dock`/`pick`/`place`/`retreat`) | ✅ |
| `phase0_smoke.launch.py` (mission BT na fake'ach) | ✅ przeszedł, BT pętli się sam |
| Integracja `unitree_mujoco` (Faza 1) — communication path | ✅ DDS bridge zwalidowany w obie strony |
| Real `pick_action_server` zwalidowany na MuJoCo (bridge) | ✅ pełna sekwencja P0→P6 + grasp_verifier OK |
| Real `place_action_server` zwalidowany na MuJoCo (bridge) | ✅ pełna sekwencja P5→ZERO + handoff_to_fsm OK |
| `phase1_smoke.launch.py` (mission BT z real arm + retreat + fake nav/dock) | ✅ pełen cykl A↔B z 4 real arm + 2 real retreat (~70 s) |
| `sim_lidar_publisher_node` (LiDAR fixture, ray-cast vs odom) | ✅ |
| AprilTag tagi + kamera RGB w `unitree_mujoco` scene XML (Faza 1.1) | ❌ mac side, edycja scene_29dof.xml |
| Real LiDAR sensor w `unitree_mujoco` scene XML (Faza 1.2) | ❌ mac side, rangefinder array |
| **Pelvis weld pin** (sim-only, dla 1.0..1.2) | ❌ wymaga edycji mac scene.xml — patch w `PELVIS_WELD_PATCH.md`. Po dodaniu robot stoi jak posąg, arm tasks działają bez walking controllera. Wzorowane na `g1_logistics_demo`. |
| **Walking controller** (`/cmd_vel` → leg motors), Faza 1.3 | ❌ **brama dla 1.4–1.6**; oba α (per-joint PD) i β (pretrained RL policy z unitree_rl_gym) **zawiodły** — α nie ma gravity feedforward, β nie była trenowana przeciwko external arm forces (P0→P6 arm motion = 20+ Nm na pelvis = poza distribution policy). Wymaga retreningu RL z arm-force domain randomization, albo whole-body MPC. Kod `g1_courier_locomotion/walking_*_node.py` zachowany jako wireup. Aktualnie 1.0..1.2 używają pelvis weld pin'a w scene.xml zamiast walking. |
| Mapa z LiDAR `slam_toolbox` w MuJoCo (Faza 1.4) | ❌ wymaga 1.2 + 1.3 |
| AMCL + nav2 real `navigate_to_pose` (Faza 1.5) | ❌ wymaga 1.4 |
| Pełen mission BT cycle z real ruchem A↔B (Faza 1.6) | ❌ wymaga 1.1 + 1.5 |
| Walidacja foto-realistyczna na Isaac Sim (cloud, Faza 2) | ❌ na koniec |

## Decyzja: symulator (strategia trzy-fazowa)

Zamiast jednego symulatora trzy fazy o rosnącej wierności i koszcie, napędzane co potrafią walidować:

### Faza 0 — no-sim (kinematic only)

- **Co**: `sim_cmd_vel_bridge_node` integruje `/cmd_vel` w czasie i publikuje TF `map→base_link` + `/odom`. Plus fixturized `unitree_hg/LowState` (bag z reala albo synthetic). Plus mock action server stuby do testów BT.
- **Co walidujemy**: kontrakty action, busy-lock/cancel/timeout paths, mission BT cycle, dock state machine, arm controller na fake `lowstate`.
- **Czego nie walidujemy**: fizyki, kolizji, sensorów, AprilTag detection.
- **Hardware**: dowolny. Działa na Parallels VM Apple Silicon.

### Faza 1 — MuJoCo (`unitree_mujoco`) — *główna praca projektu*

- **Co**: oficjalne repo Unitree z modelem G1 i ROS2 bridgem. Używa **`unitree_hg/LowCmd, LowState`** — *dokładnie tych samych* wiadomości co realny robot.
- **Powody**:
  - **Real Unitree contract** — arm controller, `lowcmd_crc.py`, pick/place skille są walidowane na prawdziwym IDL kontrakcie. Brak fake'owania jak w Isaac.
  - **GPU opcjonalny** — fizyka MuJoCo jest CPU-only; renderowanie kamery przyspiesza GPU, ale nie wymaga RTX/CUDA.
- **Co walidujemy**: arm controller na realistycznych stawach, IDL kontrakt LowState/LowCmd, CRC, grasp/release verifier, mission BT (sequence + recovery), dock LIDAR_LINE (RANSAC line fit). Po dorobieniu LiDAR sensora i AprilTag textur w scene XML: mapowanie LiDARem (slam_toolbox), AMCL, dock APRILTAG. Po dorobieniu walking controllera (patrz Faza 1.3 niżej): nav2 end-to-end i pełen mission cycle.
- **Świadomy kompromis — locomotion**: `unitree_mujoco` daje **sam bridge IDL**, **nie** ma wbudowanego walking controllera. Sport mode API w realnym G1 chodzi w firmware (proprietary) — w MuJoCo trzeba dorobić własny kontroler chodu. Bez niego G1 jako bipedal natychmiast pada pod grawitacją, ramiona można sterować przez arm_sdk (działa) ale `/cmd_vel` nie ma jak dotrzeć do motors nóg. Dlatego dzielimy Fazę 1 na sub-fazy 1.0..1.6 (patrz "Sub-fazy Fazy 1" niżej) — walking controller jest **osobnym krokiem** 1.3.
- **Hardware**: dowolny x86_64/aarch64 Linux. Renderowanie kamery przez virtio GPU (Parallels) jest wolne (~5-10 FPS) — wystarcza do `apriltag_ros`, ale dynamiczny dock może wymagać GPU passthrough lub mocniejszego hosta.

### Sub-fazy Fazy 1

Każda kolejna sub-faza wymaga poprzednich. Walking controller (1.3) jest **bramą** — bez niego nav2/dock APRILTAG/mapowanie nie ma jak realnie sterować robotem w MuJoCo.

- **1.0 — arm + dock LIDAR_LINE** (zrobione): `pick_action_server`, `place_action_server`, `dock_action_server` MODE_LIDAR_LINE z RANSAC, mission BT cycle z fake nav/dock + real arm + real retreat.
- **1.1 — AprilTag w scene + dock APRILTAG**: dorobić tag textury w scene XML, włączyć kamerę RGB, podpiąć `apriltag_ros` na detekcje, real `dock_action_server` MODE_APRILTAG.
- **1.2 — LiDAR sensor w scene XML + `/scan`**: dodać rangefinder array w MuJoCo (Mid360 surrogate), bridge eksportuje LaserScan na DDS, dock LIDAR_LINE może chodzić na real LiDAR (zamiast `sim_lidar_publisher_node`).
- **1.3 — walking controller** (`/cmd_vel` → motor commands na nogi): conditio sine qua non dla 1.4+. Trzy realne ścieżki, z których **α i β zostały próbowane i obie zawiodły** w aktualnym setupie:
  - α) **Standing PD + cmd_vel fakery** — **NIE DZIAŁA**. Per-joint PD bez gravity feedforward / whole-body coordination → oscylacje. Kod w `walking_controller_node.py` jako wireup.
  - β) **Pretrained RL policy** (`unitree_rl_gym/deploy/pre_train/g1/motion.pt`) — **TEŻ NIE DZIAŁA** w naszym use case. Policy trenowana na free-standing walking (cmd_vel → gait), bez external arm-force disturbance. Nasz pick/place P0→P6 daje 20+ Nm momenty na pelvis — to jest **poza distribution policy** → policy emituje chaotyczne targety nóg → oscylacje, upadek. Kod w `walking_policy_node.py` (TorchScript inference 50 Hz, ~0.5 ms/inference) zachowany jako wireup. Polityka działa technicznie (loaduje, infersowala, publikuje LowCmd) — sam policy training jest niewłaściwy.
  - γ) **Whole-body MPC**: research-grade, sesje+, bardzo wysokie ryzyko.
  - β2) **Custom RL retraining** — wziąć infrastrukturę legged_gym, dodać arm motion / external forces do domain randomization, retrenować policy na cloud GPU. Realne ale kosztowne (dni-tygodnie + cloud koszty).
  - **Aktualna decyzja: pelvis weld pin** (jak w `g1_logistics_demo`) jako fundament dla 1.0..1.2. Robot stoi jak posąg dzięki MuJoCo `<equality><weld>` — patch w `PELVIS_WELD_PATCH.md`. Pełen walking 1.3 odsuwany do momentu gdy będzie dedykowany RL retraining albo MPC.
- **1.4 — Mapa LiDARem** (`slam_toolbox`): wymaga 1.2 i 1.3 (manual teleop driving po sali).
- **1.5 — AMCL + nav2 + real `navigate_to_pose`**: wymaga 1.4.
- **1.6 — Pełen mission BT cycle z prawdziwym ruchem A↔B**: wymaga 1.5 i 1.1.

### Faza 2 — Isaac Sim na cloud GPU (final validation)

- **Co**: Isaac Sim 4.x na AWS g5/g6 albo Lambda Labs (RTX A10/A100). Krótkie sesje walidacyjne, nie main work.
- **Po co**: foto-realistyczne renderowanie AprilTagów (specular highlights, oświetlenie), RTX-ray-trace LiDAR Mid360 (realistyczny szum, nie idealne zwroty). Bramka przed ruszeniem na realny robot.
- **Co walidujemy**: visual servo dock na realistycznym oświetleniu, robustness AprilTag detection na warunkach zbliżonych do realu.
- **Hardware**: cloud RTX, ~1-3 USD/h, sesje godziny nie dni. Koszt całościowy projektu ograniczony do dziesiątek-setek USD, nie tysięcy.
- **Świadomy kompromis**: Isaac Sim 4.x **nie ma natywnego Unitree sport API** (jak było w pierwotnej decyzji). W Fazie 2 *nie* sterujemy chodem przez Isaac — używamy `sim_cmd_vel_bridge_node` z Fazy 0 (kinematic). Faza 2 to *visual* validation, nie dynamics validation.

### Co czego *nie* testujemy w żadnej fazie

- Sport mode firmware Unitree (proprietary, real-only). W MuJoCo dorabiamy *własny* walking controller w 1.3 — może być prostszy i mniej zaawansowany niż firmware.
- Extreme balance recovery (popchnięcia, slip, schody) — to tylko realny robot.
- Battery management, BMS, thermal — tylko real.
- Specyficzne hardware quirks (motor PID, joint friction, IMU drift, latencja CAN) — tylko real.

### Hardware referencyjny dla teamu

- **Faza 0+1 lokalnie**: dowolny Linux box, 16 GB RAM, GPU mile widziane ale nie wymagane.
- **Faza 2**: cloud GPU instance (AWS g5.xlarge wystarcza).
- ROS2 Humble + Ubuntu 22.04 (zgodnie z pierwotną decyzją; lokalne odstępstwa od distro nie idą do team-wide doc).

## Pierwsza praca w nowej sesji (priorytety)

Kolejność odzwierciedla strategię trzy-fazową. Jeżeli workspace nie buduje się czysto, zacznij od kroku 1; jeśli buduje, idź do tego co jest pierwszym `❌` w "Stanie obecnym".

### Faza 0 — no-sim

1. **Sanity build**: `cd g1_courier_ws && colcon build --symlink-install`. Brakujące paczki apt:
   ```bash
   sudo apt install ros-humble-nav2-bringup ros-humble-slam-toolbox \
     ros-humble-pointcloud-to-laserscan ros-humble-apriltag-ros \
     ros-humble-py-trees-ros ros-humble-rosidl-generator-dds-idl
   ```
   Plus `unitree_hg`, `unitree_api` ze źródeł `unitree_ros2` (clone do `src/`, build z workspace; `example/` daj `COLCON_IGNORE`).

2. **Napisz `sim_cmd_vel_bridge_node`** — nowa paczka `g1_courier_sim` (rekomendowane) albo plik w `g1_courier_safety`. Spec:
   - Subskrybuje `/cmd_vel` z arbitra.
   - Integruje pozycję `base_link` w czasie (`x += vx*dt` itd.) — pełna kinematic-only ścieżka.
   - Publikuje TF `map→odom→base_link` i `/odom`.
   - **Nie** zakłada żadnego symulatora — działa autonomicznie. Stoi się też niezależnie sensowny w Fazie 1 jako alternatywa do MuJoCo physics dla scenariuszy bez fizyki (testy mission BT bez kosztów MuJoCo).

3. **Fixture `unitree_hg/LowState`** — prosty publisher na `/lowstate` z bagu (z reala) albo syntetyczny. Cel: arm controller i grasp verifier mają na czym ćwiczyć cancel/timeout/torque-threshold paths.

4. **Test pojedynczych skill**: `ros2 action send_goal /pick_box ...`, `ros2 action send_goal /dock_to_table ... mode: AMCL_ONLY`. Mode AMCL_ONLY działa bez sensorów. APRILTAG i LIDAR_LINE czekają na Fazę 1.

5. **Mission BT smoke** — odpal BT, podaj fake'owane akcje navigate/pick/place/dock. Zwaliduj że cykl A↔B przechodzi w warunkach happy path i że recovery branches odpalają na fake'owym fail.

### Faza 1 — MuJoCo

6. **Instaluj MuJoCo 3.3.6** + `unitree_mujoco` (clone, build, `example/ros2/` jako reference). Bridge eksportuje `unitree_hg/LowCmd, LowState` na te same topiki co realny robot.

7. **Załaduj G1 model** w `unitree_mujoco`, dodaj scenę: dwa biurka, dwa AprilTagi (textury), karton jako rigid body. Uruchom z `--ros2-bridge`.

8. **Mapowanie**: `ros2 launch g1_courier_bringup mapping.launch.py`, jeźdź ręcznie (teleop_twist_keyboard albo nasz `cmd_vel` przez arbitra), zapisz `~/maps/lab.yaml`.

9. **Pełen flow**: `ros2 launch g1_courier_bringup courier_full.launch.py map:=$HOME/maps/lab.yaml`. Cel: 100 cykli bez porażki.

10. **Tuning** `TODO_TUNE` w configach (najistotniejsze):
    - `apriltag.kp_xy`, `apriltag.kp_yaw`, `apriltag.target_distance_m` (`config/docking.yaml`).
    - `predock_x/y/yaw` per stół (`config/waypoints.yaml`).
    - `grasp_tau_threshold_nm` (`config/arm_skills.yaml`).
    - Limity carry mode (`config/safety.yaml`).

### Faza 2 — Isaac Sim na cloud (validation gate przed real)

11. **Postaw cloud GPU instance** (np. AWS g5.xlarge z Ubuntu 22.04 + RTX A10), zainstaluj Isaac Sim 4.x. ROS2 Humble lokalnie + tunelowanie domain ID albo tunel SSH.

12. **Walidacja foto-realistyczna**: scena identyczna jak w MuJoCo, AprilTagi z tekstur PNG, oświetlenie zmienne. Zwaliduj że `apriltag_ros` wykrywa stabilnie z różnych dystansów i kątów.

13. **Walidacja RTX LiDARu** (Mid360): mapowanie i AMCL na ray-trace skanie zamiast idealnego.

Dopiero po Fazie 2 → real robot.

## Reguły pracy w tym projekcie

Pełne zasady w `docs/ARCHITECTURE.md`. Najważniejsze, których nie wolno łamać:

1. **Każda zmiana zgodna z ARCHITECTURE.md**. Jeżeli musisz odejść — najpierw aktualizujesz dokument, potem kod. Sekcja Appendix A (anti-patterns) opisuje dokładnie czego nie powtarzać.
2. **Nowy interfejs między węzłami → najpierw `g1_courier_msgs`**. Bez ad-hoc `String` z JSON-em.
3. **Każdy skill = action** (nie service). Z timeoutem, cancel, busy-lockiem, success/message w result. Wzór: `pick_action_server.py`.
4. **QoS**: `qos_profile_sensor_data` dla LiDAR/kamery, `default` dla `/cmd_vel*`, `TRANSIENT_LOCAL` dla latched topiców (e-stop).
5. **`MultiThreadedExecutor`** dla każdego węzła z action server + subskrypcjami sensorów.
6. **Sleep > 100 ms tylko z checkiem `stop_event`** w pętli (wzór: `arm_controller._wait_for_low_state`).
7. **Controller (logika) bez ROS-a**, ROS w glue layer (wzór: `arm_controller.py` vs `_ros_glue.py`).
8. **Compact code**. Użytkownik wprost preferuje krótszy kod nad rozbudowaną abstrakcją. "Three similar lines is better than premature abstraction".
9. **Dokument i kod zawsze spójne**. Jak coś jest TODO w kodzie — flagujesz w ARCHITECTURE.md albo w odpowiednim configu (`TODO_TUNE`).

## Język i styl

- **Komunikacja z użytkownikiem**: po polsku, zwięźle, bez emoji.
- **Komentarze i docstringi w kodzie**: po angielsku (zgodnie z konwencją OSS, dla potencjalnego ujawnienia).
- **Nazwy zmiennych**: angielskie (snake_case Python, camelCase ROS messages).
- **Dokumentacja techniczna w `docs/`**: po polsku. ARCHITECTURE.md jest w tej formie.
- Użytkownik nie znosi spolszczonych anglicyzmów typu "ramki", "strumyk", "owijka". Woli oryginalne angielskie słowa niż niezgrabne kalki. Wyjątek: "układ współrzędnych" zamiast "frame" — to *była* świadoma decyzja, bo "ramka" mu nie pasowała.

## Czego unikać

- **Nie komplikuj**. Użytkownik wprost prosił o kompaktowy, czytelny kod. Każda nowa abstrakcja wymaga uzasadnienia, w nie "jakby to się przydało w przyszłości".
- **Nie dorzucaj logging na każdym kroku**. INFO dla ważnych zdarzeń, DEBUG domyślnie wyłączone.
- **Nie wdrażaj rzeczy z TODO bez pytania użytkownika**. Lista TODO jest świadoma — IK, RANSAC, MissionStatus publishing, blackboard cycle counter — wszystko czeka aż user da znak.
- **Nie commituj nic samodzielnie**. User decyduje kiedy `git commit`.
- **Nie odpalaj symulatorów z poziomu Bash**. MuJoCo viewer (`unitree_mujoco`) i Isaac Sim wymagają GUI; user uruchamia ręcznie.
- **Nie dotykaj `keyframes.py` (P0..P6)**. Te wartości pochodzą z kalibracji na realnym robocie i są walidowane. Można dodać nowe sekwencje, nie zmieniać istniejących.

## Pytania otwarte do zadania użytkownikowi przy starcie

1. W której fazie aktualnie jesteśmy (0 / 1 / 2)? Jeśli niejasne, sprawdź "Stan obecny" — pierwszy `❌` definiuje fazę.
2. Hardware: czy host ma NVIDIA GPU (`nvidia-smi`), czy jest to ARM/Apple Silicon Parallels (zostajemy CPU-only do Fazy 2)?
3. Czy paczki `unitree_ros2` (`unitree_hg`, `unitree_api`) są zbudowane w workspace?
4. (Faza 1) Czy `unitree_mujoco` jest sklonowane i zbuduje się na host architecture?
5. (Faza 2) Czy mamy konto cloud GPU, na której instancji budżet?

## Najczęstsze problemy które mogą wystąpić

- **`unitree_hg` nie buduje się** — brakuje `rosidl_generator_dds_idl` (`apt install ros-humble-rosidl-generator-dds-idl`). Plus upewnij się że nie ciągniesz mieszanki distro w PATH.
- **AprilTag node nie publikuje detekcji** — sprawdź czy kamera publikuje `camera_info` z poprawnym intrinsic. W MuJoCo trzeba ustawić ręcznie; Isaac robi automatycznie.
- **Nav2 wjeżdża w kostkę** — najczęściej zbyt mały footprint inflation. Tune w `nav2_params.yaml`.
- **AMCL nie konwerguje** — daj initial pose w rviz, albo skonfiguruj `set_initial_pose`.
- **MuJoCo renderowanie wolne** — virtio GPU w Parallels daje ~5-10 FPS. Wystarcza do `apriltag_ros`, ale przy dynamicznym docku może wymagać GPU passthrough. Headless mode dla samego physics jest szybki.
- **`unitree_mujoco` ROS2 bridge nie publikuje** — sprawdź `ROS_DOMAIN_ID` i czy używasz tego samego RMW (cyclonedds zalecane przez Unitree).
- **Robot leży w MuJoCo, ramiona się ruszają** — oczekiwane przed Fazą 1.3. `unitree_mujoco` nie ma walking controllera, my (do Fazy 1.3) sterujemy tylko ramionami przez arm_sdk. `/cmd_vel` ląduje w `sim_cmd_vel_bridge_node` (Linux TF), nie w mac MuJoCo motors. Po Fazie 1.3 robot będzie stał i chodzić.
- **Robot oscyluje i wpada w drgawki gdy włączymy `walking_controller_node`** — oczekiwane: per-joint PD bez gravity feedforward / whole-body coordination nie utrzyma G1. Zostawiamy kod jako wireup, **nie uruchamiamy** w default launch.
- **Robot oscyluje też z `walking_policy_node` (RL pretrained)** — pretrained `motion.pt` z `unitree_rl_gym` jest trenowany dla free-standing walking, nie radzi sobie z arm forces (P0..P6 sequence = 20+ Nm na pelvis). Także zostaje jako wireup. **Aktualne rozwiązanie**: pelvis weld pin w mac scene.xml (sim only) — patch w `PELVIS_WELD_PATCH.md`. Robot stoi jak posąg, arm tasks działają z prawdziwą fizyką.
- **`grasp_verified=true` w MuJoCo bez kartonu** — false-positive: gdy robot leży na boku (typowe, bo bipedal bez kontrolera = upadek), grawitacja boczna napina ramiona, `tau_est` rośnie powyżej `grasp_tau_threshold_nm` (default 1.5 Nm). W realu (robot stojący prosto) grawitacja działa wzdłuż osi ramienia i nie powoduje fałszywego dodatka. Dla testów MuJoCo można albo zwiększyć próg, albo trzymać robota balansującego (kontroler chodu) zanim odpalisz `pick`.
- **`release_verified=false` w MuJoCo bez kartonu** — symetryczny false-negative: `verify_release` oczekuje że τ_est wróci do baseline z dokładnością `< threshold`. Sekwencja `place` zmienia konfigurację stawów (P5→ZERO), więc grawitacja boczna na leżącym robocie produkuje **inne** τ niż baseline, nie podobne. `place` zwróci `success=False, status=ABORTED` mimo że ramiona wykonały pełną sekwencję. Tak jak grasp false-positive: logika verifier'a jest poprawna, bug jest w setupie testowym.
- **`unitree_sdk2py` po stronie publishera nie wpisuje type hash** — `[WARN] rmw_cyclonedds_cpp: Failed to parse type hash for topic 'rt/lowstate' from USER_DATA '(null)'`. Kosmetyka. Type matching idzie po nazwie typu i działa; brakuje tylko ROS2-specific extra integrity check.
- **`unitree_sdk2py` nie inkrementuje pola `LowState.tick`** — zostaje 0 cały czas. Nikt z naszego kodu tego nie czyta, więc niegroźne. Pamiętaj jeśli kiedykolwiek dorobimy heartbeat-watchdog opartego o tick.
- **Isaac ROS2 bridge (Faza 2) nie publikuje** — sprawdź `ROS_DOMAIN_ID` (zmienna środowiskowa przed startem Isaac) i tunelowanie sieciowe jeśli na cloud.

## Linki referencyjne

- ROS2 Humble docs: <https://docs.ros.org/en/humble/>
- MuJoCo: <https://mujoco.readthedocs.io/>
- `unitree_mujoco`: <https://github.com/unitreerobotics/unitree_mujoco>
- `unitree_ros2`: <https://github.com/unitreerobotics/unitree_ros2>
- Isaac Sim: <https://docs.isaacsim.omniverse.nvidia.com/latest/>
- Isaac Sim ROS2 bridge: <https://docs.isaacsim.omniverse.nvidia.com/latest/ros2_tutorials/index.html>
- Unitree G1 docs: <https://support.unitree.com/home/en/G1_developer>
- nav2: <https://docs.nav2.org/>

---

Pamiętaj: **simulation-first nie znaczy "tylko symulacja"**. Wszystko co robisz w symie ma być przenośne na realnego robota bez zmian poza `sim_cmd_vel_bridge_node` (jedyny węzeł sim-only — kinematic, używany w Fazie 0 i jako alternatywa fizyki w fazach dalszych). Cała reszta — mission BT, dock, arm skills, safety — jest tym samym kodem co na realu, bo MuJoCo Unitree mówi tym samym `unitree_hg/LowCmd, LowState` co prawdziwy robot.
