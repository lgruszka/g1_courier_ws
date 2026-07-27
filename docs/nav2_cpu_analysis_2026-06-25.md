# Analiza obciążenia CPU stosu nawigacyjnego

Data pomiaru: 2026-06-25

> **⚠️ AKTUALIZACJA 2026-07-27 — przeczytaj przed wdrażaniem wniosków z tego
> dokumentu.** Nowy pomiar na Jetsonie (8 rdzeni, sonda `tools/cpu_probe.py`) pokazał
> **40% wysycenia na postoju i 44% podczas jazdy** — nawigacja kosztuje tylko
> **+25 pp**. Jetson **nie jest wysycony**, więc optymalizacje kosztem funkcjonalności
> (wyłączanie kamery, upraszczanie filtra chmury, obniżanie częstotliwości nav2)
> **nie mają dziś uzasadnienia**. Rzeczywistym pułapem jest jeden proces Pythona:
> `odom_tf_relay` na ~62% **jednego** rdzenia (`/dog_odom` @ 1015 Hz).
> Pełne liczby, rozbicie per proces i wnioski: `docs/etap_a_utwardzenie_nav.md` §4.4.

Zakres:
- analiza live stosu uruchomionego z `./run_nav2.sh`
- bez modyfikacji kodu i konfiguracji
- nacisk na top-consumerów CPU, topiki i częstotliwości publikacji

Uwaga:
- podwójny publisher na `/scan` jest w tym środowisku zamierzony
- wg informacji od operatora publisher `/scan` uruchamiany razem ze stackiem bywa niesprawny, więc drugi publisher pełni rolę fallbacku
- wnioski poniżej traktują to jako świadomy workaround, nie jako przypadkowy błąd

## 1. Co uruchamia `run_nav2.sh`

`run_nav2.sh` buduje pakiety i uruchamia:

- `ros2 launch g1_courier_bringup real.launch.py`
- `map:=/home/unitree/maps/lab.yaml`
- `enable_mission:=false`
- `enable_camera:=true`

To oznacza, że razem z Nav2 uruchamiane są także nody poboczne, niekoniecznie niezbędne do samej nawigacji:

- `d435i_node`
- `apriltag_node`
- `dock_action_server`
- `pick_action_server`
- `place_action_server`
- `navigate_proxy`
- `retreat_action_server`
- `cmd_vel_arbiter`
- `unitree_cmd_vel_bridge_node`
- `lowstate_to_joint_states`
- `robot_state_publisher`
- `odom_tf_relay`
- `parcel_cloud_filter`
- `pointcloud_to_laserscan`

## 2. Najwięksi konsumenci CPU

Pomiar z hosta dla procesów potomnych `ros2 launch ... real.launch.py`:

- `pick_action_server`: ok. 80% CPU
- `place_action_server`: ok. 79% CPU
- `d435i_node`: ok. 76% CPU
- `parcel_cloud_filter`: ok. 72%
- `lowstate_to_joint_states`: ok. 70%
- `odom_tf_relay`: ok. 68%
- `nav2_container`: ok. 59%
- `apriltag_node`: ok. 15%
- `pointcloud_to_laserscan`: ok. 10%
- `unitree_cmd_vel_bridge_node`: ok. 10%
- `robot_state_publisher`: ok. 10%

Wniosek:
- sam `nav2_container` jest ciężki, ale nie jest największym źródłem obciążenia
- znaczną część CPU zjada otoczenie Nav2: sensory, filtry, bridge, TF i action serwery ramion

## 3. Graf ROS 2 i kluczowe nody

Potwierdzone aktywne nody związane z nawigacją:

- `/amcl`
- `/planner_server`
- `/controller_server`
- `/behavior_server`
- `/bt_navigator`
- `/waypoint_follower`
- `/velocity_smoother`
- `/global_costmap/global_costmap`
- `/local_costmap/local_costmap`
- `/map_server`
- `/nav2_container`

Dodatkowo aktywne i istotne dla obciążenia:

- `/parcel_cloud_filter`
- `/pointcloud_to_laserscan`
- `/pointcloud_to_laserscan`
- `/odom_tf_relay`
- `/lowstate_to_joint_states`
- `/robot_state_publisher`
- `/d435i_node`
- `/apriltag_node`
- `/dock_action_server`
- `/pick_action_server`
- `/place_action_server`

## 4. Zmierzone częstotliwości topiców

Pomiary live:

- `/livox/lidar`: ok. 10 Hz
- `/livox/lidar_filtered`: ok. 3.6 Hz
- `/scan`: ok. 3.6 Hz
- `/tf`: ok. 420 Hz
- `/dog_odom`: ok. 856 Hz
- `/lowstate`: ok. 770 Hz
- `/joint_states`: ok. 160 Hz
- `/camera/color/image_raw`: ok. 1.7-2.4 Hz w obserwowanym oknie
- `/camera/depth/image_rect_raw`: ok. 2.35 Hz w obserwowanym oknie
- `/camera/color/camera_info`: ok. 3.6 Hz w obserwowanym oknie
- `/detections`: ok. 4.9-5.2 Hz

Najmocniejszy sygnał z tych pomiarów:

- `dog_odom -> odom_tf_relay -> /tf` działa z bardzo wysoką częstotliwością
- `lowstate -> lowstate_to_joint_states -> robot_state_publisher -> /tf` też jest bardzo gęstą ścieżką
- to wygląda na istotne źródło obciążenia CPU, które nie musi dawać proporcjonalnej korzyści jakościowej Nav2

## 5. Przepływ danych i przepustowość

Pomiary `ros2 topic bw`:

- `/livox/lidar`: ok. 5.2 MB/s
- `/camera/color/image_raw`: ok. 1.2-1.5 MB/s
- `/camera/depth/image_rect_raw`: ok. 0.9-1.0 MB/s

Wniosek:
- głównym strumieniem danych jest lidar
- kamera dokłada zauważalny transfer i obróbkę CPU, mimo że nie wszystkie publikowane topiki są realnie używane

## 6. Ścieżka lidar -> scan

Potwierdzona ścieżka:

- `/livox/lidar` publikuje `livox_lidar_publisher`
- `/parcel_cloud_filter` subskrybuje `/livox/lidar` i publikuje `/livox/lidar_filtered`
- jeden `pointcloud_to_laserscan` subskrybuje `/livox/lidar_filtered`
- drugi `pointcloud_to_laserscan` subskrybuje `/livox/lidar`
- oba mogą publikować na `/scan`

Konsekwencje:

- workaround z podwójnym `/scan` ma koszt CPU i koszt semantyczny
- nawet jeśli jest celowy, Nav2 i docking pracują wtedy na topicu, który może pochodzić z dwóch alternatywnych źródeł
- jeśli publisher z launch stacku nie publikuje stabilnie, warto traktować to jako problem do usunięcia u źródła, bo fallback sam w sobie podnosi złożoność diagnozy

## 7. Kto używa `/scan`

`/scan` ma 2 publisherów i co najmniej 5 subskrybentów:

- `rviz`
- `dock_action_server`
- `amcl`
- `local_costmap`
- `global_costmap`

To oznacza, że każda niestabilność lub nadmiar pracy przy generowaniu `/scan` wpływa jednocześnie na:

- lokalizację
- costmapy
- docking
- narzędzia operatorskie

## 8. Kto publikuje `/tf`

Publisherzy `/tf`:

- `odom_tf_relay`
- `apriltag_node`
- `amcl`
- `robot_state_publisher`

Subskrybentów `/tf` jest dużo, m.in.:

- wiele `transform_listener_impl_*`
- `parcel_cloud_filter`
- `bt_navigator`

Wniosek:
- wysoka częstotliwość TF rozlewa koszt na sporą część grafu
- szczególnie kosztowny wydaje się duet:
  - `odom_tf_relay`
  - `robot_state_publisher`

## 9. Kamera i AprilTag

Runtime `d435i_node`:

- `fps = 30`
- `width = 640`
- `height = 480`
- `publish_depth_topic = true`
- `publish_legacy_color_topic = true`
- `publish_legacy_camera_info_topic = true`
- `align_depth_to_color = true`

Potwierdzone użycie topiców:

- `/camera/color/image_raw` jest używany przez `apriltag_node`
- `/camera/color/camera_info` jest używany przez `apriltag_node`
- `/detections` i `/camera_info` są używane przez `dock_action_server`

Jednocześnie:

- `/camera/image_raw` ma 0 subskrybentów
- `/camera/depth/image_rect_raw` ma 0 subskrybentów

Wniosek:
- ścieżka RGB jest funkcjonalnie potrzebna do AprilTagów i dockingu
- depth i legacy topics wyglądają na narzut publikacyjny w tym scenariuszu

## 10. Nody ramion a wpływ na nawigację

`pick_action_server` i `place_action_server`:

- subskrybują `/lowstate`
- publikują `/arm_sdk`
- każdy zjada około 80% CPU

To ważne, bo:

- są uruchamiane mimo `enable_mission:=false`
- nie są krytyczne dla podstawowej nawigacji mapowej
- konkurują o CPU z `nav2_container`, costmapami, AMCL i sensor pipeline

Wniosek:
- to prawdopodobnie największy pojedynczy czynnik pośrednio obniżający jakość nawigacji przez brak budżetu CPU

## 11. Ocena konfiguracji samego Nav2

Z `nav2_params.yaml`:

- `controller_frequency: 20.0`
- `local_costmap.update_frequency: 5.0`
- `local_costmap.publish_frequency: 2.0`
- `global_costmap.update_frequency: 1.0`
- `global_costmap.publish_frequency: 1.0`
- `behavior_server.cycle_frequency: 10.0`
- `velocity_smoother.smoothing_frequency: 20.0`

Ocena:
- te wartości są rozsądne
- nie widać tu oczywistego przetaktowania Nav2
- problem wydajnościowy leży raczej przed Nav2 i obok Nav2 niż w jego podstawowych częstotliwościach

## 12. Główne hipotezy przyczyny degradacji jakości nawigacji

Najbardziej prawdopodobne źródła problemu, w kolejności ważności:

1. Nadmierne obciążenie CPU przez nody niezwiązane bezpośrednio z samą nawigacją:
   - `pick_action_server`
   - `place_action_server`
   - `d435i_node`
   - `parcel_cloud_filter`

2. Zbyt gęsta ścieżka odometrii i TF:
   - `/dog_odom` ok. 856 Hz
   - `/tf` ok. 420 Hz
   - `/lowstate` ok. 770 Hz
   - `/joint_states` ok. 160 Hz

3. Wysoki koszt pythonowych bridge/filter nodes:
   - `odom_tf_relay`
   - `lowstate_to_joint_states`
   - `parcel_cloud_filter`

4. Dodatkowa ścieżka kamerowa i AprilTag przy aktywnym `enable_camera:=true`

5. Celowy fallback z dwoma publisherami `/scan`, który upraszcza obejście awarii, ale zwiększa koszt i utrudnia jednoznaczną analizę źródła danych

## 13. Priorytety optymalizacji

Bez wchodzenia jeszcze w zmiany kodu, priorytety do dalszej pracy wyglądają tak:

1. Ograniczyć lub warunkować uruchamianie nodów niepotrzebnych do bazowej nawigacji:
   - szczególnie `pick_action_server` i `place_action_server`

2. Zredukować częstotliwość ścieżek:
   - `/dog_odom`
   - `/tf`
   - `/lowstate`
   - `/joint_states`

3. Sprawdzić, czy `odom_tf_relay` i `lowstate_to_joint_states` mogą publikować rzadziej lub tylko przy zmianie stanu

4. Rozdzielić profil "nav-only" od profilu "nav+docking+vision"

5. Docelowo naprawić publisher `/scan` startujący razem ze stackiem, aby nie polegać stale na podwójnym źródle

6. Ograniczyć koszt kamery:
   - depth off, jeśli nieużywany
   - legacy topics off, jeśli nieużywane
   - uruchamianie `apriltag_node` tylko gdy docking jest potrzebny

## 14. Podsumowanie końcowe

Najważniejsze wnioski:

- największym problemem nie jest sama konfiguracja częstotliwości Nav2
- CPU jest mocno obciążane przez nody poboczne i pomocnicze uruchamiane razem ze stackiem
- szczególnie kosztowne są action serwery ramion, kamera, filtr pointclouda oraz ścieżka TF/odometria
- `nav2_container` jest ciężki, ale nie dominuje całego obciążenia
- workaround z podwójnym `/scan` jest zrozumiały operacyjnie, ale ma koszt i powinien pozostać rozwiązaniem tymczasowym

Praktyczna interpretacja:

- jeśli celem jest poprawa jakości nawigacji, największy zwrot prawdopodobnie da odciążenie hosta z nodów niezwiązanych z bazową nawigacją oraz zbicie częstotliwości `odom/lowstate/tf`
- strojenie samych parametrów Nav2 powinno być dopiero kolejnym etapem

## 15. Zmiana `/lowstate` -> `/lf/lowstate`

Po pierwszym raporcie wykonano zmianę źródła `LowState` na `/lf/lowstate`.

Potwierdzone odbiorniki `/lf/lowstate`:

- `lowstate_to_joint_states`
- `pick_action_server`
- `place_action_server`

Pośrednia ścieżka do TF nadal istnieje:

- `/lf/lowstate` -> `lowstate_to_joint_states` -> `/joint_states`
- `/joint_states` -> `robot_state_publisher` -> `/tf`

To oznacza:

- `/lf/lowstate` nadal nie jest wejściem bezpośrednio do Nav2
- ale pośrednio wpływa na obciążenie przez `joint_states` i `robot_state_publisher`

## 16. Porównanie przed i po zmianie

### 16.1 Porównanie CPU

Porównanie snapshotów wykonanych na tym samym hoście, dla stosu uruchomionego z `run_nav2.sh`.

Przed zmianą:

- `lowstate_to_joint_states`: ok. 69-71% CPU
- `pick_action_server`: ok. 80-81% CPU
- `place_action_server`: ok. 79-80% CPU
- `robot_state_publisher`: ok. 9.8-9.9% CPU
- `odom_tf_relay`: ok. 68-69% CPU
- `nav2_container`: ok. 59%

Po zmianie:

- `lowstate_to_joint_states`: ok. 9.8% CPU
- `pick_action_server`: ok. 9.7% CPU
- `place_action_server`: ok. 9.8% CPU
- `robot_state_publisher`: ok. 5.1% CPU
- `odom_tf_relay`: ok. 82.4% CPU
- `nav2_container`: ok. 63.6%

Różnica:

- `lowstate_to_joint_states`: spadek o ok. 60 punktów procentowych
- `pick_action_server`: spadek o ok. 70 punktów procentowych
- `place_action_server`: spadek o ok. 70 punktów procentowych
- `robot_state_publisher`: spadek prawie o połowę

Wniosek:

- zmiana na `/lf/lowstate` znacząco odciążyła całą ścieżkę zależną od `LowState`
- wcześniejsze wysokie obciążenie rzeczywiście było napędzane zbyt gęstym strumieniem `lowstate`

### 16.2 Porównanie częstotliwości

Przed zmianą:

- `/lowstate`: ok. 770 Hz
- `/joint_states`: ok. 160 Hz
- `/tf`: ok. 420 Hz
- `/dog_odom`: ok. 856 Hz

Po zmianie:

- `/lf/lowstate`: ok. 20.0 Hz
- `/joint_states`: ok. 20.0 Hz
- `/tf`: ok. 637-652 Hz
- `/dog_odom`: ok. 955-991 Hz

Interpretacja:

- zmiana na `/lf/lowstate` obcięła częstotliwość wejścia `LowState` z setek Hz do ok. 20 Hz
- `joint_states` spadł do tego samego rzędu, co tłumaczy duży spadek CPU w `lowstate_to_joint_states` i `robot_state_publisher`
- `/tf` nie spadł, bo nadal jest silnie napędzany przez innych publisherów, szczególnie:
  - `odom_tf_relay`
  - `apriltag_node`
  - `amcl`
  - `robot_state_publisher`
- `/dog_odom` pozostał bardzo szybki, a nawet był wyższy niż w poprzednim snapshotcie

## 17. Co zmieniło się w rankingu top-consumerów

Przed zmianą na `/lf/lowstate` duży udział w obciążeniu miały:

- `pick_action_server`
- `place_action_server`
- `lowstate_to_joint_states`

Po zmianie głównymi top-consumerami zostały:

- `d435i_node`: ok. 101%
- `parcel_cloud_filter`: ok. 96%
- `odom_tf_relay`: ok. 82%
- `nav2_container`: ok. 64%

Wniosek:

- zmiana nie rozwiązała całości problemu CPU
- usunęła jednak jeden z największych i najłatwiejszych do wskazania kosztów
- po tej zmianie głównym kandydatem do dalszej optymalizacji jest już nie `lowstate`, tylko:
  - kamera
  - filtr pointclouda
  - ścieżka `dog_odom -> odom_tf_relay -> /tf`

## 18. Changelog raportu

### 2026-06-25, aktualizacja po zmianie na `/lf/lowstate`

Dodano:

- weryfikację aktualnych subskrybentów `/lf/lowstate`
- porównanie CPU przed i po zmianie
- porównanie częstotliwości przed i po zmianie
- aktualizację rankingu top-consumerów

Najważniejszy nowy wniosek:

- przełączenie na `/lf/lowstate` znacząco poprawiło obciążenie w ścieżce `LowState`
- aktualnym dominującym kosztem pozostały `d435i_node`, `parcel_cloud_filter`, `odom_tf_relay` i `nav2_container`

### 2026-06-29, filtr paczki 3D -> 2D (`parcel_cloud_filter` usunięty)

Zdjęcie kosztu `parcel_cloud_filter` (zmierzone ~96% CPU) — filtrowanie przeniesione
ZA konwersję na scan (§6, §7).

Co zrobiono:

- usunięto węzeł `parcel_cloud_filter` (proces Python: na każdą ramkę pełna kopia
  bufora ~20 tys. punktów, promocja do float64, mnożenie macierzowe całej chmury,
  plus dodatkowy hop DDS pełnej chmury do `pointcloud_to_laserscan`)
- nowy tor: `chmura -> pointcloud_to_laserscan -> /scan_raw -> laser_filters/LaserScanBoxFilter -> /scan`
- filtr 2D (C++) tnie ~722 wiązki skanu zamiast ~20 tys. punktów; znika jeden hop DDS
- box wycinający bez zmian (base_link, te same granice co dawny cropbox 3D), teraz w `config/scan_box_filter.yaml`
- przełącznik `filter_parcel` zachowany (`false` => `pointcloud_to_laserscan` publikuje `/scan` wprost, bez filtra)

QoS (zweryfikowane lokalnie `ros2 topic info --verbose`):

- `/scan_raw`: p2l publikuje BEST_EFFORT, filtr subskrybuje BEST_EFFORT — zgodne
- `/scan`: filtr publikuje RELIABLE — kompatybilne z każdym dotychczasowym odbiorcą
  (publisher RELIABLE obsługuje subskrybentów i best_effort, i reliable)

Kompromis (świadomy):

- filtr 3D usuwał punkty paczki PRZED rzutowaniem, więc scan pokazywał ścianę za paczką;
  filtr 2D kasuje całą wiązkę z paczką, więc w wąskim stożku z przodu ściana za paczką
  bywa tracona (wiązka = inf). Dla AMCL (pełny skan ~360°) akceptowalne. Rollback:
  `git revert` albo `filter_parcel:=false`.

Do zmierzenia na Jetsonie:

- CPU `parcel_cloud_filter` (było ~96%) powinno zniknąć z rankingu
- `pointcloud_to_laserscan` może lekko wzrosnąć (czyta teraz pełną chmurę zamiast przefiltrowanej)

### 2026-07-03, filtr paczki 2D -> cropbox 3D (pcl_ros) + fix dubla publisherów /scan_raw

Diagnoza „robot z paczką nawiguje kiepsko" na nagraniach z robota
(idle/walking × hands_normal/hands_up, 6 bagów). Trzy znalezione przyczyny:

1. **Sekcja yaml filtra 2D nie pasowała do nazwy węzła** (`scan_to_scan_filter_chain`
   vs `scan_box_filter`) — parametry się nie ładowały, łańcuch był pusty, `/scan`
   zawierał surowe punkty paczki. Naprawione wildcardem `/**:` (c9057a6).
2. **Dwa publishery na `/scan_raw`** — `run_pointcloud_laserscan.sh` (stary pas
   −1.65…0.7) obok p2l z launcha (−0.5…0.8). W bagach walking 100% stampów
   zdublowanych: AMCL dostawał naprzemiennie dwa różne pasy wysokości, 2× rate.
   Fix: skrypt publikuje na `/scan_raw_debug` + wyrównany pas.
3. **Filtr 2D kasuje całe wiązki** (p2l bierze najbliższy punkt na kierunek) —
   dziura ~26° z przodu przy niesieniu; do tego ręce PODCZAS CHODU (balans)
   sięgają 0.46–0.59 m i szerzej niż ±0.25 => przecieki za box.

Zmiana: powrót do cięcia w 3D PRZED projekcją, ale w C++ —
`pcl_ros filter_crop_box_node` (`negative:true`, box w natywnej ramce livox,
`config/parcel_cropbox.yaml`) -> `/livox/lidar_filtered` -> p2l -> `/scan`.
Wiązki za paczką pokazują ścianę widzianą ponad nią (bez dziury). Usunięty
łańcuch `laser_filters`; dep `laser_filters` -> `pcl_ros` (na Jetsonie:
`sudo apt install ros-jazzy-pcl-ros`).

Walidacja (symulacja cropbox+p2l na chmurach z bagów; przecieki = wiązki
<0.6 m w ±60° z przodu, median/skan):

| bag | bez filtra | box 0.45/±0.25 (stary) | box 0.60/±0.35 (nowy) |
|---|---|---|---|
| idle_hands_up | 49 | 0 | 0 |
| walking_hands_up | 75 | 29 | 0 |
| walking_hands_up2 | 68 | 18 | 0 |
| walking_hands_normal | 0 (p90 47) | 0 (p90 47) | 0 (p90 0) |

Dziura z przodu identyczna we wszystkich wariantach (23–30° = baseline
pomieszczenia) — cropbox 3D nie dokłada nic, filtr 2D dokładał +26°.

Przy okazji (dokowanie): po wymianie d435i_node na realsense2_camera (8c7b813)
nikt nie publikował `/camera_info`, którego słuchał dock (do tego subskrypcja
TRANSIENT_LOCAL niekompatybilna z volatile realsense) — dock jechał na
fallbackowych intrinsics z sima (fx=415.69 vs realne ~617, ~33% błąd skali
odległości). Fix: `docking.yaml` camera_info_topic ->
`/camera/camera/color/camera_info`, subskrypcja QoS -> sensor_data.

Ryzyko do przetestowania: box sięga bliżej niż `lidar.target_distance` doku
(0.30) — dokowanie LIDAR_LINE z paczką może głodzić RANSAC na finalnym
podejściu (uwaga: te wiązki i tak fizycznie zasłania niesiona paczka).
W razie problemu: poszerzyć okno kątowe fitu albo `filter_parcel:=false` na
czas testu.
