# Baza wiedzy o fazach projektu

Każda faza ma jeden plik `phase_X_Y.md` z:

- **Cel** — co dana faza miała walidować.
- **Co osiągnięto** — tabela komponentów, stan przed/po.
- **Architektura** — diagram przepływu danych między mac MuJoCo a Linux ROS2.
- **Algorytm krok po kroku** — opis działania kluczowego pipeline'u (np. dokowania, picku).
- **Lessons learned** — niespodzianki natrafione w trakcie, które warto pamiętać przy kolejnych fazach.
- **Co zostało odłożone** — świadome decyzje o niewyjaśnionych elementach (czekają na kolejne fazy).
- **Pliki dotknięte** — Linux side + mac side, dla orientacji co czytać.

## Indeks faz

| Faza | Plik | Status | Streszczenie |
|---|---|---|---|
| 1.0 | (nie spisana) | ✅ | Real arm pick/place na MuJoCo bridge, dock LIDAR_LINE z RANSAC fixturem, mission BT cycle z fake nav/dock + real arm + real retreat. |
| 1.1 | [phase_1_1.md](phase_1_1.md) | ✅ | AprilTag w mac scene XML + kamera head_cam + pupil_apriltags detector na DDS. Linux dock_action_server MODE_APRILTAG end-to-end (cv2.solvePnPGeneric + IPPE_SQUARE). Mac kinematic mocap movement (subscribe `/cmd_vel`, integruje pelvis_anchor pos/yaw). Pełen mission BT cycle przechodzi z dock A real + dock B `amcl_only` jako diagnostyczny passthrough. |
| 1.2 | [phase_1_2.md](phase_1_2.md) | ✅ | Real LiDAR sensor w mac scene XML (`lidar_site` w pelvis, 360-ray `mj_ray()` programmatic scan). Mac publikuje `rt/scan` jako `sensor_msgs/LaserScan` @ ~7 Hz. Linux `dock_action_server` MODE_LIDAR_LINE konsumuje plus zbiega do biurka B z RANSAC line fit. Pełen mission BT cycle z dock A APRILTAG + dock B LIDAR_LINE z fizycznym ruchem. |
| 1.3 | (planowana) | ❌ | Walking controller (`/cmd_vel` → motor commands na nogi). Bramka dla 1.4–1.6. |
| 1.4 | (planowana) | ❌ | Mapowanie LiDARem (`slam_toolbox`). Wymaga 1.2 + 1.3. |
| 1.5 | (planowana) | ❌ | AMCL + nav2 + real `navigate_to_pose`. Wymaga 1.4. |
| 1.6 | (planowana) | ❌ | Pełen mission BT cycle z prawdziwym ruchem A↔B przez nav2. Wymaga 1.5 + 1.1. |
| 2.0 | (planowana) | ❌ | Walidacja foto-realistyczna w Isaac Sim na cloud GPU. |
