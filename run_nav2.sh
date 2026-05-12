colcon build --packages-select g1_courier_bringup g1_courier_msgs g1_courier_docking g1_courier_arm_skills g1_courier_mission g1_courier_safety
source install/setup.bash
ros2 launch g1_courier_bringup real.launch.py map:=/home/neo/j2s/maps/scenarios_g1_map/thin_05_075_zp0.50_p0.75.yaml
