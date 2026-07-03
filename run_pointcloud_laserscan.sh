# Reczny/debugowy pointcloud_to_laserscan — publikuje na /scan_raw_debug,
# NIGDY na /scan ani inny topik stacku. Powod: real.launch.py ma wlasny tor
# (cropbox 3D -> p2l -> /scan); drugi publisher na tym samym topiku daje
# przeplot skanow (2x rate, rozne pasy wysokosci) i AMCL dostaje na przemian
# dwa rozne "swiaty" — zmierzone na nagraniach 2026-07-03 (100% zdublowanych
# stampow). To narzedzie do ogladania skanu na boku, nie czesc stacku.
# Pas wysokosci trzymaj zgodny z config/pointcloud_to_laserscan.yaml.
ros2 run pointcloud_to_laserscan pointcloud_to_laserscan_node --ros-args \
  -r cloud_in:=/livox/lidar \
  -r scan:=/scan_raw_debug \
  -p target_frame:=livox_frame \
  -p queue_size:=40 \
  -p min_height:=-0.5 \
  -p max_height:=0.8 \
  -p angle_min:=-3.14159 \
  -p angle_max:=3.14159 \
  -p angle_increment:=0.0087 \
  -p scan_time:=0.1 \
  -p range_min:=0.2 \
  -p range_max:=25.0 \
  -p use_inf:=true
