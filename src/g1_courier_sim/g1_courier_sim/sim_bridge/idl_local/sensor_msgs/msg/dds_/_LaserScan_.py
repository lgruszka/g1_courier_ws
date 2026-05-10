"""
Hand-written cyclonedds IdlStruct for sensor_msgs/msg/LaserScan.

Wire-compatible with sensor_msgs/LaserScan.idl as published by ROS2
rmw_cyclonedds_cpp. All scalar fields are float32 (the .idl uses `float`,
not `double`). Reuses Header_ from unitree_sdk2py.
"""

from dataclasses import dataclass

import cyclonedds.idl as idl
import cyclonedds.idl.annotations as annotate
import cyclonedds.idl.types as types

from unitree_sdk2py.idl.std_msgs.msg.dds_._Header_ import Header_


@dataclass
@annotate.final
@annotate.autoid("sequential")
class LaserScan_(idl.IdlStruct, typename="sensor_msgs.msg.dds_.LaserScan_"):
    header: Header_
    angle_min: types.float32
    angle_max: types.float32
    angle_increment: types.float32
    time_increment: types.float32
    scan_time: types.float32
    range_min: types.float32
    range_max: types.float32
    ranges: types.sequence[types.float32]
    intensities: types.sequence[types.float32]
