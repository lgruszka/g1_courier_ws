"""
Hand-written cyclonedds IdlStruct for sensor_msgs/msg/Image.

Wire-compatible with sensor_msgs/Image.idl from ROS2 (rmw_cyclonedds_cpp).
Reuses Header_ from unitree_sdk2py.

Field reference (sensor_msgs/Image.msg):
  std_msgs/Header header
  uint32 height
  uint32 width
  string encoding
  uint8 is_bigendian
  uint32 step
  uint8[] data
"""

from dataclasses import dataclass

import cyclonedds.idl as idl
import cyclonedds.idl.annotations as annotate
import cyclonedds.idl.types as types

from unitree_sdk2py.idl.std_msgs.msg.dds_._Header_ import Header_


@dataclass
@annotate.final
@annotate.autoid("sequential")
class Image_(idl.IdlStruct, typename="sensor_msgs.msg.dds_.Image_"):
    header: Header_
    height: types.uint32
    width: types.uint32
    encoding: str
    is_bigendian: types.uint8
    step: types.uint32
    data: types.sequence[types.uint8]
