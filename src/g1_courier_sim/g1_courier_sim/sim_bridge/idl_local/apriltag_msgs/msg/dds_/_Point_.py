"""
Hand-written cyclonedds IdlStruct for apriltag_msgs/msg/Point.

apriltag_msgs has its OWN Point type (2D, x and y only) that is different from
geometry_msgs/Point (3D with z). Wire-compatible with the rosidl-generated
Python bindings of apriltag_msgs/Point.
"""

from dataclasses import dataclass

import cyclonedds.idl as idl
import cyclonedds.idl.annotations as annotate
import cyclonedds.idl.types as types


@dataclass
@annotate.final
@annotate.autoid("sequential")
class Point_(idl.IdlStruct, typename="apriltag_msgs.msg.dds_.Point_"):
    x: types.float64
    y: types.float64
