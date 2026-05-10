"""
Hand-written cyclonedds IdlStruct for apriltag_msgs/msg/AprilTagDetection.

Wire-compatible with apriltag_msgs/AprilTagDetection.idl as published by
ROS2 rmw_cyclonedds_cpp. Uses apriltag_msgs's OWN Point type (2D), NOT
geometry_msgs/Point (3D).
"""

from dataclasses import dataclass

import cyclonedds.idl as idl
import cyclonedds.idl.annotations as annotate
import cyclonedds.idl.types as types

from ._Point_ import Point_


@dataclass
@annotate.final
@annotate.autoid("sequential")
class AprilTagDetection_(idl.IdlStruct, typename="apriltag_msgs.msg.dds_.AprilTagDetection_"):
    family: str
    id: types.int32
    hamming: types.int32
    goodness: types.float32
    decision_margin: types.float32
    centre: Point_
    corners: types.array[Point_, 4]
    homography: types.array[types.float64, 9]
