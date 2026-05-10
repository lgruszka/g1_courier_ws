"""
Hand-written cyclonedds IdlStruct for apriltag_msgs/msg/AprilTagDetectionArray.

Wire-compatible with apriltag_msgs/AprilTagDetectionArray.msg as published by
ROS2 rmw_cyclonedds_cpp. Reuses Header_ from unitree_sdk2py.
"""

from dataclasses import dataclass

import cyclonedds.idl as idl
import cyclonedds.idl.annotations as annotate
import cyclonedds.idl.types as types

from unitree_sdk2py.idl.std_msgs.msg.dds_._Header_ import Header_

from ._AprilTagDetection_ import AprilTagDetection_


@dataclass
@annotate.final
@annotate.autoid("sequential")
class AprilTagDetectionArray_(idl.IdlStruct, typename="apriltag_msgs.msg.dds_.AprilTagDetectionArray_"):
    header: Header_
    detections: types.sequence[AprilTagDetection_]
