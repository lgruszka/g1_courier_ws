"""Sim bridge configuration.

ROBOT_SCENE defaults to the bundled scene_courier.xml in this package's
assets/. Override via env var or CLI for custom scenes.
"""
import os

ROBOT = "g1"

# Path to scene XML — defaults to bundled courier scene.
_HERE = os.path.dirname(os.path.abspath(__file__))
ROBOT_SCENE = os.environ.get(
    "G1_COURIER_SCENE",
    os.path.join(_HERE, "assets", "scene_courier.xml"),
)

DOMAIN_ID = int(os.environ.get("ROS_DOMAIN_ID", "0"))
# Linux network interface — typically "lo" for loopback. Override with
# G1_COURIER_INTERFACE env var if you need a specific NIC for DDS.
INTERFACE = os.environ.get("G1_COURIER_INTERFACE", "lo")

USE_JOYSTICK = 0
JOYSTICK_TYPE = "xbox"
JOYSTICK_DEVICE = 0

PRINT_SCENE_INFORMATION = True
ENABLE_ELASTIC_BAND = False

SIMULATE_DT = 0.005
VIEWER_DT = 0.02
