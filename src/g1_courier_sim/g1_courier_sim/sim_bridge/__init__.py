"""Linux-native MuJoCo bridge for G1 Courier sim.

Bundles the mac unitree_sdk2py_bridge code into the workspace. Run via:

    ros2 run g1_courier_sim sim_bridge_node

or via launch:

    ros2 launch g1_courier_sim sim_bridge.launch.py

Side effect at import time: prepend `<share>/sim_bridge/idl_local/` to
sys.path so the hand-written `sensor_msgs` and `apriltag_msgs` IDL
bindings resolve as top-level imports inside `bridge.py`. Done here (not
in bridge.py) so the lookup uses the ament install share path, which
differs between dev (symlink-install) and a release install.
"""
import os
import sys

try:
    from ament_index_python.packages import get_package_share_directory
    _share = get_package_share_directory('g1_courier_sim')
    _idl_local = os.path.join(_share, 'sim_bridge', 'idl_local')
    if os.path.isdir(_idl_local) and _idl_local not in sys.path:
        sys.path.insert(0, _idl_local)
except Exception:
    # ament_index not available — fall back to source-tree relative path.
    _here = os.path.dirname(os.path.abspath(__file__))
    _idl_local = os.path.join(_here, 'idl_local')
    if os.path.isdir(_idl_local) and _idl_local not in sys.path:
        sys.path.insert(0, _idl_local)
