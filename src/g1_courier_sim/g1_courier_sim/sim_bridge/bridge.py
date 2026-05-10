import mujoco
import numpy as np
import pygame
import sys
import struct
import os
import math
import threading
import time
import http.server

# sys.path setup for hand-written IDL bindings (sensor_msgs, apriltag_msgs)
# is performed in this package's __init__.py via ament_index_python. By the
# time this module is imported, the idl_local directory is on sys.path and
# the top-level imports below resolve correctly.

import cv2
from pupil_apriltags import Detector as AprilTagDetector

from unitree_sdk2py.core.channel import ChannelSubscriber, ChannelPublisher

from unitree_sdk2py.idl.unitree_go.msg.dds_ import SportModeState_
from unitree_sdk2py.idl.unitree_go.msg.dds_ import WirelessController_
from unitree_sdk2py.idl.default import unitree_go_msg_dds__SportModeState_
from unitree_sdk2py.idl.default import unitree_go_msg_dds__WirelessController_
from unitree_sdk2py.idl.std_msgs.msg.dds_._Header_ import Header_
from unitree_sdk2py.idl.std_msgs.msg.dds_._String_ import String_
from unitree_sdk2py.idl.builtin_interfaces.msg.dds_._Time_ import Time_
from unitree_sdk2py.idl.geometry_msgs.msg.dds_._Twist_ import Twist_
from unitree_sdk2py.utils.thread import RecurrentThread

# apriltag_msgs has its OWN 2D Point type — NOT geometry_msgs/Point.
from apriltag_msgs.msg.dds_ import (
    Point_ as AprilTagPoint_,
    AprilTagDetection_,
    AprilTagDetectionArray_,
)

from sensor_msgs.msg.dds_ import LaserScan_, Image_


class _DebugHTTPHandler(http.server.BaseHTTPRequestHandler):
    """Serves the latest head_cam debug frame as JPEG and a small HTML page.

    Avoids cv2.imshow because OpenCV's GUI on macOS requires the Cocoa main
    thread, which mjpython does not expose to Python code reliably.
    """
    bridge = None  # injected by UnitreeSdk2Bridge before the server starts

    _INDEX = (
        b"<!doctype html><html><body style='margin:0;background:#222;'>"
        b"<img id='i' style='width:100%;display:block'>"
        b"<script>setInterval(()=>{document.getElementById('i').src="
        b"'/f.jpg?'+Date.now()},100)</script></body></html>"
    )

    def do_GET(self):
        if self.path == "/" or self.path.startswith("/index"):
            self.send_response(200)
            self.send_header("Content-Type", "text/html")
            self.send_header("Content-Length", str(len(self._INDEX)))
            self.end_headers()
            self.wfile.write(self._INDEX)
            return
        if self.path.startswith("/f.jpg"):
            with self.bridge._debug_lock:
                img = self.bridge._latest_debug_img
            if img is None:
                self.send_response(204)
                self.end_headers()
                return
            ok, buf = cv2.imencode(".jpg", img, [cv2.IMWRITE_JPEG_QUALITY, 80])
            if not ok:
                self.send_response(500)
                self.end_headers()
                return
            data = buf.tobytes()
            self.send_response(200)
            self.send_header("Content-Type", "image/jpeg")
            self.send_header("Content-Length", str(len(data)))
            self.send_header("Cache-Control", "no-store")
            self.end_headers()
            self.wfile.write(data)
            return
        self.send_response(404)
        self.end_headers()

    def log_message(self, *args, **kwargs):
        pass  # silence per-request logging


class _DebugHTTPServer(http.server.ThreadingHTTPServer):
    daemon_threads = True
    allow_reuse_address = True

import config
if config.ROBOT=="g1":
    from unitree_sdk2py.idl.unitree_hg.msg.dds_ import LowCmd_
    from unitree_sdk2py.idl.unitree_hg.msg.dds_ import LowState_
    from unitree_sdk2py.idl.default import unitree_hg_msg_dds__LowState_ as LowState_default
else:
    from unitree_sdk2py.idl.unitree_go.msg.dds_ import LowCmd_
    from unitree_sdk2py.idl.unitree_go.msg.dds_ import LowState_
    from unitree_sdk2py.idl.default import unitree_go_msg_dds__LowState_ as LowState_default

TOPIC_LOWCMD = "rt/lowcmd"
TOPIC_LOWSTATE = "rt/lowstate"
TOPIC_HIGHSTATE = "rt/sportmodestate"
TOPIC_WIRELESS_CONTROLLER = "rt/wirelesscontroller"
TOPIC_DETECTIONS = "rt/detections"
TOPIC_CMD_VEL = "rt/cmd_vel"
MOCAP_ANCHOR_BODY = "pelvis_anchor"
MOCAP_ANCHOR_Z = 0.793        # match the welded pelvis standing height
CMD_VEL_TIMEOUT_S = 0.5       # zero out mocap velocity if Linux drops commands

CAM_NAME = "head_cam"
CAM_W = 640
CAM_H = 480
CAM_HZ = 15.0
DEBUG_HTTP_PORT = 8088     # open http://localhost:8088 to see head_cam overlay

TOPIC_HEAD_CAM_IMAGE = "rt/head_cam/image_raw"

TOPIC_SCAN = "rt/scan"
LIDAR_SITE = "lidar_site"
LIDAR_FRAME_ID = "lidar_link"
LIDAR_NUM_RAYS = 360
LIDAR_RANGE_MIN = 0.10     # m, ROS LaserScan convention: discard dist < range_min
LIDAR_RANGE_MAX = 10.0     # m
LIDAR_HZ = 10.0            # 10 Hz scan rate matches the dock LIDAR_LINE servo loop

TOPIC_PARCEL_STATE = "rt/parcel_state"
PARCEL_BODY = "parcel"
PARCEL_FREEJOINT = "parcel_free"
PARCEL_WELD = "pin_parcel"
# Default rest poses for parcel teleport on "on_table_X" state.
# TODO_TUNE: align with Linux pick start pose (where palm closes in stage P3).
PARCEL_REST_TABLE_A = (1.20, 0.0, 0.975)  # on table A top (raised +0.105), -X edge
PARCEL_REST_TABLE_B = (3.70, 0.0, 0.975)  # on table B top (raised +0.105), -X edge


class LidarScanner:
    """Programmatic 2D LaserScan via mujoco.mj_ray() — no XML rangefinders.

    Pre-computes beam directions in lidar_site local frame (XY plane, sweep
    CCW from +X over [0, 2pi)). Each scan() transforms them to world via
    site_xmat and ray-casts. Returns ranges[N] of float32 in meters; values
    outside [range_min, range_max] are replaced with range_max+1 (the ROS
    LaserScan convention for "discard").
    """

    def __init__(self, mj_model, mj_data, num_rays=LIDAR_NUM_RAYS,
                 range_min=LIDAR_RANGE_MIN, range_max=LIDAR_RANGE_MAX):
        self.mj_model = mj_model
        self.mj_data = mj_data
        self.num_rays = num_rays
        self.range_min = range_min
        self.range_max = range_max
        self.site_id = int(mj_model.site(LIDAR_SITE).id)
        angles = np.linspace(0.0, 2.0 * np.pi, num_rays, endpoint=False)
        self.local_dirs = np.column_stack([
            np.cos(angles),
            np.sin(angles),
            np.zeros_like(angles),
        ])  # (num_rays, 3)
        self._geomid = np.zeros(1, dtype=np.int32)

    def scan(self):
        site_xpos = self.mj_data.site_xpos[self.site_id]
        site_xmat = self.mj_data.site_xmat[self.site_id].reshape(3, 3)
        world_dirs = self.local_dirs @ site_xmat.T
        ranges = np.empty(self.num_rays, dtype=np.float32)
        no_hit = np.float32(self.range_max + 1.0)
        for i in range(self.num_rays):
            dist = mujoco.mj_ray(
                self.mj_model, self.mj_data,
                site_xpos, world_dirs[i],
                None,            # geomgroup mask: include all
                1,               # flg_static: include static geoms
                -1,              # bodyexclude: none
                self._geomid,
            )
            if dist < 0.0 or dist > self.range_max or dist < self.range_min:
                ranges[i] = no_hit
            else:
                ranges[i] = np.float32(dist)
        return ranges

MOTOR_SENSOR_NUM = 3
NUM_MOTOR_IDL_GO = 20
NUM_MOTOR_IDL_HG = 35

# Sim-only escape hatch: when motor_cmd[i].mode == KINEMATIC_MODE, the bridge
# writes qpos directly instead of computing the per-motor PD torque. Used for
# arm joints in MuJoCo to avoid PD jitter from the DDS round-trip. Never enable
# on the real robot — its motor firmware does not define this mode.
KINEMATIC_MODE = 99


class TwoHandGrasp:
    """Midpoint kinematic tracking for a two-hand parcel grasp.

    Replaces the single-hand weld constraint that produced visual snapping
    and asymmetric carry. Pattern from g1_logistics_demo dex3_controller.

    On attach(), gates on hand-to-parcel distance and palm separation. If
    gating passes, captures the offset between parcel and the hand midpoint
    so no teleport happens (parcel stays where it is, midpoint follows).

    update_per_tick() runs every physics step; while closed, sets parcel
    qpos to midpoint + captured offset and zeroes its 6-DOF velocity.
    """

    MAX_HAND_SEP = 0.30      # m — palms farther apart than this means box not enclosed
    MAX_DIST_EACH = 0.20     # m — each hand must be within this from parcel center

    def __init__(self, mj_model, mj_data,
                 parcel_body=PARCEL_BODY,
                 left_ee_body="left_wrist_yaw_link",
                 right_ee_body="right_wrist_yaw_link",
                 parcel_freejoint=PARCEL_FREEJOINT):
        self.mj_model = mj_model
        self.mj_data = mj_data
        self._bid = -1
        self._left_id = -1
        self._right_id = -1
        self._qadr = -1
        self._dadr = -1
        try:
            self._bid = int(mj_model.body(parcel_body).id)
            self._left_id = int(mj_model.body(left_ee_body).id)
            self._right_id = int(mj_model.body(right_ee_body).id)
            jid = int(mj_model.joint(parcel_freejoint).id)
            self._qadr = int(mj_model.jnt_qposadr[jid])
            self._dadr = int(mj_model.jnt_dofadr[jid])
        except Exception as e:
            print(f"[grasp] init failed: {e}", flush=True)
        self._offset = None
        self._closed = False

    @property
    def usable(self):
        return self._bid >= 0 and self._left_id >= 0 and self._right_id >= 0

    def attach(self):
        if not self.usable:
            return False
        l = np.asarray(self.mj_data.xpos[self._left_id])
        r = np.asarray(self.mj_data.xpos[self._right_id])
        b = np.asarray(self.mj_data.xpos[self._bid])
        d_l = float(np.linalg.norm(l - b))
        d_r = float(np.linalg.norm(r - b))
        sep = float(np.linalg.norm(l - r))
        if d_l > self.MAX_DIST_EACH or d_r > self.MAX_DIST_EACH or sep > self.MAX_HAND_SEP:
            print(
                f"[grasp] attach FAILED: d_l={d_l:.3f} d_r={d_r:.3f} "
                f"sep={sep:.3f} (limits {self.MAX_DIST_EACH}/{self.MAX_DIST_EACH}/{self.MAX_HAND_SEP})",
                flush=True,
            )
            return False
        midpoint = 0.5 * (l + r)
        self._offset = (b - midpoint).copy()
        self._closed = True
        print(
            f"[grasp] attached: midpoint={midpoint} parcel={b} "
            f"offset={self._offset} d_l={d_l:.3f} d_r={d_r:.3f} sep={sep:.3f}",
            flush=True,
        )
        return True

    def open(self):
        if self._closed:
            print("[grasp] released", flush=True)
        self._closed = False
        self._offset = None

    def update_per_tick(self):
        if not self._closed or not self.usable:
            return
        l = self.mj_data.xpos[self._left_id]
        r = self.mj_data.xpos[self._right_id]
        midpoint = 0.5 * (l + r)
        new_pos = midpoint + self._offset
        self.mj_data.qpos[self._qadr:self._qadr + 3] = new_pos
        self.mj_data.qvel[self._dadr:self._dadr + 6] = 0.0


class UnitreeSdk2Bridge:

    def __init__(self, mj_model, mj_data):
        self.mj_model = mj_model
        self.mj_data = mj_data

        self.num_motor = self.mj_model.nu
        self.dim_motor_sensor = MOTOR_SENSOR_NUM * self.num_motor
        self.have_imu = False
        self.have_frame_sensor = False
        self.dt = self.mj_model.opt.timestep
        self.idl_type = (self.num_motor > NUM_MOTOR_IDL_GO) # 0: unitree_go, 1: unitree_hg

        # Precompute qpos / qvel addresses for each motor for kinematic-mode
        # writes (motor_cmd[i].mode == KINEMATIC_MODE). The MuJoCo
        # actuator-to-joint map comes from actuator_trnid + jnt_qposadr / jnt_dofadr.
        self._qpos_addr = []
        self._qvel_addr = []
        for i in range(self.num_motor):
            jid = int(self.mj_model.actuator_trnid[i, 0])
            self._qpos_addr.append(int(self.mj_model.jnt_qposadr[jid]))
            self._qvel_addr.append(int(self.mj_model.jnt_dofadr[jid]))

        # Sticky kinematic-mode cache. LowCmdHandler stores the last commanded
        # q per joint with mode==KINEMATIC_MODE. ApplyKinematicCache() runs in
        # the physics thread before each mj_step and re-writes qpos for those
        # joints, so the arms stay locked between LowCmd arrivals regardless
        # of the publisher rate (gravity gets no window to drift the pose).
        self._kinematic_cache = {}
        self._kinematic_lock = threading.Lock()

        self.joystick = None

        # Check sensor
        for i in range(self.dim_motor_sensor, self.mj_model.nsensor):
            name = mujoco.mj_id2name(
                self.mj_model, mujoco._enums.mjtObj.mjOBJ_SENSOR, i
            )
            if name == "imu_quat":
                self.have_imu_ = True
            if name == "frame_pos":
                self.have_frame_sensor_ = True

        # Unitree sdk2 message
        self.low_state = LowState_default()
        self.low_state_puber = ChannelPublisher(TOPIC_LOWSTATE, LowState_)
        self.low_state_puber.Init()
        self.lowStateThread = RecurrentThread(
            interval=self.dt, target=self.PublishLowState, name="sim_lowstate"
        )
        self.lowStateThread.Start()

        self.high_state = unitree_go_msg_dds__SportModeState_()
        self.high_state_puber = ChannelPublisher(TOPIC_HIGHSTATE, SportModeState_)
        self.high_state_puber.Init()
        self.HighStateThread = RecurrentThread(
            interval=self.dt, target=self.PublishHighState, name="sim_highstate"
        )
        self.HighStateThread.Start()

        self.wireless_controller = unitree_go_msg_dds__WirelessController_()
        self.wireless_controller_puber = ChannelPublisher(
            TOPIC_WIRELESS_CONTROLLER, WirelessController_
        )
        self.wireless_controller_puber.Init()
        self.WirelessControllerThread = RecurrentThread(
            interval=0.01,
            target=self.PublishWirelessController,
            name="sim_wireless_controller",
        )
        self.WirelessControllerThread.Start()

        self.low_cmd_suber = ChannelSubscriber(TOPIC_LOWCMD, LowCmd_)
        self.low_cmd_suber.Init(self.LowCmdHandler, 10)

        # Mocap drive from /cmd_vel (sim-only "ice skating" — robot pelvis
        # anchor body integrates Twist messages until a real walking controller
        # replaces it). Pelvis is welded to the anchor, so moving the anchor
        # drags the whole robot.
        self._mocap_x = 0.0
        self._mocap_y = 0.0
        self._mocap_yaw = 0.0
        self._mocap_z = MOCAP_ANCHOR_Z
        self._cmd_vel = (0.0, 0.0, 0.0)   # vx, vy, vyaw in robot body frame
        self._cmd_vel_lock = threading.Lock()
        self._cmd_vel_last_t = 0.0
        self._cmd_vel_count = 0           # diagnostic: msgs received in current 1 s window
        self._last_cmd_vel_print = 0.0
        try:
            self._mocap_body_id = int(self.mj_model.body(MOCAP_ANCHOR_BODY).mocapid[0])
        except Exception:
            self._mocap_body_id = -1
        if self._mocap_body_id >= 0:
            self._cmd_vel_suber = ChannelSubscriber(TOPIC_CMD_VEL, Twist_)
            self._cmd_vel_suber.Init(self._on_cmd_vel, 10)

        # 2D LaserScan via mj_ray. Skips silently if lidar_site is missing.
        self._lidar = None
        self._scan_puber = None
        try:
            self.mj_model.site(LIDAR_SITE)
            has_lidar = True
        except Exception:
            has_lidar = False
        if has_lidar:
            self._lidar = LidarScanner(self.mj_model, self.mj_data)
            self._scan_puber = ChannelPublisher(TOPIC_SCAN, LaserScan_)
            self._scan_puber.Init()
            self._scan_period_s = 1.0 / LIDAR_HZ
            self._last_scan_t = -1e9

        # Geometry diagnostic — periodic log of pelvis / parcel / cam world XYZ.
        self._last_geom_log_t = -1e9

        # Parcel state — dwa mechanizmy z fallback'iem:
        #  1) TwoHandGrasp (preferred): midpoint kinematic tracking między palms.
        #     Trigger: rt/parcel_state "carrying" → attach() jeśli hands blisko.
        #  2) Legacy weld pin_parcel (active="false" w XML, fallback only).
        # "on_table_X" zawsze release + teleport na fixed biurka pos.
        self._parcel_eq_id = -1
        self._parcel_qpos_addr = -1
        self._parcel_qvel_addr = -1
        try:
            self._parcel_eq_id = int(self.mj_model.equality(PARCEL_WELD).id)
            jid = int(self.mj_model.joint(PARCEL_FREEJOINT).id)
            self._parcel_qpos_addr = int(self.mj_model.jnt_qposadr[jid])
            self._parcel_qvel_addr = int(self.mj_model.jnt_dofadr[jid])
        except Exception:
            pass
        self._grasp = TwoHandGrasp(self.mj_model, self.mj_data)
        if self._parcel_eq_id >= 0 or self._grasp.usable:
            self._parcel_state_suber = ChannelSubscriber(TOPIC_PARCEL_STATE, String_)
            self._parcel_state_suber.Init(self._on_parcel_state, 5)

        # Camera + AprilTag detection (sim only). Skips silently if the named
        # camera is not present in the model.
        self._tag_renderer = None
        self._tag_detector = None
        self._tag_puber = None
        try:
            cam_obj = mujoco.mjtObj.mjOBJ_CAMERA
        except AttributeError:
            cam_obj = mujoco._enums.mjtObj.mjOBJ_CAMERA
        cam_id = mujoco.mj_name2id(self.mj_model, cam_obj, CAM_NAME)
        if cam_id >= 0:
            self._tag_renderer = mujoco.Renderer(
                self.mj_model, height=CAM_H, width=CAM_W
            )
            self._tag_detector = AprilTagDetector(
                families="tag36h11",
                nthreads=2,
                quad_decimate=1.0,
                quad_sigma=0.0,
                refine_edges=True,
                decode_sharpening=0.25,
            )
            self._tag_puber = ChannelPublisher(TOPIC_DETECTIONS, AprilTagDetectionArray_)
            self._tag_puber.Init()
            # Raw RGB image stream for rqt_image_view / Linux-side perception.
            self._image_puber = ChannelPublisher(TOPIC_HEAD_CAM_IMAGE, Image_)
            self._image_puber.Init()
            # No RecurrentThread — render must run in the same thread as the
            # MuJoCo physics step (on macOS the GL context is owned by that
            # thread and render() from another thread blocks). The simulation
            # loop calls MaybeStepCamera() after each mj_step.
            self._cam_period_s = 1.0 / CAM_HZ
            self._last_cam_t = -1e9
            # Debug-overlay buffer + tiny HTTP server. Open
            # http://localhost:DEBUG_HTTP_PORT in a browser for a live preview;
            # cv2.imshow is unreliable under mjpython on macOS (Cocoa main
            # thread is not exposed).
            self._debug_lock = threading.Lock()
            self._latest_debug_img = None
            _DebugHTTPHandler.bridge = self
            try:
                self._debug_http_server = _DebugHTTPServer(
                    ("127.0.0.1", DEBUG_HTTP_PORT), _DebugHTTPHandler
                )
                self._debug_http_thread = threading.Thread(
                    target=self._debug_http_server.serve_forever,
                    name="head_cam_http",
                    daemon=True,
                )
                self._debug_http_thread.start()
                print(
                    f"[bridge] head_cam preview at http://localhost:{DEBUG_HTTP_PORT}",
                    flush=True,
                )
            except OSError as e:
                print(f"[bridge] could not start debug HTTP server: {e}", flush=True)

        # joystick
        self.key_map = {
            "R1": 0,
            "L1": 1,
            "start": 2,
            "select": 3,
            "R2": 4,
            "L2": 5,
            "F1": 6,
            "F2": 7,
            "A": 8,
            "B": 9,
            "X": 10,
            "Y": 11,
            "up": 12,
            "right": 13,
            "down": 14,
            "left": 15,
        }

    def LowCmdHandler(self, msg: LowCmd_):
        if self.mj_data is None:
            return
        with self._kinematic_lock:
            for i in range(self.num_motor):
                cmd = msg.motor_cmd[i]
                if cmd.mode == KINEMATIC_MODE:
                    # Cache the target q; the physics thread re-applies it
                    # every step so the joint stays sticky between LowCmd
                    # arrivals.
                    self._kinematic_cache[i] = float(cmd.q)
                else:
                    # Joint left kinematic mode — drop from cache and resume
                    # the standard per-motor PD torque.
                    self._kinematic_cache.pop(i, None)
                    self.mj_data.ctrl[i] = (
                        cmd.tau
                        + cmd.kp * (cmd.q - self.mj_data.sensordata[i])
                        + cmd.kd * (cmd.dq - self.mj_data.sensordata[i + self.num_motor])
                    )

    def ApplyKinematicCache(self):
        """Re-apply cached kinematic-mode qpos for every joint in the cache.

        Called from the physics thread before each mj_step. Keeps qpos sticky
        regardless of LowCmd publisher rate — gravity has no window to drift
        the welded arms between commands.
        """
        if self.mj_data is None:
            return
        with self._kinematic_lock:
            cache = list(self._kinematic_cache.items())
        for i, q in cache:
            self.mj_data.qpos[self._qpos_addr[i]] = q
            self.mj_data.qvel[self._qvel_addr[i]] = 0.0
            self.mj_data.ctrl[i] = 0.0

    def PublishLowState(self):
        if self.mj_data != None:
            for i in range(self.num_motor):
                # Read directly from qpos / qvel / qfrc_actuator instead of
                # sensordata. Position sensors may not be wired up for every
                # joint (notably the arms) so sensordata reads back as 0 even
                # when the joint has a real angle. _qpos_addr / _qvel_addr are
                # the actuator-to-joint maps already built in __init__.
                qpos_i = self._qpos_addr[i]
                qvel_i = self._qvel_addr[i]
                self.low_state.motor_state[i].q = float(self.mj_data.qpos[qpos_i])
                self.low_state.motor_state[i].dq = float(self.mj_data.qvel[qvel_i])
                self.low_state.motor_state[i].tau_est = float(
                    self.mj_data.qfrc_actuator[qvel_i]
                )
                self.low_state.motor_state[i].mode = 0

            if self.have_frame_sensor_:

                self.low_state.imu_state.quaternion[0] = self.mj_data.sensordata[
                    self.dim_motor_sensor + 0
                ]
                self.low_state.imu_state.quaternion[1] = self.mj_data.sensordata[
                    self.dim_motor_sensor + 1
                ]
                self.low_state.imu_state.quaternion[2] = self.mj_data.sensordata[
                    self.dim_motor_sensor + 2
                ]
                self.low_state.imu_state.quaternion[3] = self.mj_data.sensordata[
                    self.dim_motor_sensor + 3
                ]

                self.low_state.imu_state.gyroscope[0] = self.mj_data.sensordata[
                    self.dim_motor_sensor + 4
                ]
                self.low_state.imu_state.gyroscope[1] = self.mj_data.sensordata[
                    self.dim_motor_sensor + 5
                ]
                self.low_state.imu_state.gyroscope[2] = self.mj_data.sensordata[
                    self.dim_motor_sensor + 6
                ]

                self.low_state.imu_state.accelerometer[0] = self.mj_data.sensordata[
                    self.dim_motor_sensor + 7
                ]
                self.low_state.imu_state.accelerometer[1] = self.mj_data.sensordata[
                    self.dim_motor_sensor + 8
                ]
                self.low_state.imu_state.accelerometer[2] = self.mj_data.sensordata[
                    self.dim_motor_sensor + 9
                ]

            if self.joystick != None:
                pygame.event.get()
                # Buttons
                self.low_state.wireless_remote[2] = int(
                    "".join(
                        [
                            f"{key}"
                            for key in [
                                0,
                                0,
                                int(self.joystick.get_axis(self.axis_id["LT"]) > 0),
                                int(self.joystick.get_axis(self.axis_id["RT"]) > 0),
                                int(self.joystick.get_button(self.button_id["SELECT"])),
                                int(self.joystick.get_button(self.button_id["START"])),
                                int(self.joystick.get_button(self.button_id["LB"])),
                                int(self.joystick.get_button(self.button_id["RB"])),
                            ]
                        ]
                    ),
                    2,
                )
                self.low_state.wireless_remote[3] = int(
                    "".join(
                        [
                            f"{key}"
                            for key in [
                                int(self.joystick.get_hat(0)[0] < 0),  # left
                                int(self.joystick.get_hat(0)[1] < 0),  # down
                                int(self.joystick.get_hat(0)[0] > 0), # right
                                int(self.joystick.get_hat(0)[1] > 0),    # up
                                int(self.joystick.get_button(self.button_id["Y"])),     # Y
                                int(self.joystick.get_button(self.button_id["X"])),     # X
                                int(self.joystick.get_button(self.button_id["B"])),     # B
                                int(self.joystick.get_button(self.button_id["A"])),     # A
                            ]
                        ]
                    ),
                    2,
                )
                # Axes
                sticks = [
                    self.joystick.get_axis(self.axis_id["LX"]),
                    self.joystick.get_axis(self.axis_id["RX"]),
                    -self.joystick.get_axis(self.axis_id["RY"]),
                    -self.joystick.get_axis(self.axis_id["LY"]),
                ]
                packs = list(map(lambda x: struct.pack("f", x), sticks))
                self.low_state.wireless_remote[4:8] = packs[0]
                self.low_state.wireless_remote[8:12] = packs[1]
                self.low_state.wireless_remote[12:16] = packs[2]
                self.low_state.wireless_remote[20:24] = packs[3]

            self.low_state_puber.Write(self.low_state)

    def PublishHighState(self):

        if self.mj_data != None:
            self.high_state.position[0] = self.mj_data.sensordata[
                self.dim_motor_sensor + 10
            ]
            self.high_state.position[1] = self.mj_data.sensordata[
                self.dim_motor_sensor + 11
            ]
            self.high_state.position[2] = self.mj_data.sensordata[
                self.dim_motor_sensor + 12
            ]

            self.high_state.velocity[0] = self.mj_data.sensordata[
                self.dim_motor_sensor + 13
            ]
            self.high_state.velocity[1] = self.mj_data.sensordata[
                self.dim_motor_sensor + 14
            ]
            self.high_state.velocity[2] = self.mj_data.sensordata[
                self.dim_motor_sensor + 15
            ]

        self.high_state_puber.Write(self.high_state)

    def PublishWirelessController(self):
        if self.joystick != None:
            pygame.event.get()
            key_state = [0] * 16
            key_state[self.key_map["R1"]] = self.joystick.get_button(
                self.button_id["RB"]
            )
            key_state[self.key_map["L1"]] = self.joystick.get_button(
                self.button_id["LB"]
            )
            key_state[self.key_map["start"]] = self.joystick.get_button(
                self.button_id["START"]
            )
            key_state[self.key_map["select"]] = self.joystick.get_button(
                self.button_id["SELECT"]
            )
            key_state[self.key_map["R2"]] = (
                self.joystick.get_axis(self.axis_id["RT"]) > 0
            )
            key_state[self.key_map["L2"]] = (
                self.joystick.get_axis(self.axis_id["LT"]) > 0
            )
            key_state[self.key_map["F1"]] = 0
            key_state[self.key_map["F2"]] = 0
            key_state[self.key_map["A"]] = self.joystick.get_button(self.button_id["A"])
            key_state[self.key_map["B"]] = self.joystick.get_button(self.button_id["B"])
            key_state[self.key_map["X"]] = self.joystick.get_button(self.button_id["X"])
            key_state[self.key_map["Y"]] = self.joystick.get_button(self.button_id["Y"])
            key_state[self.key_map["up"]] = self.joystick.get_hat(0)[1] > 0
            key_state[self.key_map["right"]] = self.joystick.get_hat(0)[0] > 0
            key_state[self.key_map["down"]] = self.joystick.get_hat(0)[1] < 0
            key_state[self.key_map["left"]] = self.joystick.get_hat(0)[0] < 0

            key_value = 0
            for i in range(16):
                key_value += key_state[i] << i

            self.wireless_controller.keys = key_value
            self.wireless_controller.lx = self.joystick.get_axis(self.axis_id["LX"])
            self.wireless_controller.ly = -self.joystick.get_axis(self.axis_id["LY"])
            self.wireless_controller.rx = self.joystick.get_axis(self.axis_id["RX"])
            self.wireless_controller.ry = -self.joystick.get_axis(self.axis_id["RY"])

            self.wireless_controller_puber.Write(self.wireless_controller)

    def MaybeStepCamera(self):
        """Run camera render + AprilTag detection + DDS publish if it is time.

        Must be called from the same thread as mj_step (the MuJoCo GL context
        is owned by that thread on macOS). Rate-limited by CAM_HZ.
        """
        if self._tag_renderer is None or self.mj_data is None:
            return
        sim_t = float(self.mj_data.time)
        if sim_t - self._last_cam_t < self._cam_period_s:
            return
        self._last_cam_t = sim_t

        self._tag_renderer.update_scene(self.mj_data, camera=CAM_NAME)
        rgb = self._tag_renderer.render()
        gray = cv2.cvtColor(rgb, cv2.COLOR_RGB2GRAY)
        dets = self._tag_detector.detect(gray, estimate_tag_pose=False)

        # Build a BGR debug image with detected tag corners. Cannot call
        # cv2.imshow from this thread on macOS (Cocoa main-thread requirement);
        # buffer it for the main loop in unitree_mujoco.py to display.
        debug = cv2.cvtColor(rgb, cv2.COLOR_RGB2BGR)
        for d in dets:
            pts = d.corners.astype(int).reshape(-1, 1, 2)
            cv2.polylines(debug, [pts], isClosed=True, color=(0, 255, 0), thickness=2)
            label_pos = tuple(d.corners[0].astype(int))
            cv2.putText(
                debug,
                f"id{d.tag_id} m={d.decision_margin:.0f}",
                label_pos,
                cv2.FONT_HERSHEY_SIMPLEX,
                0.5,
                (0, 255, 0),
                1,
            )
        with self._debug_lock:
            self._latest_debug_img = debug

        sec = int(sim_t)
        nsec = int((sim_t - sec) * 1e9)
        items = []
        for d in dets:
            corners = [
                AprilTagPoint_(x=float(c[0]), y=float(c[1])) for c in d.corners
            ]
            homo = [float(v) for v in d.homography.flatten()]
            items.append(
                AprilTagDetection_(
                    family="tag36h11",
                    id=int(d.tag_id),
                    hamming=int(d.hamming),
                    goodness=0.0,
                    decision_margin=float(d.decision_margin),
                    centre=AprilTagPoint_(x=float(d.center[0]), y=float(d.center[1])),
                    corners=corners,
                    homography=homo,
                )
            )
        msg = AprilTagDetectionArray_(
            header=Header_(
                stamp=Time_(sec=sec, nanosec=nsec),
                frame_id=CAM_NAME,
            ),
            detections=items,
        )
        self._tag_puber.Write(msg)

        # Publish raw RGB image so Linux side (rqt_image_view, downstream
        # perception) can see the live POV. Same sim_t stamp as the detection
        # message for tick-level consistency. Encoding "rgb8", row-major.
        rgb_contig = np.ascontiguousarray(rgb, dtype=np.uint8)
        img_msg = Image_(
            header=Header_(
                stamp=Time_(sec=sec, nanosec=nsec),
                frame_id=CAM_NAME,
            ),
            height=int(CAM_H),
            width=int(CAM_W),
            encoding="rgb8",
            is_bigendian=0,
            step=int(CAM_W * 3),
            data=rgb_contig.tobytes(),
        )
        self._image_puber.Write(img_msg)

    def MaybeStepLidar(self):
        """Run a 2D LaserScan and publish on rt/scan if it is time.

        Must be called from the same thread as mj_step (mj_ray reads mj_data).
        Rate-limited by LIDAR_HZ on sim time.
        """
        if self._lidar is None or self.mj_data is None:
            return
        sim_t = float(self.mj_data.time)
        if sim_t - self._last_scan_t < self._scan_period_s:
            return
        self._last_scan_t = sim_t

        ranges = self._lidar.scan()
        # Stamp in Linux system time, NOT sim_t. slam_toolbox does
        # lookupTransform(lidar_link, base_link, scan.header.stamp); the Linux
        # TF buffer holds transforms in epoch seconds, so a sim_t stamp (~few
        # hundred seconds since sim start) misses every transform and the
        # scan is dropped. The other topics (lowstate / detections / lowcmd)
        # do not do TF lookup on these messages and stay on sim_t.
        wall_now = time.time()
        sec = int(wall_now)
        nsec = int((wall_now - sec) * 1e9)
        msg = LaserScan_(
            header=Header_(
                stamp=Time_(sec=sec, nanosec=nsec),
                frame_id=LIDAR_FRAME_ID,
            ),
            angle_min=0.0,
            angle_max=float(2.0 * math.pi),
            angle_increment=float(2.0 * math.pi / LIDAR_NUM_RAYS),
            time_increment=0.0,
            scan_time=float(self._scan_period_s),
            range_min=float(LIDAR_RANGE_MIN),
            range_max=float(LIDAR_RANGE_MAX),
            ranges=ranges.tolist(),
            intensities=[],
        )
        self._scan_puber.Write(msg)

    def UpdateGrasp(self):
        """Apply TwoHandGrasp midpoint tracking — call once per physics step.

        While `_grasp` is closed, snaps parcel qpos to the hand midpoint plus
        the captured offset and zeroes the parcel's 6-DOF velocity. Cheap if
        not closed (early return).
        """
        self._grasp.update_per_tick()

    def MaybeLogGeometry(self):
        """Print pelvis / parcel / head_cam world XYZ once per simulated second.

        Diagnostic for cross-checking Linux-side geometric model (brief 0019).
        """
        if self.mj_data is None:
            return
        sim_t = float(self.mj_data.time)
        if sim_t - self._last_geom_log_t < 1.0:
            return
        self._last_geom_log_t = sim_t
        try:
            pelvis = self.mj_data.body("pelvis").xpos
            parcel = self.mj_data.body("parcel").xpos
            cam_id = int(self.mj_model.camera("head_cam").id)
            cam = self.mj_data.cam_xpos[cam_id]
            print(
                f"[GEOM] t={sim_t:.1f} "
                f"pelvis=({pelvis[0]:.3f},{pelvis[1]:.3f},{pelvis[2]:.3f}) "
                f"parcel=({parcel[0]:.3f},{parcel[1]:.3f},{parcel[2]:.3f}) "
                f"cam=({cam[0]:.3f},{cam[1]:.3f},{cam[2]:.3f})",
                flush=True,
            )
        except Exception as e:
            print(f"[GEOM] body/cam lookup failed: {e}", flush=True)

    def take_debug_frame(self):
        """Return the latest head_cam debug image (BGR) or None if unchanged.

        Called from the main thread (mjpython routes that to the Cocoa main
        thread on macOS), which then calls cv2.imshow. Reading clears the
        buffer so the main loop only redraws when there is new data.
        """
        if self._tag_renderer is None:
            return None
        with self._debug_lock:
            img = self._latest_debug_img
            self._latest_debug_img = None
        return img

    def _on_cmd_vel(self, msg: Twist_):
        with self._cmd_vel_lock:
            self._cmd_vel = (
                float(msg.linear.x),
                float(msg.linear.y),
                float(msg.angular.z),
            )
            self._cmd_vel_last_t = time.monotonic()
            self._cmd_vel_count += 1

    def _on_parcel_state(self, msg: String_):
        """Handle parcel state transitions from Linux mission BT.

        States:
          "carrying"   -> attempt TwoHandGrasp.attach() (gates on hand-to-parcel
                          distance + palm sep); legacy weld also flipped on as
                          back-compat fallback. Plus dumps wrist XYZ for P4
                          calibration when Linux publishes at stage 'lift'.
          "on_table_a" -> release grasp + teleport parcel onto table A.
          "on_table_b" -> release grasp + teleport parcel onto table B.
          "none" or other -> release grasp, leave pose alone.
        """
        if self.mj_data is None:
            return
        state = (msg.data or "").strip()
        if state == "carrying":
            # P4 calibration log (kept from brief 0010 — diagnostic).
            try:
                L = self.mj_data.body("left_wrist_yaw_link").xpos.copy()
                R = self.mj_data.body("right_wrist_yaw_link").xpos.copy()
                mid = ((L[0] + R[0]) / 2, (L[1] + R[1]) / 2, (L[2] + R[2]) / 2)
                sep_y = abs(L[1] - R[1])
                print(
                    f"[P4 CALIBRATION] L_wrist={L} R_wrist={R} "
                    f"midpoint=({mid[0]:.3f},{mid[1]:.3f},{mid[2]:.3f}) sep_Y={sep_y:.3f}",
                    flush=True,
                )
            except Exception as e:
                print(f"[P4 CALIBRATION] body lookup failed: {e}", flush=True)
            # Primary v3 grasp: midpoint kinematic tracking with attach-gate.
            self._grasp.attach()
            # Legacy weld toggle — left as inert fallback (active_default=0
            # in XML; if user re-enables at runtime, weld also engages).
            if self._parcel_eq_id >= 0:
                self.mj_data.eq_active[self._parcel_eq_id] = 1
            return
        # All non-"carrying" states release any held grasp first.
        self._grasp.open()
        if self._parcel_eq_id >= 0:
            self.mj_data.eq_active[self._parcel_eq_id] = 0

        if state == "reset_all":
            # Full sim state reset for autonomous iteration loop. Brings the
            # robot back to origin, parcel onto table A, zeroes all velocities
            # and clears bridge integrator state (cmd_vel drift, sticky
            # kinematic cache from previous iteration's arm commands).
            self.mj_data.qpos[0:3] = (0.0, 0.0, MOCAP_ANCHOR_Z)
            self.mj_data.qpos[3:7] = (1.0, 0.0, 0.0, 0.0)
            if self._mocap_body_id >= 0:
                self.mj_data.mocap_pos[self._mocap_body_id] = (0.0, 0.0, MOCAP_ANCHOR_Z)
                self.mj_data.mocap_quat[self._mocap_body_id] = (1.0, 0.0, 0.0, 0.0)
            self._mocap_x = 0.0
            self._mocap_y = 0.0
            self._mocap_yaw = 0.0
            if self._parcel_qpos_addr >= 0:
                a = self._parcel_qpos_addr
                self.mj_data.qpos[a:a + 3] = PARCEL_REST_TABLE_A
                self.mj_data.qpos[a + 3:a + 7] = (1.0, 0.0, 0.0, 0.0)
            self.mj_data.qvel[:] = 0.0
            with self._kinematic_lock:
                self._kinematic_cache.clear()
            print("[reset_all] sim state reset to origin", flush=True)
            return

        if state == "on_table_a":
            rest = PARCEL_REST_TABLE_A
        elif state == "on_table_b":
            rest = PARCEL_REST_TABLE_B
        else:
            return  # "none" or unknown — no teleport
        if self._parcel_qpos_addr < 0:
            return
        a = self._parcel_qpos_addr
        v = self._parcel_qvel_addr
        self.mj_data.qpos[a:a + 3] = rest
        self.mj_data.qpos[a + 3:a + 7] = (1.0, 0.0, 0.0, 0.0)  # quat (w,x,y,z) identity
        self.mj_data.qvel[v:v + 6] = 0.0

    def MaybeStepMocap(self):
        """Integrate the latest /cmd_vel into the mocap anchor pose.

        Must be called from the same thread as mj_step (modifies mj_data).
        Body-frame linear velocity is rotated by the current yaw before
        integrating into world-frame mocap_pos. Failsafe: zero out velocity
        if no command arrived in the last CMD_VEL_TIMEOUT_S seconds.
        """
        if self._mocap_body_id < 0 or self.mj_data is None:
            return
        now = time.monotonic()
        with self._cmd_vel_lock:
            vx, vy, vyaw = self._cmd_vel
            age = now - self._cmd_vel_last_t
            if now - self._last_cmd_vel_print > 1.0:
                msgs = self._cmd_vel_count
                self._cmd_vel_count = 0
                self._last_cmd_vel_print = now
            else:
                msgs = None
        if msgs is not None:
            print(f"[bridge] cmd_vel msgs/s = {msgs}", flush=True)
        if age > CMD_VEL_TIMEOUT_S:
            vx = vy = vyaw = 0.0

        dt = self.mj_model.opt.timestep
        cy = math.cos(self._mocap_yaw)
        sy = math.sin(self._mocap_yaw)
        self._mocap_x += (cy * vx - sy * vy) * dt
        self._mocap_y += (sy * vx + cy * vy) * dt
        self._mocap_yaw += vyaw * dt

        half = self._mocap_yaw * 0.5
        self.mj_data.mocap_pos[self._mocap_body_id] = [
            self._mocap_x,
            self._mocap_y,
            self._mocap_z,
        ]
        self.mj_data.mocap_quat[self._mocap_body_id] = [
            math.cos(half),
            0.0,
            0.0,
            math.sin(half),
        ]

    def SetupJoystick(self, device_id=0, js_type="xbox"):
        pygame.init()
        pygame.joystick.init()
        joystick_count = pygame.joystick.get_count()
        if joystick_count > 0:
            self.joystick = pygame.joystick.Joystick(device_id)
            self.joystick.init()
        else:
            print("No gamepad detected.")
            sys.exit()

        if js_type == "xbox":
            self.axis_id = {
                "LX": 0,  # Left stick axis x
                "LY": 1,  # Left stick axis y
                "RX": 3,  # Right stick axis x
                "RY": 4,  # Right stick axis y
                "LT": 2,  # Left trigger
                "RT": 5,  # Right trigger
                "DX": 6,  # Directional pad x
                "DY": 7,  # Directional pad y
            }

            self.button_id = {
                "X": 2,
                "Y": 3,
                "B": 1,
                "A": 0,
                "LB": 4,
                "RB": 5,
                "SELECT": 6,
                "START": 7,
            }

        elif js_type == "switch":
            self.axis_id = {
                "LX": 0,  # Left stick axis x
                "LY": 1,  # Left stick axis y
                "RX": 2,  # Right stick axis x
                "RY": 3,  # Right stick axis y
                "LT": 5,  # Left trigger
                "RT": 4,  # Right trigger
                "DX": 6,  # Directional pad x
                "DY": 7,  # Directional pad y
            }

            self.button_id = {
                "X": 3,
                "Y": 4,
                "B": 1,
                "A": 0,
                "LB": 6,
                "RB": 7,
                "SELECT": 10,
                "START": 11,
            }
        else:
            print("Unsupported gamepad. ")

    def PrintSceneInformation(self):
        print(" ")

        print("<<------------- Link ------------->> ")
        for i in range(self.mj_model.nbody):
            name = mujoco.mj_id2name(self.mj_model, mujoco._enums.mjtObj.mjOBJ_BODY, i)
            if name:
                print("link_index:", i, ", name:", name)
        print(" ")

        print("<<------------- Joint ------------->> ")
        for i in range(self.mj_model.njnt):
            name = mujoco.mj_id2name(self.mj_model, mujoco._enums.mjtObj.mjOBJ_JOINT, i)
            if name:
                print("joint_index:", i, ", name:", name)
        print(" ")

        print("<<------------- Actuator ------------->>")
        for i in range(self.mj_model.nu):
            name = mujoco.mj_id2name(
                self.mj_model, mujoco._enums.mjtObj.mjOBJ_ACTUATOR, i
            )
            if name:
                print("actuator_index:", i, ", name:", name)
        print(" ")

        print("<<------------- Sensor ------------->>")
        index = 0
        for i in range(self.mj_model.nsensor):
            name = mujoco.mj_id2name(
                self.mj_model, mujoco._enums.mjtObj.mjOBJ_SENSOR, i
            )
            if name:
                print(
                    "sensor_index:",
                    index,
                    ", name:",
                    name,
                    ", dim:",
                    self.mj_model.sensor_dim[i],
                )
            index = index + self.mj_model.sensor_dim[i]
        print(" ")


class ElasticBand:

    def __init__(self):
        self.stiffness = 200
        self.damping = 100
        self.point = np.array([0, 0, 3])
        self.length = 0
        self.enable = True

    def Advance(self, x, dx):
        """
        Args:
          δx: desired position - current position
          dx: current velocity
        """
        δx = self.point - x
        distance = np.linalg.norm(δx)
        direction = δx / distance
        v = np.dot(dx, direction)
        f = (self.stiffness * (distance - self.length) - self.damping * v) * direction
        return f

    def MujuocoKeyCallback(self, key):
        glfw = mujoco.glfw.glfw
        if key == glfw.KEY_7:
            self.length -= 0.1
        if key == glfw.KEY_8:
            self.length += 0.1
        if key == glfw.KEY_9:
            self.enable = not self.enable
