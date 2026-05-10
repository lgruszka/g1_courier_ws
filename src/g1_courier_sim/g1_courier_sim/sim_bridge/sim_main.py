"""Entry point for the courier sim bridge.

Loads scene XML from `config.ROBOT_SCENE` (defaults to bundled
`assets/scene_courier.xml`), starts MuJoCo viewer + simulation thread,
runs `UnitreeSdk2Bridge` from `bridge.py` which publishes to DDS topics
(rt/lowstate, rt/scan, rt/detections, rt/head_cam/image_raw, ...) and
subscribes (rt/lowcmd, rt/cmd_vel, rt/parcel_state).

Run via:
    ros2 run g1_courier_sim sim_bridge_node
"""
import threading
import time

import mujoco
import mujoco.viewer

from unitree_sdk2py.core.channel import ChannelFactoryInitialize

from . import config
from .bridge import UnitreeSdk2Bridge, ElasticBand


def main() -> None:
    locker = threading.Lock()

    mj_model = mujoco.MjModel.from_xml_path(config.ROBOT_SCENE)
    mj_data = mujoco.MjData(mj_model)

    band_attached_link = -1
    elastic_band = None
    if config.ENABLE_ELASTIC_BAND:
        elastic_band = ElasticBand()
        if config.ROBOT in ("h1", "g1"):
            band_attached_link = mj_model.body("torso_link").id
        else:
            band_attached_link = mj_model.body("base_link").id
        viewer = mujoco.viewer.launch_passive(
            mj_model, mj_data, key_callback=elastic_band.MujuocoKeyCallback,
        )
    else:
        viewer = mujoco.viewer.launch_passive(mj_model, mj_data)

    mj_model.opt.timestep = config.SIMULATE_DT

    time.sleep(0.2)   # let viewer attach

    def simulation_thread():
        ChannelFactoryInitialize(config.DOMAIN_ID, config.INTERFACE)
        unitree = UnitreeSdk2Bridge(mj_model, mj_data)

        if config.USE_JOYSTICK:
            unitree.SetupJoystick(device_id=0, js_type=config.JOYSTICK_TYPE)
        if config.PRINT_SCENE_INFORMATION:
            unitree.PrintSceneInformation()

        while viewer.is_running():
            step_start = time.perf_counter()
            with locker:
                if config.ENABLE_ELASTIC_BAND and elastic_band is not None:
                    if elastic_band.enable:
                        mj_data.xfrc_applied[band_attached_link, :3] = (
                            elastic_band.Advance(mj_data.qpos[:3], mj_data.qvel[:3])
                        )
                unitree.MaybeStepMocap()
                unitree.ApplyKinematicCache()
                mujoco.mj_step(mj_model, mj_data)
                unitree.UpdateGrasp()
                unitree.MaybeStepCamera()
                unitree.MaybeStepLidar()
                unitree.MaybeLogGeometry()
            t_left = mj_model.opt.timestep - (time.perf_counter() - step_start)
            if t_left > 0:
                time.sleep(t_left)

    def viewer_thread_fn():
        while viewer.is_running():
            with locker:
                viewer.sync()
            time.sleep(config.VIEWER_DT)

    sim_t = threading.Thread(target=simulation_thread, daemon=False)
    view_t = threading.Thread(target=viewer_thread_fn, daemon=False)
    sim_t.start()
    view_t.start()
    sim_t.join()
    view_t.join()


if __name__ == "__main__":
    main()
