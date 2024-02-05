import os
import time

import mujoco
import mujoco.viewer
from robot_descriptions.a1_mj_description import MJCF_PATH

from mujoco_logger import SimLog, SimMock

assert os.path.exists("examples/a1_run.json"), "Run use_logger.py first to generate a1_run.json"

log = SimLog("examples/a1_run.json")
simulator = SimMock(log)

model = mujoco.MjModel.from_xml_path(MJCF_PATH)
data = mujoco.MjData(model)

mujoco.mj_resetData(model, data)

with mujoco.viewer.launch_passive(model, data) as viewer:
    # Close the viewer automatically after 30 wall-seconds.
    start = time.time()
    N = len(simulator)
    while viewer.is_running() and simulator.iteration < N:
        step_start = time.time()

        # update data from logged data
        data.qpos[:] = simulator.qmuj
        data.qvel[:] = simulator.vmuj
        data.qacc[:] = simulator.dv
        data.ctrl[:] = simulator.u
        mujoco.mj_step(model, data)

        simulator.step()

        # Example modification of a viewer option: toggle contact points every two seconds.
        with viewer.lock():
            viewer.opt.flags[mujoco.mjtVisFlag.mjVIS_CONTACTPOINT] = int(data.time % 2)

        # Pick up changes to the physics state, apply perturbations, update options from GUI.
        viewer.sync()

        # Rudimentary time keeping, will drift relative to wall clock.
        time_until_next_step = model.opt.timestep - (time.time() - step_start)
        if time_until_next_step > 0:
            time.sleep(time_until_next_step)

    viewer.close()
