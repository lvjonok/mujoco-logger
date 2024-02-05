import time

import mujoco
import mujoco.viewer
import numpy as np
from robot_descriptions.a1_mj_description import MJCF_PATH

from mujoco_logger import SimLogger

model = mujoco.MjModel.from_xml_path(MJCF_PATH)
data = mujoco.MjData(model)

mujoco.mj_resetData(model, data)

with mujoco.viewer.launch_passive(model, data) as viewer, SimLogger(
    model, data, output_filepath="examples/a1_run.json"
) as logger:
    # Close the viewer automatically after 30 wall-seconds.
    start = time.time()
    while viewer.is_running() and time.time() - start < 5.0:
        step_start = time.time()

        data.ctrl = 20 * np.random.randn(model.nu)  # Random actuator inputs.
        mujoco.mj_step(model, data)
        logger.record()

        # Example modification of a viewer option: toggle contact points every two seconds.
        with viewer.lock():
            viewer.opt.flags[mujoco.mjtVisFlag.mjVIS_CONTACTPOINT] = int(data.time % 2)

        # Pick up changes to the physics state, apply perturbations, update options from GUI.
        viewer.sync()

        # Rudimentary time keeping, will drift relative to wall clock.
        time_until_next_step = model.opt.timestep - (time.time() - step_start)
        if time_until_next_step > 0:
            time.sleep(time_until_next_step)
