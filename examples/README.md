# Examples

This directory provides examples of how to use the `mujoco-logger` library.

## 1. How to use logger? - [use_logger.py](use_logger.py)

In this example file `examples/a1_run.json` is created with the content from simulation.

You can simply run logger alongside `passive_viewer` as follows:

```python
with mujoco.viewer.launch_passive(model, data) as viewer, SimLogger(model, data, output_filepath="your_file.json") as logger:
    ...

    mujoco.mj_step(model, data)
    logger.record()
```

File will be saved on simulation end.

## 2. How to use saved data for further analysis? - [analyze_data.py](analyze_data.py)

In this example file `examples/a1_run.json` is loaded and analyzed. Simple metadata is printed and `qpos` of floating body is plotted.

One may easily take a subset of logged data:

```python
from mujoco_logger import SimLog

log = SimLog("examples/a1_run.json")

# get subset of data (only second half)
N = len(log) // 2
log_subset = log[N:]
```

## 3. Mock simulation - [mock_sim.py](mock_sim.py)

In this example it is shown how to load logger data and visualize trajectory of robot. It might be handy to simulate `real-time` data processing for various algorithms.

```python
from mujoco_logger import SimLog, SimMock

assert os.path.exists("examples/a1_run.json"), "Run use_logger.py first to generate a1_run.json"

log = SimLog("examples/a1_run.json")
simulator = SimMock(log)

# take current simulation iteration
simulator.iteration

# update data from logged data at current iteration
data.qpos[:] = simulator.qmuj
data.qvel[:] = simulator.vmuj
data.qacc[:] = simulator.dv
data.ctrl[:] = simulator.u

mujoco.mj_step(model, data)
# step logger to next iteration
simulator.step()
```
