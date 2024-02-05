# mujoco-logger

## Overview

I have found a need in a handy logger for `MuJoCo` simulator. As most of my research is based on the modeling of robot in `Pinocchio`-based libraries and simulating the behaviour and control in `MuJoCo` it was crucial to have a proper conversion from one convention of data to another (e.g. `qpos` in `MuJoCo` to `pinocchio`).

This library provides a simple way to log data from `MuJoCo` simulation and load it for further analysis. It also provides a simple way to mock the simulation and visualize the trajectory of the robot.

## Installation

```bash
pip install mujoco-logger
```

## API

### SimLogger class

| Method   | Type   | Description                                  |
| -------- | ------ | -------------------------------------------- |
| `record` | `None` | Method to record data and sensor information |
| `save`   | `None` | Method to save the recorded data to a file   |

### SimLog class

| Name        | Type       | Description                                                         |
| ----------- | ---------- | ------------------------------------------------------------------- |
| `timestamp` | `property` | Property to access the creation timestamp from the simulation data  |
| `nq`        | `property` | Property to access the number of generalized coordinates            |
| `nv`        | `property` | Property to access the number of generalized velocities             |
| `nu`        | `property` | Property to access the number of control inputs                     |
| `time`      | `property` | Property to access the simulation time data                         |
| `qpin`      | `property` | Property to access the pinocchio convention generalized coordinates |
| `vpin`      | `property` | Property to access the pinocchio convention generalized velocities  |
| `dv`        | `property` | Property to access the generalized accelerations                    |
| `u`         | `property` | Property to access the control inputs                               |

| Method                                                                     | Description                                                                                               |
| -------------------------------------------------------------------------- | --------------------------------------------------------------------------------------------------------- |
| `data(key: str, convention: Convention = Convention.MUJ) -> npt.ArrayLike` | Get simulation data for a specific key and convention. Example usage: `time_data = sim_mock.data("time")` |
| `sensor(key: str) -> npt.ArrayLike`                                        | Get sensor data for a specific key. Example usage: `sensor_data = sim_mock.sensor("sensor_key")`          |

### SimMock class

| Property    | Type            | Description                                            |
| ----------- | --------------- | ------------------------------------------------------ |
| `iteration` | `int`           | Get the current iteration of the simulation.           |
| `time`      | `npt.ArrayLike` | Get the simulation time data.                          |
| `qmuj`      | `npt.ArrayLike` | Get the joint positions data in Mujoco convention.     |
| `qpin`      | `npt.ArrayLike` | Get the joint positions data in Pinocchio convention.  |
| `vmuj`      | `npt.ArrayLike` | Get the joint velocities data in Mujoco convention.    |
| `vpin`      | `npt.ArrayLike` | Get the joint velocities data in Pinocchio convention. |
| `dv`        | `npt.ArrayLike` | Get the joint accelerations data.                      |
| `u`         | `npt.ArrayLike` | Get the control input data.                            |

| Method                                                                     | Description                                                                                               |
| -------------------------------------------------------------------------- | --------------------------------------------------------------------------------------------------------- |
| `step()`                                                                   | Advance the simulation by one iteration.                                                                  |
| `data(key: str, convention: Convention = Convention.MUJ) -> npt.ArrayLike` | Get simulation data for a specific key and convention. Example usage: `time_data = sim_mock.data("time")` |
| `sensor(key: str) -> npt.ArrayLike`                                        | Get sensor data for a specific key. Example usage: `sensor_data = sim_mock.sensor("sensor_key")`          |

## Examples

All examples are located in the `examples` directory.
