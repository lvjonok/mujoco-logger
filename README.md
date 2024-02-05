# mujoco-logger

## Overview

I have found a need in a handy logger for `MuJoCo` simulator. As most of my research is based on the modeling of robot in `Pinocchio`-based libraries and simulating the behaviour and control in `MuJoCo` it was crucial to have a proper conversion from one convention of data to another (e.g. `qpos` in `MuJoCo` to `pinocchio`).

This library provides a simple way to log data from `MuJoCo` simulation and load it for further analysis. It also provides a simple way to mock the simulation and visualize the trajectory of the robot.

## Installation

```bash
pip install mujoco-logger
```

## API

The library itself is quite simple, you can simple look through docstrings of the classes to understand how to use it.

- [SimLog](mujoco_logger/sim_log.py)
- [SimMock](mujoco_logger/emulator.py)
- [SimLogger](mujoco_logger/logger.py)

## Examples

All examples are located in the `examples` directory.
