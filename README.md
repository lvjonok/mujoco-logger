# mujoco-logger

[![PyPI version](https://badge.fury.io/py/mujoco-logger.svg)](https://pypi.org/project/mujoco-logger)
[![pre-commit.ci status](https://results.pre-commit.ci/badge/github/lvjonok/mujoco-logger/main.svg)](https://results.pre-commit.ci/badge/github/lvjonok/mujoco-logger/main.svg)

<!-- [![Documentation Status](https://readthedocs.org/projects/cmeel/badge/?version=latest)](https://cmeel.readthedocs.io/en/latest/?badge=latest) -->

[![Code style: black](https://img.shields.io/badge/code%20style-black-000000.svg)](https://github.com/psf/black)
[![Poetry](https://img.shields.io/endpoint?url=https://python-poetry.org/badge/v0.json)](https://python-poetry.org/)
[![Ruff](https://img.shields.io/endpoint?url=https://raw.githubusercontent.com/charliermarsh/ruff/main/assets/badge/v1.json)](https://github.com/charliermarsh/ruff)

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
