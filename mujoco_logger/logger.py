"""Module for simulation run logger"""
import json
from datetime import datetime
from typing import List

import mujoco


class SimLogger:
    def __init__(
        self,
        mjmodel: mujoco.MjModel,
        mjdata: mujoco.MjData,
        data_keys: List[str] | None = None,
    ) -> None:
        self.__model = mjmodel
        self.__data = mjdata
        self.__history = {}

        # add metadata to history
        self.__history["timestamp"] = datetime.now().isoformat()
        self.__history["nq"] = mjmodel.nq
        self.__history["nv"] = mjmodel.nv
        self.__history["nu"] = mjmodel.nu

        default_keys = [
            "time",
            "qpos",
            "qvel",
            "qacc",
            "ctrl",
        ]

        # record data keys
        self.__data_keys = list(set(default_keys) | set(data_keys or []))

        for key in self.__data_keys:
            self.__history[f"data_{key}"] = []

        self.__sensor_names = []
        for i in range(mjmodel.nsensor):
            name = mjmodel.sensor(i).name
            self.__sensor_names.append(name)
            self.__history[f"sensor_{name}"] = []

    def record(self) -> None:
        # record data keys
        for key in self.__data_keys:
            if key == "time":
                self.__history[f"data_{key}"].append(self.__data.time)
                continue
            self.__history[f"data_{key}"].append(list(getattr(self.__data, key).copy()))

        # record sensor data
        for name in self.__sensor_names:
            self.__history[f"sensor_{name}"].append(list(self.__data.sensor(name).data.copy()))

    def save(self, filepath: str) -> None:
        with open(filepath, "w") as f:
            json.dump(self.__history, f)
