import json
from enum import Enum

import numpy as np
from numpy import typing as npt

from .convention import muj2pin


class Convention(Enum):
    PIN = 1  # pinocchio convention - scalar-last quaternion
    MUJ = 2  # mujoco convention - scalar-first quaternion


class SimLog:
    def __init__(self, filename: str, history: dict | None = None) -> None:
        self.__filename = filename

        if history is None:
            with open(self.__filename) as file:
                self.__history = json.load(file)

            for key in self.__history:
                if key.startswith("sensor_") or key.startswith("data_"):
                    self.__history[key] = np.array(self.__history[key])
        else:
            self.__history = history

        # cache pinocchio convention data
        self.__pinpos = None
        self.__pinvel = None

    def __len__(self) -> int:
        return len(self.__history["data_time"])

    def __getitem__(self, key):
        if isinstance(key, slice):
            indices = range(*key.indices(self.__len__()))

            copy_history = self.__history.copy()
            for k in copy_history:
                if k.startswith("sensor_") or k.startswith("data_"):
                    copy_history[k] = copy_history[k][indices]

            return SimLog(self.__filename, copy_history)

        copy_history = self.__history.copy()
        for k in copy_history:
            if k.startswith("sensor_") or k.startswith("data_"):
                copy_history[k] = copy_history[k][key]

        return SimLog(self.__filename, copy_history)

    def __setitem__(self, key, value):
        raise NotImplementedError

    def __delitem__(self, key):
        raise NotImplementedError

    @property
    def timestamp(self) -> str:
        return self.__history["timestamp"]

    @property
    def nq(self) -> int:
        return self.__history["nq"]

    @property
    def nv(self) -> int:
        return self.__history["nv"]

    @property
    def nu(self) -> int:
        return self.__history["nu"]

    def sensor(self, name: str) -> npt.ArrayLike:
        return self.__history[f"sensor_{name}"]

    def data(self, key: str, convention: Convention = Convention.MUJ) -> npt.ArrayLike:
        if convention == Convention.MUJ:
            return self.__history[f"data_{key}"]

        # for pinocchio convention we have to apply transformation on qvel and qpos query
        if key not in ["qpos", "qvel"]:
            return self.__history[f"data_{key}"]

        if self.__pinpos is not None and self.__pinvel is not None:
            if key == "qpos":
                return self.__pinpos
            if key == "qvel":
                return self.__pinvel

        # we have to convert one by one
        self.__pinpos = np.zeros_like(self.__history["data_qpos"])
        self.__pinvel = np.zeros_like(self.__history["data_qvel"])

        for i in range(self.__pinpos.shape[0]):
            qi = self.__history["data_qpos"][i]
            vi = self.__history["data_qvel"][i]

            pinpos, pinvel = muj2pin(qi, vi)

            self.__pinpos[i] = pinpos
            self.__pinvel[i] = pinvel

        if key == "qpos":
            return self.__pinpos

        if key == "qvel":
            return self.__pinvel

    @property
    def time(self) -> npt.ArrayLike:
        return self.data("time")

    @property
    def qpin(self) -> npt.ArrayLike:
        return self.data("qpos", Convention.PIN)

    @property
    def vpin(self) -> npt.ArrayLike:
        return self.data("qvel", Convention.PIN)

    @property
    def dv(self) -> npt.ArrayLike:
        return self.data("qacc")

    @property
    def u(self) -> npt.ArrayLike:
        return self.data("ctrl")
