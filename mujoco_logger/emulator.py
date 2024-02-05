import numpy.typing as npt

from .sim_log import SimLog


class SimMock:
    def __init__(self, sim_log: SimLog) -> None:
        self.__log = sim_log
        self.__iteration = 0

    def __len__(self) -> int:
        return len(self.__log.data("time"))

    def step(self) -> None:
        self.__iteration += 1

    def data(self, key: str) -> npt.ArrayLike:
        return self.__log.data(key)

    def sensor(self, key: str) -> npt.ArrayLike:
        return self.__log.sensor(key)
