#!/usr/bin/env python

from filterpy.kalman import KalmanFilter
import numpy as np


class Point():
    """
    A class to represent a point.

    Attributes
    ----------
    x : float
    y : float
    coords : tuple
    """

    def __init__(self, x: float, y: float):
        self.__x = x
        self.__y = y
        self.__coords = (x, y)

    @property
    def coords(self):
        return self.__coords

    @coords.setter
    def coords(self, value: tuple):
        self.__coords = value

    @property
    def x(self):
        return self.__x

    @property
    def y(self):
        return self.__y


class Detection(Point):
    """
    A class to represent a detection, subclasses Point.

    Attributes
    ----------
    x : float
    y : float
    coords : tuple
    """
    def __init__(self, x: float, y: float):
        super().__init__(x, y)


class Obstacle(Point):
    """
    A class to represent an obstacle, subclasses Point.

    Attributes
    ----------
    x : float
    y : float
    coords : tuple
    id : static int

    Methods:
    --------
    predict():
        Kalman prediction
    get_prediction():
        Give Point object from state vector.
    """
    __id = 0

    def __init__(self, x: float, y: float):
        super().__init__(x, y)
        Obstacle.__id += 1
        self.__id = Obstacle.__id

        self.kf = KalmanFilter(dim_x=4, dim_z=2)
        R_std = 0.35
        Q_std = 0.001
        self.setup_kalman(R_std, Q_std)

    def __del__(self):
        Obstacle.__id -= 1

    @property
    def id(self):
        return self.__id

    def setup_kalman(self, R_std, Q_std, dt=1) -> None:
        self.kf.F = np.array([[1, dt, 0,  0],
                              [0,  1, 0,  0],
                              [0,  0, 1, dt],
                              [0,  0, 0,  1]])
        self.kf.u = 0.
        self.kf.H = np.array([[1, 0, 0, 0],
                              [0, 0, 1, 0]])

        self.kf.R = np.eye(2) * R_std**2
        self.kf.Q = np.eye(4) * Q_std**2
        self.kf.x = np.array([[0, 0, 0, 0]]).T
        self.kf.P = np.eye(4) * 500.

    def predict(self) -> Point:
        z = np.array([[self.x, self.y]]).T
        self.kf.predict()
        self.kf.update(z)

        return self.get_prediction()

    def get_prediction(self) -> Point:
        x, y = self.kf.x[0], self.kf.x[2]
        return Point(x, y)


class DB():
    """
    A class to represent an database.

    Attributes
    ----------
    data: list

    Methods
    --------
    add():
        Append new DB object.
    """
    def __init__(self):
        self.__data = []

    @property
    def data(self):
        return self.__data

    def add(self, object_) -> None:
        self.__data.append(object_)


class DetectionsDB():
    """
    A class to represent an detection database.

    Attributes
    ----------
    data: list
    """
    def __init__(self):
        self.__data = []

    @property
    def data(self):
        return self.__data

    @data.setter
    def data(self, detections: list):
        self.__data = []
        for x, y in detections:
            self.__data.append(Detection(x, y))


class ObstaclesDB(DB):
    """
    A class to represent a obstacle database, subclasses DB.

    Attributes
    ----------
    data: list

    Methods
    --------
    add():
        Append new ObstacleDB object.
    predict():
        For each obstacle predict and return list of predictions.
    """
    def __init__(self):
        super().__init__()

    def predict(self) -> list:
        preds = []
        for o in self.data:
            preds.append(o.predict())
        return preds
