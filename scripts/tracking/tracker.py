from db import Point, Detection, Obstacle, DB, DetectionsDB, ObstaclesDB
import numpy as np


# class SingletonMeta(type):
#     """
#     The Singleton class can be implemented in different ways in Python. Some
#     possible methods include: base class, decorator, metaclass. We will use the
#     metaclass because it is best suited for this purpose.
#     """

#     _instances = {}

#     def __call__(cls, *args, **kwargs):
#         """
#         Possible changes to the value of the `__init__` argument do not affect
#         the returned instance.
#         """
#         if cls not in cls._instances:
#             instance = super().__call__(*args, **kwargs)
#             cls._instances[cls] = instance
#         return cls._instances[cls]


class Tracker():
    """
    A class to represent a tracker.

    Attributes
    ----------
    obstacles: ObstaclesDB object
    detections: DetectionsDB object

    Methods:
    --------
    """
    def __init__(self):
        self.__obstacles = ObstaclesDB()
        self.__detections = DetectionsDB()

    @property
    def predictions(self):
        return self.__obstacles.predict()

    @property
    def detections(self):
        return self.__detections

    @detections.setter
    def detections(self, new_detections: DetectionsDB):
        self.__detections.data = new_detections
        self.check()

    @property
    def obstacles(self):
        return self.__obstacles

    def create_obstacle(self, x: float, y: float) -> None:
        o = Obstacle(x, y)
        self.obstacles.add(o)

    def update_obstacles(self, row_inds: list, col_inds: list) -> None:
        for row, col in zip(row_inds, col_inds):
            self.obstacles.data[col].coords = self.detections.data[row].coords
        row_max = max(row_inds)
        full_rows = [ind for ind in range(row_max + 1)]
        res = set(full_rows) - set(row_inds)
        if len(res):
            for ind in res:
                x, y = self.detections.data[ind].coords
                self.create_obstacle(x, y)

    def check(self):
        if len(self.obstacles.data) == 0:
            for detection in self.detections.data:
                self.create_obstacle(detection.x, detection.y)
        else:
            pass
            # obstacle_predictions = self.obstacles.predict()
            # matching = self.find_match(obstacle_predictions)
            # self.obstacles.assign(matching)
            # self.update_obstacles()
