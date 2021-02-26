#!/usr/bin/env python

from .db import Obstacle, DetectionsDB, ObstaclesDB
from .hungarian import HungarianAlgorithm


class Tracker():
    """
    A class to represent a tracker.

    Attributes
    ----------
    obstacles: ObstaclesDB object
    detections: DetectionsDB object

    Methods
    --------
    create_obstacle(x, y):
        Create obstacle form coordinates
    update_obstacles(row_inds, col_inds):
        Given indices from cost matrix assign detections to obstacles
        if exists, otherwise create obstacle
    check():
        Check if obstacles exists than create obstacles,
        otherwise update obstacles
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

    def create_obstacle(self, vertices: list) -> None:
        o = Obstacle(vertices)
        self.obstacles.add(o)

    def update_obstacles(self, row_inds: list,
                         col_inds: list) -> None:
        for row, col in zip(row_inds, col_inds):
            self.obstacles.data[col].vertices = \
                self.detections.data[row].vertices
        detections_num = len(self.detections.data)
        full_rows = [ind for ind in range(detections_num)]
        res = set(full_rows) - set(row_inds)
        if len(res):
            for ind in res:
                verts = self.detections.data[ind].vertices
                self.create_obstacle(verts)

    def check(self) -> None:
        if len(self.obstacles.data) == 0:
            for detection in self.detections.data:
                self.create_obstacle(detection.vertices)
        else:
            predictions = self.obstacles.predict()
            hung = HungarianAlgorithm(self.detections.centers, predictions)
            row_ind, col_ind = hung.get_indices()
            self.update_obstacles(row_ind, col_ind)
