import unittest
from tracker.db import Point, Detection, Obstacle, DB, DetectionsDB, ObstaclesDB


class Obstacle_test(unittest.TestCase):
    def test_init(self):
        o1 = Obstacle(12., 14.)
        o2 = Obstacle(13.4, 15.2)
        self.assertEqual(o1.coords, (12., 14.))
        self.assertEqual(o1.id, 1)
        self.assertEqual(o2.coords, (13.4, 15.2))
        self.assertEqual(o2.id, 2)


class DetectionsDB_test(unittest.TestCase):
    def test_init(self):
        db = DetectionsDB()
        db.data = [(1, 2), (3, 4)]
        c1 = [point.coords for point in db.data]
        self.assertEqual(c1, [(1, 2), (3, 4)])
        db.data = [(3, 5), (7, 8)]
        c2 = [point.coords for point in db.data]
        self.assertEqual(c2, [(3, 5), (7, 8)])


class ObstaclesDB_test(unittest.TestCase):
    def test_assign_obstacles(self):
        obs = ObstaclesDB()
        o1 = Obstacle(2, 3)
        o2 = Obstacle(5, 6)
        obs.add(o1)
        obs.add(o2)
        matchs = ((((1, 0.5), (2, 3)), 1.5), (((4, 7), (5, 6)), 4.5))
        obs.assign(matchs)
        self.assertEqual(o1.coords, (1, 0.5))
        self.assertEqual(o2.coords, (4, 7))
