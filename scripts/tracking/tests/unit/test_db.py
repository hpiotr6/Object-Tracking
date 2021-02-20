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
    pass
