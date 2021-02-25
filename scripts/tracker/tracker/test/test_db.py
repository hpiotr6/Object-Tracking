#!/usr/bin/env python

import unittest
from tracker.db import Obstacle, DetectionsDB, Point


class Obstacle_test(unittest.TestCase):
    def test_init(self):
        o1 = Obstacle([Point(2, 3), Point(4, 5), Point(7, 8), Point(10, 10)],
                      13, 15)
        self.assertEqual(len(o1.vertices), 4)
        self.assertEqual(o1.vertices[0].coords, (2, 3))
        self.assertEqual(o1.vertices[3].coords, (10, 10))
        self.assertEqual(o1.center.coords, (13, 15))


# class DetectionsDB_test(unittest.TestCase):
#     def test_init(self):
#         db = DetectionsDB()
#         db.data = [(1, 2), (3, 4)]
#         c1 = [point.coords for point in db.data]
#         self.assertEqual(c1, [(1, 2), (3, 4)])
#         db.data = [(3, 5), (7, 8)]
#         c2 = [point.coords for point in db.data]
#         self.assertEqual(c2, [(3, 5), (7, 8)])