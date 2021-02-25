#!/usr/bin/env python

import unittest
from tracker.db import Obstacle, DetectionsDB, Point, Polygon


class Obstacle_test(unittest.TestCase):
    def setUp(self):
        self.o1 = Obstacle([Point(2, 3), Point(4, 5),
                            Point(7, 8), Point(10, 10)])

    def test_init(self):
        self.assertEqual(len(self.o1.vertices), 4)
        self.assertEqual(self.o1.vertices[0].coords, (2, 3))
        self.assertEqual(self.o1.vertices[3].coords, (10, 10))
        self.assertIsInstance(self.o1.center, Point)


# class DetectionsDB_test(unittest.TestCase):
#     def test_init(self):
#         db = DetectionsDB()
#         db.data = [(1, 2), (3, 4)]
#         c1 = [point.coords for point in db.data]
#         self.assertEqual(c1, [(1, 2), (3, 4)])
#         db.data = [(3, 5), (7, 8)]
#         c2 = [point.coords for point in db.data]
#         self.assertEqual(c2, [(3, 5), (7, 8)])

class Polygon_test(unittest.TestCase):
    def setUp(self):
        self.p1 = Polygon([Point(2, 6), Point(7, 6),
                           Point(2, 3), Point(7, 3)])

    def test_compute_center(self):
        center1 = self.p1.compute_center()
        self.assertEqual(center1.coords, (4.5, 4.5))

    def test_init(self):
        self.assertEqual(self.p1.vertices[0].coords, (2, 6))
        self.assertEqual(self.p1.vertices[1].coords, (7, 6))
        self.assertEqual(self.p1.vertices[2].coords, (2, 3))
        self.assertEqual(self.p1.vertices[3].coords, (7, 3))
        self.assertEqual(self.p1.center.coords, (4.5, 4.5))
