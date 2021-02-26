#!/usr/bin/env python

import unittest
from tracker.hungarian import HungarianAlgorithm
from tracker.db import Point, Detection, DetectionsDB
from math import sqrt


class Hungarian_test(unittest.TestCase):
    def setUp(self):
        d1 = Detection([Point(x, y)
                        for x, y in [(0, 0), (0, 6), (4, 0), (4, 6)]])
        d2 = Detection([Point(x, y)
                        for x, y in [(1, 3), (1, 9), (9, 3), (9, 9)]])
        detections = DetectionsDB()
        detections.data = [d1, d2]
        predictions = [Point(-2, 0), Point(6, 7)]
        self.hung = HungarianAlgorithm(detections.centers,
                                       predictions)

    def test_init(self):
        self.assertEqual(len(self.hung.detections), 2)
        self.assertEqual(len(self.hung.predictions), 2)
        self.assertEqual([d.coords for d in self.hung.detections],
                         [(2, 3), (5, 6)])

    def test_create_matrix(self):
        matrix = self.hung.create_matrix()
        self.assertAlmostEqual(matrix[0][0], 5)
        self.assertAlmostEqual(matrix[0][1], sqrt(32))
        self.assertAlmostEqual(matrix[1][0], sqrt(85))
        self.assertAlmostEqual(matrix[1][1], sqrt(2))
