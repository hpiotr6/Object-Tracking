import unittest
from hungarian import HungarianAlgorithm
from db import Point, Detection, Obstacle, DB, DetectionsDB, ObstaclesDB
from math import sqrt


class Hungarian_test(unittest.TestCase):
    def setUp(self):
        detections = [Detection(2, 3), Detection(5, 6)]
        predictions = [Point(-2, 0), Point(6, 7)]
        self.hung = HungarianAlgorithm(detections, predictions)

    def test_init(self):
        self.assertEquals(len(self.hung.detections), 2)
        self.assertEquals(len(self.hung.predictions), 2)
    
    def test_create_matrix(self):
        matrix = self.hung.create_matrix()
        self.assertAlmostEqual(matrix[0][0], 5)
        self.assertAlmostEqual(matrix[0][1], sqrt(32))
        self.assertAlmostEqual(matrix[1][0], sqrt(85))
        self.assertAlmostEqual(matrix[1][1], sqrt(2))
