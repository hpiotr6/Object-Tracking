import unittest
from tracker.hungarian import HungarianAlgorithm
from tracker.db import DetectionsDB


class Hungarian_test(unittest.TestCase):
    def test_ac(self):
        db = DetectionsDB()
        db.data = [(2, 3), (7, 1)]
        predictions = [(4, 5), (6, 7)]
        hung = HungarianAlgorithm(db.data, predictions)
        self.assertEqual(hung.detections[0].coords, (2, 3))
