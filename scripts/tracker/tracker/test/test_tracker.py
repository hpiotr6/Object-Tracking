#!/usr/bin/env python

import unittest
from tracker import Tracker
from tracker.db import Detection, Point


class Tracker_test(unittest.TestCase):
    def setUp(self):
        self.tracker = Tracker()
        d1 = Detection([Point(x, y)
                        for x, y in [(0, 0), (0, 6), (4, 0), (4, 6)]])
        d2 = Detection([Point(x, y)
                        for x, y in [(1, 3), (1, 9), (9, 3), (9, 9)]])
        self.tracker.detections = [d1, d2]

    # def test_init(self):
    #     d1 = Detection([Point(x, y)
    #                     for x, y in [(0, 0), (0, 6), (4, 0), (4, 6)]])
    #     d2 = Detection([Point(x, y)
    #                     for x, y in [(1, 3), (1, 9), (9, 3), (9, 9)]])
    #     self.tracker.detections = [d2, d1]
    #     c1 = [point.coords for point in self.tracker.detections.data]
    #     self.assertEqual(c1, [(1, 2), (3, 4)])
    #     self.tracker.detections = [(3, 5), (7, 8)]
    #     c2 = [point.coords for point in self.tracker.detections.data]
    #     self.assertEqual(c2, [(3, 5), (7, 8)])

#     def test_check_len0(self):
#         self.assertEqual(self.tracker.obstacles.data[0].coords, (2, 3))
#         self.assertEqual(self.tracker.obstacles.data[0].id, 1)
#         self.assertEqual(self.tracker.obstacles.data[1].coords, (7, 1))
#         self.assertEqual(self.tracker.obstacles.data[1].id, 2)

#     def test_update_obstacles_same(self):
#         self.tracker.update_obstacles([0, 1], [0, 1])
#         self.assertEqual(len(self.tracker.obstacles.data), 2)
#         self.assertEqual(self.tracker.obstacles.data[0].coords, (2, 3))
#         self.assertEqual(self.tracker.obstacles.data[0].id, 1)
#         self.assertEqual(self.tracker.obstacles.data[1].coords, (7, 1))
#         self.assertEqual(self.tracker.obstacles.data[1].id, 2)
#         self.tracker.update_obstacles([0, 1], [0, 1])
#         self.assertEqual(self.tracker.obstacles.data[1].id, 2)

#     def test_update_obstacles_reverse(self):
#         self.tracker.update_obstacles([0, 1], [1, 0])
#         self.assertEqual(len(self.tracker.obstacles.data), 2)
#         self.assertEqual(self.tracker.obstacles.data[0].coords, (7, 1))
#         self.assertEqual(self.tracker.obstacles.data[0].id, 1)
#         self.assertEqual(self.tracker.obstacles.data[1].coords, (2, 3))
#         self.assertEqual(self.tracker.obstacles.data[1].id, 2)

#     def test_update_obstacles_more_detections(self):
#         self.tracker.detections = [(2, 3), (7, 1), (5, 6), (10, 10)]
#         self.tracker.update_obstacles([0, 2], [1, 0])
#         self.assertEqual(self.tracker.obstacles.data[0].coords, (5, 6))
#         self.assertEqual(self.tracker.obstacles.data[1].coords, (2, 3))
#         self.assertEqual(self.tracker.obstacles.data[0].id, 1)
#         self.assertEqual(self.tracker.obstacles.data[1].id, 2)
#         self.assertEqual(self.tracker.obstacles.data[2].coords, (7, 1))
#         self.assertEqual(self.tracker.obstacles.data[2].id, 3)

#     def test_update_obstacles_less_detections(self):
#         self.tracker.detections = [(10, 20)]
#         self.tracker.update_obstacles([0], [1])
#         self.assertEqual(len(self.tracker.obstacles.data), 2)
#         self.assertEqual(self.tracker.obstacles.data[1].coords, (10, 20))
#         self.assertEqual(self.tracker.obstacles.data[1].id, 2)


# if __name__ == '__main__':
#     unittest.main()
