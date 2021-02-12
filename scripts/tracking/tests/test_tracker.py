import unittest
from tracker import Tracker
from db import Point


class Tracker_test(unittest.TestCase):
    def setUp(self):
        self.tracker = Tracker()
        self.tracker.detections = [(2, 3), (7, 1)]

    # def test_init(self):
    #     self.tracker.detections = [(1, 2), (3, 4)]
    #     c1 = [point.coords for point in self.tracker.detections.data]
    #     self.assertEqual(c1, [(1, 2), (3, 4)])
    #     self.tracker.detections = [(3, 5), (7, 8)]
    #     c2 = [point.coords for point in self.tracker.detections.data]
    #     self.assertEqual(c2, [(3, 5), (7, 8)])        

    # def test_check_len0(self):
    #     self.assertEqual(self.tracker.obstacles.data[0].coords, (2, 3))
    #     self.assertEqual(self.tracker.obstacles.data[0].id, 1)
    #     self.assertEqual(self.tracker.obstacles.data[1].coords, (7, 1))
    #     self.assertEqual(self.tracker.obstacles.data[1].id, 2)

    def test_update_obstacles_same(self):
        self.tracker.update_obstacles([0, 1], [0, 1])
        self.assertEqual(len(self.tracker.obstacles.data), 2)
        self.assertEqual(self.tracker.obstacles.data[0].coords, (2, 3))
        self.assertEqual(self.tracker.obstacles.data[0].id, 1)
        self.assertEqual(self.tracker.obstacles.data[1].coords, (7, 1))
        self.assertEqual(self.tracker.obstacles.data[1].id, 2)
        self.tracker.update_obstacles([0, 1], [0, 1])
        self.assertEqual(self.tracker.obstacles.data[1].id, 2)

    def test_update_obstacles_reverse(self):
        self.tracker.update_obstacles([0, 1], [1, 0])
        self.assertEqual(len(self.tracker.obstacles.data), 2)
        self.assertEqual(self.tracker.obstacles.data[0].coords, (7, 1))
        self.assertEqual(self.tracker.obstacles.data[0].id, 1)
        self.assertEqual(self.tracker.obstacles.data[1].coords, (2, 3))
        self.assertEqual(self.tracker.obstacles.data[1].id, 2)

    def test_update_obstacles_more_detections(self):
        self.tracker.detections = [(2, 3), (7, 1), (5, 6)]
        self.tracker.update_obstacles([0, 2], [1, 0])
        self.assertEqual(self.tracker.obstacles.data[0].coords, (5, 6))
        self.assertEqual(self.tracker.obstacles.data[1].coords, (2, 3))
        self.assertEqual(self.tracker.obstacles.data[0].id, 1)
        self.assertEqual(self.tracker.obstacles.data[1].id, 2)
        self.assertEqual(len(self.tracker.obstacles.data), 3)
        self.assertEqual(self.tracker.obstacles.data[2].coords, (7, 1))
        self.assertEqual(self.tracker.obstacles.data[2].id, 3)

    def test_update_obstacles_less_detections(self):
        self.tracker.detections = [(10, 20)]
        self.tracker.update_obstacles([0], [1])
        self.assertEqual(len(self.tracker.obstacles.data), 2)
        self.assertEqual(self.tracker.obstacles.data[1].coords, (10, 20))
        self.assertEqual(self.tracker.obstacles.data[1].id, 2)


    # def test_find_match_same_size(self):
    #     preds = {1: Point(3, 4),
    #              2: Point(8, 2)}
    #     matchings = self.tracker.find_match(preds)
    #     self.assertEqual(len(matchings), 2)
    #     self.assertEqual(matchings[1][0][0], (2, 3))
    #     self.assertEqual(matchings[1][0][1], (3, 4))
    #     self.assertGreater(matchings[1][1], 1)
    #     self.assertEqual(matchings[0][0][0], (7, 1))
    #     self.assertEqual(matchings[0][0][1], (8, 2))
    #     self.assertGreater(matchings[0][1], 1)

    # def test_find_match_detecions_grater(self):
    #     self.tracker.detections = [(2, 3), (7, 1), (8, 8)]
    #     preds = {1: Point(3, 4),
    #              2: Point(8, 2)}
    #     matchings = self.tracker.find_match(preds)
    #     self.assertEqual(len(matchings), 2)
    #     self.assertEqual(matchings[1][0][0], (2, 3))
    #     self.assertEqual(matchings[1][0][1], (3, 4))
    #     self.assertGreater(matchings[1][1], 1)
    #     self.assertEqual(matchings[0][0][0], (7, 1))
    #     self.assertEqual(matchings[0][0][1], (8, 2))
    #     self.assertGreater(matchings[0][1], 1)

    # def test_find_match_predictions_grater(self):
    #     preds = {1: Point(3, 4),
    #              2: Point(8, 2),
    #              3: Point(8, 8)}
    #     matchings = self.tracker.find_match(preds)
    #     self.assertEqual(len(matchings), 2)
    #     self.assertEqual(matchings[1][0][0], (2, 3))
    #     self.assertEqual(matchings[1][0][1], (3, 4))
    #     self.assertGreater(matchings[1][1], 1)
    #     self.assertEqual(matchings[0][0][0], (7, 1))
    #     self.assertEqual(matchings[0][0][1], (8, 2))
    #     self.assertGreater(matchings[0][1], 1)

    # def test_find_match_predictions_other(self):
    #     self.tracker.detections = [(1, 2), (3, 4)]
    #     preds = {1: Point(3, 4),
    #              2: Point(8, 2)}
    #     matchings = self.tracker.find_match(preds)
    #     self.assertEqual(len(matchings), 2)


    # def test_assign_obstacles(self):
    #         print([point.coords for point in self.tracker.obstacles.data])
    #         print([point.coords for point in self.tracker.detections.data])
    #         self.assertEqual(1, 1)


if __name__ == '__main__':
    unittest.main()
