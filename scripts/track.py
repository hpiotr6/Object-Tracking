#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy
from std_msgs.msg import Float64MultiArray
from tracker import Tracker


class Tracking:
    """
    Class wrapper for tracking obstacles in ROS.

    Attributes
    ----------
    sub: rospy.Subscriber
    pub: rospy.Publisher
    detections: list of tuples of float's coordinates [(float, float), (), ...]
    obstacles: list of Obstacle's objects
    tracker: Tracker Object

    Methods
    -------
    callback: ...
    """
    def __init__(self):
        self.sub = rospy.Subscriber('/rectangleMakerv2', Float64MultiArray,
                                    self.callback)
        self.pub = rospy.Publisher('/tracking', Float64MultiArray,
                                   queue_size=10)
        self.__detections = []
        self.__obstacles = []
        self.tracker = Tracker()

    @property
    def obstacles(self):
        return self.__obstacles

    @obstacles.setter
    def obstacles(self, value):
        self.obstacles = value

    @property
    def detections(self):
        return self.__detections

    @detections.setter
    def detections(self, value):
        self.detections = value

    def callback(self, data) -> None:
        self.detections = data.data
        self.tracker.detections = self.detections
        self.obstacles = self.tracker.obstacles.data


if __name__ == "__main__":
    rospy.init_node('tracking_node', anonymous=True)
    tracking_node = Tracking()
    rate = rospy.Rate(10)  # 10Hz
    while not rospy.is_shutdown():
        print([detection.coords for detection
               in tracking_node.detections])
        print([obstacle.coords for obstacle in tracking_node.obstacles])
        rate.sleep()
