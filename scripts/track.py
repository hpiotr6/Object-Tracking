#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy
from std_msgs.msg import Float64MultiArray, ColorRGBA
from geometry_msgs.msg import Polygon, Point32
from tracker import Tracker
from visualization_msgs.msg import Marker, MarkerArray
from geometry_msgs.msg import Point

from scan_clustering_kalman.msg import PolygonArray


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
        self.sub = rospy.Subscriber('/middlePoint', PolygonArray,
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
        self.__obstacles = value

    @property
    def detections(self):
        return self.__detections

    @detections.setter
    def detections(self, value):
        self.__detections = value

    def callback(self, data) -> None:
        self.detections = data.polygons
        self.tracker.detections = self.detections
        self.obstacles = self.tracker.obstacles.data
    
    def marker_function(self):
        marker_array = MarkerArray()
        marker_array.markers.clear()
        for obstacle in self.obstacles:
            myMarker = Marker()
            myMarker.id = obstacle.id
            myMarker.header.frame_id = "laser"
            myMarker.header.stamp = rospy.Time.now()
            ns = "Line"
            myMarker.lifetime = rospy.Duration(0.2)
            myMarker.action = 0
            i = obstacle.id
            line_color = ColorRGBA()
            line_color.r = (i%3)/2
            line_color.g = ((i+1)%3)/2
            line_color.b = ((i+2)%3)/2
            line_color.a = 1.0
            myMarker.type = Marker.LINE_STRIP
            myMarker.scale.x = 0.02

            for point in obstacle.vertices:
                point.z = 0
                myMarker.points.append(point)
                myMarker.colors.append(line_color)
  
            marker_array.markers.append(myMarker)
        return marker_array


if __name__ == "__main__":
    rospy.init_node('tracking_node', anonymous=True)
    tracking_node = Tracking()
    print('ready')
    pub = rospy.Publisher('/middle_points_filtered', Polygon, queue_size=10)
    pub2 = rospy.Publisher('/punkciki', MarkerArray, queue_size=10)
    rate = rospy.Rate(10)  # 10Hz
    while not rospy.is_shutdown():
        list_of_middle_points_filtered = Polygon()
        marker = tracking_node.marker_function()
        pub2.publish(marker)

        rate.sleep()
