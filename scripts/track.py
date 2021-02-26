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

    # def get_detections(self):
    #     return self.detections

    # def get_obstacles(self):
    #     return self.obstaclesvisualization_marker

    def callback(self, data) -> None:
        #print("dupa")
        dupa = data.polygons
        # print(dupa)
        nowa_lista = []
        for i in range(len(dupa)):
            nowa_lista.append((dupa[i].points[0].x,dupa[i].points[0].y))
        self.detections = nowa_lista
        #self.detections = data.polygons
        self.tracker.detections = self.detections
        self.obstacles = self.tracker.obstacles.data
    
    def marker_function(self):
        
        marker_array = MarkerArray()
        marker_array.markers.clear()

        for o in self.obstacles:

            myMarker = Marker()
            myMarker.header.frame_id = "laser"
            myMarker.header.stamp = rospy.Time.now()
            myMarker.type = Marker.POINTS
            myMarker.action = Marker.ADD
            myMarker.pose.orientation.w = 1.0
            myMarker.id = o.id
            myMarker.scale.x = 0.02
            myMarker.scale.y = 0.02
            # myMarker.color.g = 1.0
            # myMarker.color.a = 1.0
            # myMarker.color.r= 0.0
            # myMarker.color.b = 0
            myMarker.lifetime = rospy.Duration(0.1)

            i = o.id
            (x , y) = (o.coords[0], o.coords[1])
            p = Point()
            p.x = x
            p.y = y
            p.z = 0
            color = ColorRGBA()
            color.g = ((i+1)%3)/2
            color.a = 1.0
            color.r= (i%3)/2
            color.b = ((i+2)%3)/2
            myMarker.points.append(p)
            myMarker.colors.append(color)
            

            marker_array.markers.append(myMarker)
        print(marker_array.markers)
        return marker_array


if __name__ == "__main__":
    rospy.init_node('tracking_node', anonymous=True)
    tracking_node = Tracking()

    #pub = rospy.Publisher('/middle_points_filtered', Polygon, queue_size=10)
    pub2 = rospy.Publisher('/punkciki', MarkerArray, queue_size=10)
    rate = rospy.Rate(10)  # 10Hz
    while not rospy.is_shutdown():
        list_of_middle_points_filtered = Polygon()
    #   print(len(tracking_node.detections))
        # print([detection.coords for detection
        #        in tracking_node.detections])
        #print([obstacle.id for obstacle in tracking_node.obstacles])
        for o in tracking_node.obstacles:
            print(o.coords)
            p = Point32()
            p.x = o.coords[0]
            p.y = o.coords[1]
            list_of_middle_points_filtered.points.append(p)
        print('\n')
        marker = tracking_node.marker_function()
        pub2.publish(marker)
        #print(list_of_middle_points_filtered)
        #pub.publish(list_of_middle_points_filtered)

        rate.sleep()
