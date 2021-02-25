#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy
import math
from sensor_msgs.msg import LaserScan
from sensor_msgs.msg import PointCloud2 as pc2
from laser_geometry import LaserProjection
from sensor_msgs import point_cloud2
from visualization_msgs.msg import Marker, MarkerArray
from geometry_msgs.msg import Point, Polygon
from std_msgs.msg import ColorRGBA,Float64MultiArray

from scan_clustering_kalman.msg import PolygonArray

import string
import itertools
import math
def clusters(visualization_marker):
    global markers

    markers = visualization_marker.markers
    #[0].points



class Rectangle():

    def __init__(self,p1,p2,p3,p4):
        self.p1 = p1
        self.p2 = p2
        self.p3 = p3
        self.p4 = p4



class rectangleMaker():

    def __init__(self):
        self.reset()



    def reset(self):
        self.clusters_list = []
        self.numberOfPoints = []
        self.lowest_point = []
        self.highest_point = []
        self.points = []
        self.closest_point = []
        self.other_point = []
        self.middle_point_tuple = []


    def findClosestPoint(self, _points, h_point, l_point):
        _max = 0
        newMax = 0
        closest_point = 0
        if _points:
            for point in _points:
                A = h_point.y-l_point.y
                print(A)
                
                B = h_point.x - l_point.x
                print(B)
                if math.sqrt(A*A+B*B)!=0:
                    newMax = abs(A*(l_point.x - point.x) + B*(point.y - l_point.y))/math.sqrt(A*A+B*B)
                else:
                    newMax = abs(h_point.x - point.x)
                if newMax > _max:
                    _max = newMax
                    closest_point = point
            return closest_point
    

    def findLowestHighestPoint(self, _points):
        min_kat = float("inf")
        max_kat = 0
        points = [None, None]
        for point in _points:
            degrees = 360/math.pi * math.atan2(point.y,point.x)
            for point2 in _points:
                degrees2 = 360/math.pi * math.atan2(point2.y,point2.x)
                if max_kat < abs(degrees-degrees2):
                    max_kat = abs(degrees-degrees2)
                    points = []
                    points.append((point,point2))
        return points[0]



    def getMiddlePoint(self, i):
        p = Point()
        p.x = (self.closest_point[i].x+self.other_point[i].x)/2
        p.y = (self.closest_point[i].y+self.other_point[i].y)/2
        p.z = 0
        return p


    def rectangleMaker(self):
        self.reset()
        length = len(markers)
        self.markers = markers
        for i in range(len(markers)):
            
            _points = self.markers[i].points
            self.numberOfPoints = len(_points)
            while not self.numberOfPoints>3:
                self.numberOfPoints = len(_points)
            self.clusters_list.append(markers[i])
            self.lowest_point.append(self.findLowestHighestPoint(_points)[0])
            self.highest_point.append(self.findLowestHighestPoint(_points)[1])
            self.points.append(_points)
            self.closest_point.append(self.findClosestPoint(_points, self.highest_point[i], self.lowest_point[i]))
            self.other_point.append(self.findOtherPoint(i))
            self.middle_point_tuple.append((self.getMiddlePoint(i).x, self.getMiddlePoint(i).y))
        return self.marker_function()

    def findOtherPoint(self, i):
        try:
            x = self.lowest_point[i].x - self.closest_point[i].x
            y = self.lowest_point[i].y - self.closest_point[i].y
        except Exception as e:
            return None
            print(e)
        new_x = self.highest_point[i].x+x
        new_y = self.highest_point[i].y+y
        p = Point()
        p.x = new_x
        p.y = new_y
        p.z = 0
        return p

    def getRectanglePoints(self):
        return self.highest_point, self.lowest_point, self.closest_point, self.other_point

    def getMiddlePointsTuple(self):
        msg = PolygonArray()
        



        floatArray = Float64MultiArray()
        for element in range(len(self.closest_point)):
            p1 = Polygon()
            p1.points.reserve(5)
            p1.points.append(self.getMiddlePoint(i))
            p1.points.append(self.highest_point(i))
            p1.points.append(self.lowest_point(i))
            p1.points.append(self.closest_point(i))
            p1.points.append(self.other_point(i))
            p1.header = header
            msg.polygons.append(p1)
        return msg


    def marker_function(self):
        marker_array = MarkerArray()
        marker_array.markers.clear()
        for i,cluster in enumerate(self.clusters_list):
            myMarker = Marker()
            myMarker.id = i
            myMarker.header.frame_id = "laser"
            myMarker.header.stamp = rospy.Time.now()
            ns = "Line"

            myMarker.lifetime = rospy.Duration(0.2)

            #print(ns, type(ns))

            myMarker.action = 0

            # myMarker.pose.position.x = 0
            # myMarker.pose.position.y = 0
            # myMarker.pose.position.z = 0

            # myMarker.pose.orientation.x = 0.0
            # myMarker.pose.orientation.y = 0.0
            # myMarker.pose.orientation.z = 0.0
            # myMarker.pose.orientation.w = 1.0
            line_color = ColorRGBA()
            line_color.r = (i%3)/2
            line_color.g = ((i+1)%3)/2
            line_color.b = ((i+2)%3)/2
            line_color.a = 1.0
            if len(cluster.points) > 5:
                myMarker.type = Marker.LINE_STRIP
                myMarker.scale.x = 0.02
                myMarker.points.append(self.lowest_point[i])
                myMarker.points.append(self.closest_point[i])
                myMarker.colors.append(line_color)
                myMarker.colors.append(line_color)
    

                marker_array.markers.append(myMarker)
            
        

                myMarker.type = Marker.LINE_STRIP
                myMarker.scale.x = 0.02
                myMarker.points.append(self.highest_point[i])
                myMarker.points.append(self.closest_point[i])
                myMarker.colors.append(line_color)
                myMarker.colors.append(line_color)
                marker_array.markers.append(myMarker)



                myMarker.type = Marker.LINE_STRIP
                myMarker.scale.x = 0.02

                myMarker.points.append(self.lowest_point[i])
                myMarker.points.append(self.other_point[i])
                myMarker.colors.append(line_color)
                myMarker.colors.append(line_color)


                marker_array.markers.append(myMarker)





                myMarker.type = Marker.LINE_STRIP
                myMarker.scale.x = 0.02

                myMarker.points.append(self.highest_point[i])
                myMarker.points.append(self.other_point[i])
                myMarker.colors.append(line_color)
                myMarker.colors.append(line_color)


                marker_array.markers.append(myMarker)
        return marker_array

            


if __name__== "__main__":
    
    markers = []
    rospy.Subscriber('/visualization_marker', MarkerArray, clusters)
    rospy.init_node("rectangleMaker_node", anonymous=True)
    print("ready")
    rate=rospy.Rate(10) # 10Hz

    pub = rospy.Publisher('/rectangleMakerv2', MarkerArray, queue_size=10)
    pub2 = rospy.Publisher('/middlePoint', PolygonArray, queue_size=10)
    rectangleMaker_node = rectangleMaker()

    while not rospy.is_shutdown():
        print("\n\n//////////////////// NEXT LOOP ////////////////////\n")
        rate.sleep()
        list_of_markers = rectangleMaker_node.rectangleMaker()
        middlePointsTuple= rectangleMaker_node.getMiddlePointsTuple()
        rate.sleep()
        rectanglePoints = rectangleMaker_node.getRectanglePoints()
        pub.publish(list_of_markers)
        pub2.publish(middlePointsTuple)
    
    print("END")