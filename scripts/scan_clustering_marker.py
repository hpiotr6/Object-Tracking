#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy
import tf
from sensor_msgs.msg import LaserScan, PointCloud2
from visualization_msgs.msg import Marker
import laser_geometry.laser_geometry as lg
import math
import sensor_msgs.point_cloud2 as pc2


# def marker_definer_CUBE():
#     cube = Marker()
#     cube.header.frame_id = "scan_clustering_frame"
#     #myMarker.header.stamp = rospy.Time.now()
#     cube.type = Marker.CUBE
#     cube.action = Marker.ADD

#     cube.pose.position.x = 0
#     cube.pose.position.y = 0
#     cube.pose.position.z = 0

#     cube.pose.orientation.x = 0.0
#     cube.pose.orientation.y = 0.0
#     cube.pose.orientation.z = 0.0
#     cube.pose.orientation.w = 1.0

#     #myMarker.lifetime = rospy.Duration
#     cube.scale.x = 1.0
#     cube.scale.y = 1.0
#     cube.scale.z = 1.0

#     cube.color.a = 1
#     cube.color.r = 0.0
#     cube.color.g = 0.9
#     cube.color.b = 0.2

#     return cube

# def marker_definer_POINTS():
#     points = Marker()
#     points.header.frame_id = "scan_clustering_frame"
#     #myMarker.header.stamp = rospy.Time.now()
#     points.type = Marker.POINTS
#     points.action = Marker.ADD

#     points.id = 0

#     points.pose.position.x = 0
#     points.pose.position.y = 0
#     points.pose.position.z = 0

#     points.pose.orientation.x = 0.0
#     points.pose.orientation.y = 0.0
#     points.pose.orientation.z = 0.0
#     points.pose.orientation.w = 1.0

#     #myMarker.lifetime = rospy.Duration
#     points.scale.x = 0.2
#     points.scale.y = 0.2
#     points.scale.z = 0.2

#     points.color.a = 1
#     points.color.r = 1
#     #points.color.g = 0.9
#     #points.color.b = 0.2

#     return points

# def scan_callback(scan):
#     global scans
#     scans = list(scan.ranges)
#     #print(scans , "\n")
#     #print(len(scans))
#     point_generatoir = pc2.read_points(pc2_msg)




if __name__== "__main__":

    #point_list
    # myMarker = marker_definer_CUBE()
    


    rospy.init_node('scan_clustering_node', anonymous=True)
    print("ready")
    pub = rospy.Publisher('/visualization_marker', Marker, queue_size=10)
    rospy.Subscriber( '/scan' , LaserScan, scan_callback)		
	
    rate=rospy.Rate(10) # 10Hz
    while not rospy.is_shutdown():
        # pub.publish(myMarker)#wyslanie treści markera
        rate.sleep()
	
    print("END")