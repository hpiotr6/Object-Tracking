#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy
from sensor_msgs.msg import LaserScan
from sensor_msgs.msg import PointCloud2 as pc2
from laser_geometry import LaserProjection
from sensor_msgs import point_cloud2


class Laser2PC():
    def __init__(self):
        self.laser_proj = LaserProjection()
        self.pc_pub = rospy.Publisher("/laserPointCloud", pc2, queue_size=1)
        self.laser_sub = rospy.Subscriber("/laser/filtered", LaserScan, self.scan_callback)
    
    def scan_callback(self, data):

        cloud_out = self.laser_proj.projectLaser(data)
        
        cloud_out.header.frame_id = "laser"

        self.pc_pub.publish(cloud_out)

        # print(cloud_out)



if __name__ == '__main__':
    rospy.init_node("laser_to_point_cloud")
    l2pc = Laser2PC()
    
    rospy.spin()