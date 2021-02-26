#!/usr/bin/env python
import rospy
import math
from sensor_msgs.msg import LaserScan
from sensor_msgs.msg import PointCloud2 as pc2
from laser_geometry import LaserProjection
from sensor_msgs import point_cloud2


class Laser_distance_filter_and_2PC():
    def __init__(self):
        self.max_range = rospy.get_param("max_range")
        self.min_range = rospy.get_param("min_range")
        self.x = []
        self.y = []
        self.sub = rospy.Subscriber("/scan", LaserScan, self.scan_callback)
        self.pub = rospy.Publisher("/scan_distance_filter", LaserScan, queue_size=1)
        self.scan = LaserScan()

    def scan_callback(self, data):
        self.scan = data

    def distance_filter(self):
    # filtorwanie na podstawie dlugosci skanu        
        j=1
        ranges = list(self.scan.ranges)

        for i in range(0, len(ranges)):
            if(ranges[i]>self.max_range)or(ranges[i]<self.min_range):
            
                ranges[i] = float("Nan")
        self.scan.ranges = tuple(ranges)

        intensities = list(self.scan.intensities)
        intensities +=[0]

        for i in range(1,len(self.scan.ranges)):

            if(abs(self.scan.ranges[i]-self.scan.ranges[i-1])<0.1):
                intensities +=[j]
            else:
                intensities+=[0]
                j=j+1
        
        self.scan.intensities = tuple(intensities)


    def publish(self):
        self.pub.publish(self.scan)

if __name__== "__main__":

    rospy.init_node('laser_node', anonymous=True)
    print("ready")
    laser_node = Laser_distance_filter_and_2PC()
    rate=rospy.Rate(10) # 10Hz
    while not rospy.is_shutdown():
        laser_node.distance_filter()
        laser_node.publish()
        rate.sleep()
    
    print("END")
