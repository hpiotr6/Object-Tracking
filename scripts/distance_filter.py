#!/usr/bin/env python
import rospy
import math
from sensor_msgs.msg import LaserScan 

class Laser(object):
    def __init__(self):
        self.max_range = 1.0
        self.min_range = 0.2
        self.pub = rospy.Publisher("/laser/filtered", LaserScan, queue_size=10)
        self.x = []
        self.y = []
        rospy.Subscriber("/scan", LaserScan, self.scan_callback)

    def scan_callback(self, data):
        j=1
        ranges = list(data.ranges)

        for i in range(0, len(ranges)):
            if(ranges[i]>self.max_range)or(ranges[i]<self.min_range):
            
                ranges[i] = float("Nan")
        data.ranges = tuple(ranges)

        intensities = list(data.intensities)
        intensities +=[0]

        for i in range(1,len(data.ranges)):

            if(abs(data.ranges[i]-data.ranges[i-1])<0.1):
                intensities +=[j]
            else:
                intensities+=[0]
                j=j+1

        data.intensities = tuple(intensities)

        self.pub.publish(data)



if __name__=='__main__':
    rospy.init_node("laser_node")
    laser_node = Laser()
    rospy.spin()
