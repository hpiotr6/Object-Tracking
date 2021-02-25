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
        #self.laser_proj = LaserProjection()
        self.pub = rospy.Publisher("/scan_distance_filter", LaserScan, queue_size=1)
        self.scan = LaserScan()

    def scan_callback(self, data):
        self.scan = data
        # print(data.angle_min)
        # print(data.angle_max)

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
        
        #self.pub.publish(data)
        #print(self.scan.ranges)
       # self.scan_filtered = data

    def publish(self):
        self.pub.publish(self.scan)
        #print(self.scan.ranges)


        # # robienie z lasera chmury punktow
        # cloud_out = self.laser_proj.projectLaser(data)
        
        # cloud_out.header.frame_id = "laser"


# if __name__=='__main__':
#     rospy.init_node("laser_node")
#     #laser_node = Laser()

#     ## rospy.init_node("laser_to_point_cloud")
#     ## l2pc = Laser2PC()
#     #laser_node = Laser2PC()
#     laser_node = Laser_distance_filter_and_2PC()
    
#     rospy.spin()

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
