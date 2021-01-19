#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy
from sensor_msgs.msg import LaserScan
from sensor_msgs.msg import PointCloud2 as pc2
from laser_geometry import LaserProjection
from sensor_msgs import point_cloud2

import string
import itertools
import math

class Cluster():
    id_iter = itertools.count()

    def __init__(self):
        self._id = next(Cluster.id_iter)
        self._list_of_points = []
    
    def add_point(self, new_point):
        self._list_of_points.append(new_point)
    
    def get_id(self):
        return self._id
    
    def get_name(self):
        return "C" + string(self._id)
    
    def get_list_of_points(self):
        return self._list_of_points

class Clustering():
    def __init__(self):
        self.laser_proj = LaserProjection()
        #self.pc_pub = rospy.Publisher("/laserPointCloud", pc2, queue_size=1)
        self.laser_sub = rospy.Subscriber("/laserPointCloud", pc2, self.pc2_callback)
    
    def pc2_callback(self, data):
        xy_coordinates_list = []
        assert isinstance(data, pc2)
        gen = point_cloud2.read_points(data)
        print(type(gen))
        for p in gen:
            xy_coordinates_list.append((p[0], p[1]))
        print(xy_coordinates_list)

        distance_threshold = 0.05
        clusters_list = [Cluster]
        N = len(xy_coordinates_list)
        N_horizon = 5
        

        for n in range(0 , N-1):
            if N - n - N_horizon <= 0:
                N_horizon -= 1
            min_distance = math.dist(xy_coordinates_list[n], xy_coordinates_list[n+1])
            point_n = xy_coordinates_list[n]
            point_m = xy_coordinates_list[n+1]
            for i in range(2, N_horizon):
                new_distance = math.dist(xy_coordinates_list[n], xy_coordinates_list[n+i])
                if min_distance > new_distance:
                    min_distance = new_distance
                    point_m = xy_coordinates_list[n+i]
            # if min_distance <= distance_threshold:

            #     bool point_n_already_clustered = False
            #     bool point_m_already_clustered = False
            #     for j in clusters_list:
            #         if point_n in clusters_list[j].get_list_of_points():
            #             point_n_already_clustered = True
            #         if point_m in clusters_list[j].get_list_of_points():
            #             point_m_already_clustered = True
                    

                       
                       
            #             clusters_list[0].add_point(point_m)
            #         else:
            #             c = Cluster()
            #             c.add_point(point_n)
            #             c.add_point(point_m)

def cluster_2_points(point_n, point_m):
    
            





            





if __name__ == '__main__':
    rospy.init_node("clustering")
    cluster = Clustering()
    
    rospy.spin()