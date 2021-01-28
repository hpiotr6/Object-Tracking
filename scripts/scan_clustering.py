#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy
from sensor_msgs.msg import LaserScan
from sensor_msgs.msg import PointCloud2 as pc2
from laser_geometry import LaserProjection
from sensor_msgs import point_cloud2
from visualization_msgs.msg import Marker
from geometry_msgs.msg import Point

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

    def get_mean_point_of_all_points_in_cluster(self):
        n = len(self.get_list_of_points())
        x_sum = 0
        y_sum = 0
        for point in self.get_list_of_points():
            x_sum += point.x
            y_sum += point.y
        return (x_sum/n , y_sum/n)
    


class Clustering():
    def __init__(self):
        self.laser_proj = LaserProjection()
        #self.pc_pub = rospy.Publisher("/laserPointCloud", pc2, queue_size=1)
        self.laser_sub = rospy.Subscriber("/scan_distance_filter", LaserScan, self.scan_callback)
        self.clusters_list = []
        self.scan = LaserScan()
        self.point_cloud = pc2()
        
    #def scan_callback(self, data):
    def scan_callback(self, data):
        self.scan = data
    
    def scan_to_pc2(self):
        # robienie z lasera chmury punktow
        self.point_cloud = self.laser_proj.projectLaser(self.scan)
        #cloud_out.header.frame_id = "laser"
        
    def clustering(self):
        assert isinstance(self.point_cloud, pc2)
        #gen = point_cloud2.read_points(self.point_cloud)
        #print(type(gen))
        #print(point_cloud2.read_points_list(self.point_cloud))
        self.list_of_point_namedtuples = point_cloud2.read_points_list(self.point_cloud)
        # print(list_of_point_namedtuples)
        # for p in gen:
        #     self.xy_coordinates_list.append((p[0], p[1])) # lista wspolrzednych chmury punktow
        #     print(p)
 
        
        distance_threshold = 0.05 # DAĆ JAKO ROSPARAM!
        clusters_list = [Cluster]
        N = len(self.list_of_point_namedtuples)
        N_horizon = 10  # DAĆ JAKO ROSPARAM!
        if N_horizon >= N:
            N_horizon = N-1

        print(' - liczba punktow: ', N)
        
        for point in self.list_of_point_namedtuples:
            n = self.list_of_point_namedtuples.index(point)
            x = point.x
            y = point.y
            index_n = point.index


        #for n in range(N):
            # print("\n\n==== NEW POINT ====")
            # print(' - liczba punktow: ', len(self.list_of_point_namedtuples),)
            # print(' - liczba clusterow: ', len(self.clusters_list))
            # for c in self.clusters_list:
            #     print(c.get_list_of_points())
            # 3 # szukanie najblizszego punktu w przod do aktualnie rozpatrywanego
            a = n+1
            if n == N-1:
                a = 0
            min_distance = distance_between_points(self.list_of_point_namedtuples[n], self.list_of_point_namedtuples[a])
            point_n = self.list_of_point_namedtuples[n]
            point_m = self.list_of_point_namedtuples[a]
            #print('min dist: ', min_distance)
            for i in range(2, N_horizon+1):
                b = n+i
                if n+i > N-1:
                    b = -(N-1-n-i)
                new_distance = distance_between_points(self.list_of_point_namedtuples[n], self.list_of_point_namedtuples[b])
                #print("new dis: ", new_distance)
                if min_distance > new_distance:
                    min_distance = new_distance
                    point_m = self.list_of_point_namedtuples[b]
                #print(point_m)
            D_dist_0 = self.find_D_max(index_n)
            #print("\n"," - D_dist_0: ", D_dist_0 , "\n - min distance: ", min_distance)
            #print("PUNKTY:\n - n:  ", point_n, "\n - m:  " ,point_m)
            if min_distance < D_dist_0: # 4 #
                self.cluster_2_points(point_n, point_m) # 5 #
                #print("\n ==> CLUSTER BY DISTANCE")

            else: # 6 # znalezienie najblizszego punktu w tyl
                min_distance = distance_between_points(self.list_of_point_namedtuples[n], self.list_of_point_namedtuples[n-1])
                point_n = self.list_of_point_namedtuples[n]
                point_j = self.list_of_point_namedtuples[n-1]
                for i in range(1, N_horizon+1):
                    if n-i<=0:
                        break
                    new_distance = distance_between_points(self.list_of_point_namedtuples[n], self.list_of_point_namedtuples[n-i])
                    #print('ELSE new distance', new_distance)
                    if min_distance > new_distance:
                        min_distance = new_distance
                        point_j = self.list_of_point_namedtuples[n-i]

                v1 = (point_n[0]-point_j[0], point_n[1]- point_j[1])
                v2 = (point_m[0]-point_n[0], point_m[1]- point_n[1])
                u = (point_m[0]- point_j[0], point_m[1], point_j[1])
                sigma = find_sigma(v1, v2)
                p_mean = find_mean_coordinates_of_three_points(point_j, point_n, point_m)
                w = p_mean
                phi = find_sigma(w, u)

                sigma_0 = math.radians(7) # ROSPARAM !!
                phi_0 = math.radians(7) # ROSPARAM !
                D_extra_max = 0.05 # ROSPARAM !
                alpha_0 = math.radians(1) # ROSPARAM !

                D_angle_0 = D_dist_0 + D_extra_max*function_to_evaluate_D_0_angle(phi, alpha_0)

                if lenght_of_vector(v1) < lenght_of_vector(v2):
                    D_v_max = lenght_of_vector(v1)
                else:
                    D_v_max = lenght_of_vector(v2)

                if sigma < sigma_0 and phi < phi_0 and D_v_max < D_angle_0: # 12 #
                    self.cluster_2_points(point_j, point_n)
                    self.cluster_2_points(point_n, point_m)
                    #print("\n ==> CLUSTER BY ANGLE")

                elif self.check_if_point_is_clustered(point_n) is False:
                    new_cluster = Cluster()
                    new_cluster.add_point(point_n)
                    self.add_cluster_to_clusters_list(new_cluster)
                    
        # #print(len(self.clusters_list))
        print(' - liczba clusterow: ' ,len(self.clusters_list))

        return self.marker_function()

    def check_if_point_is_clustered(self, point):
        answer = False
        for cluster in self.clusters_list:
            if point in cluster.get_list_of_points():
                answer = True
                break
        return answer
            
    def cluster_2_points(self, point_n, point_m):
        if len(self.clusters_list) == 0:
            new_cluster = Cluster()
            new_cluster.add_point(point_n)
            new_cluster.add_point(point_m)
            self.add_cluster_to_clusters_list(new_cluster)

        for cluster in self.clusters_list:
            if point_n in cluster.get_list_of_points():
                cluster.add_point(point_m)
                #print(' - n juz ma')
                return 0
            elif point_m in cluster.get_list_of_points():
                cluster.add_point(point_n)
                #print(' - m juz ma')
                return 0
        #print('nowy cluster')
        new_cluster = Cluster()
        #print('punkty n: ', point_n)
        #print('punkty m: ', point_m)
        new_cluster.add_point(point_n)
        new_cluster.add_point(point_m)
        self.add_cluster_to_clusters_list(new_cluster)
        

                
    def add_cluster_to_clusters_list(self, new_cluster):
        self.clusters_list.append(new_cluster)

    # adaptice breakpoint detection (3.3)
    def find_D_max(self, n_index):
        sigma_r = 0.07 # ROSPARAM!
        r_n_minus_1 = self.scan.ranges[n_index-1]
        if math.isnan(r_n_minus_1):
            return 3*sigma_r 

        
        lambda_angle = self.find_angle_of_scan_index(n_index-1)
        delta_phi_angle = self.find_angle_of_scan_index(n_index) - lambda_angle

        d_max = abs(r_n_minus_1*(math.sin(delta_phi_angle)/math.sin(lambda_angle - delta_phi_angle)))
        #return d_max
        return d_max + 3*sigma_r # sigma_r to odleglosc rozdzielczosci lidara 
        
    def find_angle_of_scan_index(self, scan_index):
        #angle_increment = 0.006135923322290182
        angle_increment = self.scan.angle_increment
        angle_min = self.scan.angle_min
        angle = angle_min + scan_index*angle_increment
        return angle

    def marker_function(self):
        points = Marker()
        points.header.frame_id = "laser"
        #myMarker.header.stamp = rospy.Time.now()
        points.type = Marker.POINTS
        points.action = Marker.ADD

        points.id = 0

        points.pose.position.x = 0
        points.pose.position.y = 0
        points.pose.position.z = 0

        points.pose.orientation.x = 0.0
        points.pose.orientation.y = 0.0
        points.pose.orientation.z = 0.0
        points.pose.orientation.w = 1.0

        points.scale.x = 0.2
        points.scale.y = 0.2
        points.scale.z = 0.2


        points.colors.a = 1
        points.colors.r = 0.9
        points.colors.g = 0
        points.colors.b = 0

        for cluster in self.clusters_list:
            if cluster == self.clusters_list[0]:            
            #if cluster == self.clusters_list[1]:   
            #if cluster == self.clusters_list[2]:
            #if cluster == self.clusters_list[3]:
            #if cluster == self.clusters_list[4]:
                for point in cluster.get_list_of_points():
                    
                    points.colors.a = 1
                    points.colors.r = 0.9
                    points.colors.g = 0
                    points.colors.b = 0

                    (x , y) = (point.x, point.y)
                    p = Point()
                    p.x = x
                    p.y = y
                    p.z = 0

                    points.points.append(p)

         
                # (x , y) = cluster.get_mean_point_of_all_points_in_cluster()
                # p = Point()
                # p.x = x
                # p.y = y
                # p.z = 0
                # points.points.append(p)

            print('pierwszy: ',cluster.get_list_of_points()[0],'ostatni: ',cluster.get_list_of_points()[-1], '\n')
               
        # cube = Marker()
        # cube.header.frame_id = "laser"
        # #myMarker.header.stamp = rospy.Time.now()
        # cube.type = Marker.CUBE_LIST
        # cube.action = Marker.ADD
        
        # # cube.pose.position.x = 0
        # # cube.pose.position.y = 0
        # # cube.pose.position.z = 0

        # cube.pose.orientation.x = 0.0
        # cube.pose.orientation.y = 0.0
        # cube.pose.orientation.z = 0.0
        # cube.pose.orientation.w = 1.0

        # #myMarker.lifetime = rospy.Duration
        # cube.scale.x = 1
        # cube.scale.y = 1
        # cube.scale.z = 1

        # cube.color.a = 1
        # cube.color.r = 0.0
        # cube.color.g = 0.9
        # cube.color.b = 0.2


        # for cluster in self.clusters_list:
        #     for point in cluster.get_list_of_points():

        #         (x , y) = (point.x, point.y)
        #         p = Point()
        #         p.x = x
        #         p.y = y
        #         p.z = 0

        #         cube.points.append(p)

        # return cube




        #myMarker.lifetime = rospy.Duration

        #points.color.g = 0.9
        #points.color.b = 0.2

        return points


def function_to_evaluate_D_0_angle(phi, alpha_0):
    if phi <= 0:
        return 1
    elif phi > 0 and phi <= alpha_0:
        y = (-phi**2 + alpha_0**2)/(alpha_0**2)
        return y
    elif phi > alpha_0:
        return 0
 
def find_sigma(v1, v2): # wzory (4.2a) i (4.2b)
    v1_lenght = lenght_of_vector(v1) # euclidean distance
    v2_lenght = lenght_of_vector(v2)
    dotproduct=0
    for v1,v2 in zip(v1,v2):
        dotproduct = dotproduct+v1*v2 # iloczyn skalarny wektorow v1 i v2
    
    sigma_prim = math.acos(dotproduct/(v1_lenght*v2_lenght))

    sigma = min(sigma_prim, math.pi - sigma_prim)
    return sigma

def find_mean_coordinates_of_three_points(p_j, p_n, p_m):
    mean_coordinates = ((p_j[0]+p_n[0]+p_m[0])/3 , (p_j[1]+p_n[1]+p_m[1])/3)
    return mean_coordinates

def lenght_of_vector(vector):
    return math.sqrt(vector[0]**2 + vector[1]**2)

def distance_between_points(a, b):
    return math.sqrt((a[0]-b[0])**2 + (a[1]-b[1])**2)





if __name__== "__main__":

    rospy.init_node("clustering_node", anonymous=True)
    print("ready")
    cluster_node = Clustering()
    rate=rospy.Rate(10) # 10Hz

    pub = rospy.Publisher('/visualization_marker', Marker, queue_size=10)

    while not rospy.is_shutdown():
        print("\n\n//////////////////// NEXT LOOP ////////////////////\n")
        cluster_node.clusters_list = []
        cluster_node.scan_to_pc2()
        myMarker = cluster_node.clustering()
        rate.sleep()
        
        pub.publish(myMarker)#wyslanie treści markera
    
    print("END")