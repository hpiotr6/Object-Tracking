#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy
from sensor_msgs.msg import LaserScan
from sensor_msgs.msg import PointCloud2 as pc2
from laser_geometry import LaserProjection
from sensor_msgs import point_cloud2
from visualization_msgs.msg import Marker, MarkerArray
from geometry_msgs.msg import Point
from std_msgs.msg import ColorRGBA

import string
import math


class Cluster():

    def __init__(self):
        self._list_of_points = []
    
    def add_point(self, new_point):
        if not new_point in self.get_list_of_points():
            self._list_of_points.append(new_point)

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
    
    def check_if_point_is_in_this_cluster(self, p):
        answer = False
        for point in self.get_list_of_points():
            if point == p:
                answer = True
        return answer

class Clustering():

    def __init__(self):
        self.laser_proj = LaserProjection()
        #self.pc_pub = rospy.Publisher("/laserPointCloud", pc2, queue_size=1)
        self.laser_sub = rospy.Subscriber("/scan_distance_filter", LaserScan, self.scan_callback)
        self.clusters_list = []
        self.scan = LaserScan()
        self.point_cloud = pc2()
        self.parameters = ({"N_horizon": rospy.get_param("N_horizon"), 
                            "sigma_r": rospy.get_param("sigma_r"), 
                            "sigma_0": rospy.get_param("sigma_0"), 
                            "phi_0":rospy.get_param("phi_0"), 
                            "D_extra_max": rospy.get_param("D_extra_max"), 
                            "alpha_0": rospy.get_param("alpha_0"),
                            "distance_threshold": rospy.get_param("distance_threshold")})

    def scan_callback(self, data):
        self.scan = data
    
    def scan_to_pc2(self):
        # robienie z lasera chmury punktow
        self.point_cloud = self.laser_proj.projectLaser(self.scan)
        
    def clustering(self):
        assert isinstance(self.point_cloud, pc2)
        self.list_of_point_namedtuples = point_cloud2.read_points_list(self.point_cloud)
        # lista wspolrzednych chmury punktow

        clusters_list = [Cluster]
        N = len(self.list_of_point_namedtuples)
        self.N_horizon = self.parameters["N_horizon"]
        if self.N_horizon >= N:
            self.N_horizon = N-1

        print(' - liczba punktow: ', N)
        
        for point in self.list_of_point_namedtuples:
            n = self.list_of_point_namedtuples.index(point)
            x = point.x
            y = point.y
            index_n = point.index

            if n == 0:
                first_cluster = Cluster()
                first_cluster.add_point(point)
                self.add_cluster_to_clusters_list(first_cluster)

            '''
            # 3 # szukanie najblizszego punktu w przod do aktualnie rozpatrywanego
            '''    
            a = n+1
            if n == N-1:
                break

            min_distance = distance_between_points(self.list_of_point_namedtuples[n], self.list_of_point_namedtuples[a])
            point_n = self.list_of_point_namedtuples[n]
            point_o = self.list_of_point_namedtuples[a]
            for i in range(2, self.N_horizon+1):
                b = n+i
                if n+i > N-1:
                    break
                new_distance = distance_between_points(self.list_of_point_namedtuples[n], self.list_of_point_namedtuples[b])
                if min_distance > new_distance:
                    min_distance = new_distance
                    point_o = self.list_of_point_namedtuples[b]
            
            self.D_dist_0 = self.find_D_max(index_n)

            if min_distance < self.D_dist_0: # 4 #
                self.cluster_2_points(point_n, point_o) # 5 #

            min_distance = distance_between_points(self.list_of_point_namedtuples[n], self.list_of_point_namedtuples[n-1])
            sigma_0 = self.parameters["sigma_0"]
            phi_0 = self.parameters["phi_0"]
            D_extra_max = self.parameters["D_extra_max"]
            alpha_0 = self.parameters["alpha_0"]
            # 6 # znalezienie najblizszego punktu w tyl
            point_j = self.list_of_point_namedtuples[n-1]
            for i in range(1, self.N_horizon+1):
                if n-i<=0:
                    break
                new_distance = distance_between_points(self.list_of_point_namedtuples[n], self.list_of_point_namedtuples[n-i])

                if min_distance > new_distance:
                    min_distance = new_distance
                    point_j = self.list_of_point_namedtuples[n-i]
            '''
            sprawdzenie czy n-1, n, n+1 są w linni
            ''' 
            v1 = (point_n[0]-point_j[0], point_n[1]- point_j[1])
            v2 = (point_o[0]-point_n[0], point_o[1]- point_n[1])
            u = (point_o[0]- point_j[0], point_o[1]- point_j[1])
            sigma = find_sigma(v1, v2)
            p_mean = find_mean_coordinates_of_three_points(point_j, point_n, point_o)
            w = p_mean
            phi = find_sigma(w, u)
            D_angle_0 = self.D_dist_0 + D_extra_max*function_to_evaluate_D_0_angle(phi, alpha_0)
            if lenght_of_vector(v1) > lenght_of_vector(v2):
                D_v_max = lenght_of_vector(v1)
            else:
                D_v_max = lenght_of_vector(v2)

            if (sigma < sigma_0 and phi < phi_0 and D_v_max < D_angle_0): # 12 #
                self.cluster_2_points(point_j, point_n)
                self.cluster_2_points(point_n, point_o)
                continue

            if n-2>=0 and n+2<N:
                point_p = self.list_of_point_namedtuples[n+2]
                point_o = self.list_of_point_namedtuples[n+1]
                point_j = self.list_of_point_namedtuples[n-1]
                point_i = self.list_of_point_namedtuples[n-2]
                if (abs(point_o.index - point_n.index)==1 and abs(point_p.index - point_o.index)==1 and 
                    abs(point_n.index - point_j.index)==1 and abs(point_j.index - point_i.index)==1):
                    prev_p = []
                    prev_p.append(point_j)
                    prev_p.append(point_i)

                    next_p = []
                    next_p.append(point_o)
                    next_p.append(point_p)

                    mean_prev = find_mean_coordinates_of_points(prev_p)
                    mean_next = find_mean_coordinates_of_points(next_p)

                    sigma, D_angle_0, D_v_max = self.function_to_evaluate_sigma_Dangle0_Dvmax(mean_prev, point_n, mean_next)

                    if (sigma < sigma_0 and D_v_max < D_angle_0): # 12 #
                        self.cluster_2_points(point_j, point_n)
                        self.cluster_2_points(point_n, point_o)
                        continue

                    if n+3<N:
                        point_q = self.list_of_point_namedtuples[n+3]
                        next_p.append(point_q)
                        mean_next = find_mean_coordinates_of_points(next_p)

                        sigma, D_angle_0, D_v_max = self.function_to_evaluate_sigma_Dangle0_Dvmax(point_n, point_o, mean_next)
                        if (sigma < sigma_0 and D_v_max < D_angle_0): # 12 #
                            self.cluster_2_points(point_n, point_o)
                            continue
                    
                    if n-3>=0:
                        point_h = self.list_of_point_namedtuples[n-3]
                        prev_p.append(point_h)
                        mean_prev = find_mean_coordinates_of_points(prev_p)

                        sigma, D_angle_0, D_v_max = self.function_to_evaluate_sigma_Dangle0_Dvmax(mean_prev, point_j, point_n)
                        
                        if (sigma < sigma_0 and D_v_max < D_angle_0): # 12 #
                            self.cluster_2_points(point_j, point_n)
                            continue
                    
                    if n-3>=0 and n+3<N:
                        point_q = self.list_of_point_namedtuples[n+3]
                        point_h = self.list_of_point_namedtuples[n-3]
                        if (abs(point_i.index - point_h.index)==1 and abs(point_q.index - point_p.index)==1):
                            prev_p.append(point_h)
                            next_p.append(point_q)
                            mean_prev = find_mean_coordinates_of_points(prev_p)
                            mean_next = find_mean_coordinates_of_points(next_p)
                            sigma, D_angle_0, D_v_max = self.function_to_evaluate_sigma_Dangle0_Dvmax(mean_prev, point_n, mean_next)

                            if (sigma < sigma_0 and D_v_max < D_angle_0): # 12 #
                                self.cluster_2_points(point_j, point_n)
                                self.cluster_2_points(point_n, point_o)
                                continue
                            
                            sigma, D_angle_0, D_v_max = self.function_to_evaluate_sigma_Dangle0_Dvmax(mean_prev, point_j, mean_next)
                            if (sigma < sigma_0 and D_v_max < D_angle_0): # 12 #
                                self.cluster_2_points(point_j, point_n)
                                self.cluster_2_points(point_n, point_o)
                                continue

            if self.check_if_point_is_clustered(point_n) is False:
                new_cluster = Cluster()
                new_cluster.add_point(point_n)
                self.add_cluster_to_clusters_list(new_cluster)

        for cluster in self.clusters_list:
            if len(cluster.get_list_of_points())<3:
                self.clusters_list.remove(cluster)
        
        return self.clusters_list

    def merge_clusters_by_mean_point(self, list_of_previous_clusters):
        if list_of_previous_clusters == None or len(self.clusters_list)==0 or len(list_of_previous_clusters)==0:
            return self.clusters_list
        else:
            if len(self.clusters_list) == len(list_of_previous_clusters):
                return self.clusters_list
            elif len(self.clusters_list) > len(list_of_previous_clusters):
                print("robi się")

                i = 1
                while i < len(self.clusters_list):
                    for j in range(len(list_of_previous_clusters)):
                        previous_fisrt_point = list_of_previous_clusters[j].get_list_of_points()[0]
                        current_first_point = self.clusters_list[i-1].get_list_of_points()[0]
                        translation_vector = (current_first_point[0]-previous_fisrt_point[0], current_first_point[1]-previous_fisrt_point[1])
                        #print('translation_vector =', translation_vector)
                        previous_cluster = list_of_previous_clusters[j]
                        current_cluster_1 = self.clusters_list[i-1]
                        current_cluster_2 = self.clusters_list[i]
                        current_mean_x = (current_cluster_1.get_mean_point_of_all_points_in_cluster()[0] + current_cluster_2.get_mean_point_of_all_points_in_cluster()[0])/2
                        current_mean_y = (current_cluster_1.get_mean_point_of_all_points_in_cluster()[1] + current_cluster_2.get_mean_point_of_all_points_in_cluster()[1])/2
                        current_mean = (current_mean_x, current_mean_y)
                        previous_mean = previous_cluster.get_mean_point_of_all_points_in_cluster()
                        previous_mean = (previous_mean[0]+translation_vector[0], previous_mean[1]+translation_vector[1]) # uwzględnienie przesunięcia
                        distance_threshold = self.parameters["distance_threshold"]#0.1
                        if distance_between_points(previous_mean, current_mean) < distance_threshold:
                            print(distance_between_points(previous_mean, current_mean))
                            self.merge_clusters(current_cluster_1, current_cluster_2)
                            print('MERGE dwoch')
                        else:
                            i+=1
                    
    def check_if_point_is_clustered(self, point):
        answer = False
        for cluster in self.clusters_list:
            if point in cluster.get_list_of_points():
                answer = True
                break
        return answer
 
    def find_cluster_for_point(self, p):
        for cluster in self.clusters_list:
            if p in cluster.get_list_of_points():
                return cluster
   
    def check_if_points_are_in_one_cluster(self, points):
        answer = True
        cluster = self.find_cluster_for_point(points[0])
        for point in points:
            if not cluster.check_if_point_is_in_this_cluster(point):
                answer = False
        return answer
 
    def cluster_2_points(self, point_n, point_o):
        if abs(point_n.index-point_o.index)>self.N_horizon:
            return 0

        n_check = self.check_if_point_is_clustered(point_n)
        m_check = self.check_if_point_is_clustered(point_o)

        if n_check == False and m_check==False:
            new_cluster = Cluster()
            new_cluster.add_point(point_n)
            new_cluster.add_point(point_o)
            self.add_cluster_to_clusters_list(new_cluster)
            return 0

        elif (n_check == True and m_check==False) or (n_check == False and m_check==True):
            for cluster in self.clusters_list:
                if point_n in cluster.get_list_of_points():
                    cluster.add_point(point_o)
                    return 0

                elif point_o in cluster.get_list_of_points():
                    cluster.add_point(point_n)
                    return 0

        elif n_check == True and m_check==True:
            c1 = self.find_cluster_for_point(point_o)
            c2 = self.find_cluster_for_point(point_n)
            if c1 == c2:
                return 0
            for point in c2.get_list_of_points():
                c1.add_point(point)
            self.clusters_list.remove(c2)
            return 0
        
        new_cluster = Cluster()

        new_cluster.add_point(point_n)
        new_cluster.add_point(point_o)
        self.add_cluster_to_clusters_list(new_cluster)
    
    def merge_clusters(self, c1, c2):
        if c1 == c2:
                return 0
        for point in c2.get_list_of_points():
            c1.add_point(point)
        self.clusters_list.remove(c2)
        return 1
                
    def add_cluster_to_clusters_list(self, new_cluster):
        self.clusters_list.append(new_cluster)

    def find_D_max(self, n_index):
        # adaptive breakpoint detection (3.3)
        sigma_r = self.parameters["sigma_r"]
        r_n_minus_1 = self.scan.ranges[n_index-1]
        if math.isnan(r_n_minus_1):
            return 3*sigma_r 

        lambda_angle = self.find_angle_of_scan_index(n_index-1)
        delta_phi_angle = self.find_angle_of_scan_index(n_index) - lambda_angle
        d_max = abs(r_n_minus_1*(math.sin(delta_phi_angle)/math.sin(lambda_angle - delta_phi_angle)))

        if d_max > 0.25:
            d_max = 0.25
        
        return d_max + 3*sigma_r # sigma_r to odleglosc rozdzielczosci lidara 
        
    def find_angle_of_scan_index(self, scan_index):
        angle_increment = self.scan.angle_increment
        angle_min = self.scan.angle_min
        angle = angle_min + scan_index*angle_increment
        return angle

    def function_to_evaluate_sigma_Dangle0_Dvmax(self, point1, point2, point3):
        D_extra_max = self.parameters["D_extra_max"]
        alpha_0 = self.parameters["alpha_0"]
        v1 = (point1[0]-point2[0], point1[1]- point2[1])
        v2 = (point2[0]-point3[0], point2[1]- point3[1])
        sigma = find_sigma(v1, v2)
        u = (point3[0]- point1[0], point3[1]- point1[1])
        p_mean = find_mean_coordinates_of_three_points(point1, point2, point3)
        w = p_mean
        phi = find_sigma(w, u)
        D_angle_0 = self.D_dist_0 + D_extra_max*function_to_evaluate_D_0_angle(phi, alpha_0)
        if lenght_of_vector(v1) > lenght_of_vector(v2):
            D_v_max = lenght_of_vector(v1)
        else:
            D_v_max = lenght_of_vector(v2)
        return sigma, D_angle_0, D_v_max

    def marker_function(self):
        
        marker_array = MarkerArray()
        marker_array.markers.clear()
        for cluster in self.clusters_list:

            myMarker = Marker()
            myMarker.header.frame_id = "laser"
            myMarker.header.stamp = rospy.Time.now()
            ns = "cluster" + str(self.clusters_list.index(cluster))
            myMarker.ns = ns
            myMarker.type = Marker.POINTS
            myMarker.action = Marker.ADD
            myMarker.pose.orientation.w = 1.0
            myMarker.id = 0
            myMarker.scale.x = 0.005
            myMarker.scale.y = 0.005
            myMarker.color.g = 0.0
            myMarker.color.a = 1.0
            myMarker.color.r= 1.0
            myMarker.color.b = 0
            myMarker.lifetime = rospy.Duration(0.1)
            print(' - liczba clusterow: ' ,len(self.clusters_list))
        
            for point in cluster.get_list_of_points():
                (x , y) = (point.x, point.y)
                p = Point()
                p.x = x
                p.y = y
                p.z = 0
                myMarker.points.append(p)
                myMarker.colors.append(myMarker.color)

            print('pierwszy: ',cluster.get_list_of_points()[0],'ostatni: ',cluster.get_list_of_points()[-1] )
            print('liczba punktow w clusterze: ' , len(cluster.get_list_of_points()),'\n')
            marker_array.markers.append(myMarker)

        return marker_array


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
    if v1_lenght == 0 or v2_lenght == 0:
        print(v1, v2)
    sigma_prim = math.acos(dotproduct/(v1_lenght*v2_lenght))

    sigma = min(sigma_prim, math.pi - sigma_prim)
    return sigma

def find_mean_coordinates_of_three_points(p_j, p_n, p_m):
    mean_coordinates = ((p_j[0]+p_n[0]+p_m[0])/3 , (p_j[1]+p_n[1]+p_m[1])/3)
    return mean_coordinates

def find_mean_coordinates_of_points(points):
    n = len(points)
    mean_coordinates = [0,0]
    for point in points:
        mean_coordinates[0] += point[0]/n
        mean_coordinates[1] += point[1]/n
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

    pub = rospy.Publisher('/visualization_marker', MarkerArray, queue_size=10)

    list_of_previous_clusters = []
    while not rospy.is_shutdown():
        print("\n\n//////////////////// NEXT LOOP ////////////////////\n")
        
        cluster_node.clusters_list = []
        cluster_node.scan_to_pc2()
        cluster_node.clustering()
        cluster_node.merge_clusters_by_mean_point(list_of_previous_clusters)
        list_of_markers = cluster_node.marker_function()
        pub.publish(list_of_markers)#wyslanie treści markera
        list_of_previous_clusters = cluster_node.clusters_list
        rate.sleep()
    print("END")