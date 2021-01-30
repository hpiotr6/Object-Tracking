#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy
from sensor_msgs.msg import LaserScan
from sensor_msgs.msg import PointCloud2 as pc2
from laser_geometry import LaserProjection
from sensor_msgs import point_cloud2
from visualization_msgs.msg import Marker
from geometry_msgs.msg import Point
from std_msgs.msg import ColorRGBA

import string
import itertools
import math



class Cluster():
    id_iter = itertools.count()

    def __init__(self):
        self._id = next(Cluster.id_iter)
        self._list_of_points = []
    
    def add_point(self, new_point):
        if not new_point in self.get_list_of_points():
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
        self.list_of_point_namedtuples = point_cloud2.read_points_list(self.point_cloud)
        # lista wspolrzednych chmury punktow

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

            if n == 0:
                first_cluster = Cluster()
                first_cluster.add_point(point)
                self.add_cluster_to_clusters_list(first_cluster)

            # print("\n\n==== NEW POINT ====")
            # print(' - liczba punktow: ', len(self.list_of_point_namedtuples),)
            # print(' - liczba clusterow: ', len(self.clusters_list))

            # 3 # szukanie najblizszego punktu w przod do aktualnie rozpatrywanego
            a = n+1
            if n == N-1:
                a = 0
            min_distance = distance_between_points(self.list_of_point_namedtuples[n], self.list_of_point_namedtuples[a])
            point_n = self.list_of_point_namedtuples[n]
            point_m = self.list_of_point_namedtuples[a]
            for i in range(2, N_horizon+1):
                b = n+i
                if n+i > N-1:
                    b = n+i - N
                new_distance = distance_between_points(self.list_of_point_namedtuples[n], self.list_of_point_namedtuples[b])
                if min_distance > new_distance:
                    min_distance = new_distance
                    point_m = self.list_of_point_namedtuples[b]
            
            D_dist_0 = self.find_D_max(index_n)
            #print(point.index ,D_dist_0)
            #print("\n"," - D_dist_0: ", D_dist_0 , "\n - min distance: ", min_distance)
            #print("PUNKTY:\n - n:  ", point_n.index, "\n - m:  " ,point_m.index)
            if min_distance < D_dist_0: # 4 #
                self.cluster_2_points(point_n, point_m) # 5 #
                #print("\n ==> CLUSTER BY DISTANCE")
                #print(point_n.index, point_m.index)

            else: 
                
                min_distance = distance_between_points(self.list_of_point_namedtuples[n], self.list_of_point_namedtuples[n-1])
                sigma_0 = math.radians(10) # ROSPARAM !!
                phi_0 = math.radians(10) # ROSPARAM !
                D_extra_max = 0.1 # ROSPARAM !
                alpha_0 = math.radians(1) # ROSPARAM !
                # 6 # znalezienie najblizszego punktu w tyl
                point_j = self.list_of_point_namedtuples[n-1]
                for i in range(1, N_horizon+1):
                    if n-i<=0:
                        break
                    new_distance = distance_between_points(self.list_of_point_namedtuples[n], self.list_of_point_namedtuples[n-i])
                    #print('ELSE new distance', new_distance)
                    if min_distance > new_distance:
                        min_distance = new_distance
                        point_j = self.list_of_point_namedtuples[n-i]

                if n+1 > N-1 or (self.list_of_point_namedtuples[n].index - self.list_of_point_namedtuples[n-1].index<N_horizon and self.list_of_point_namedtuples[n+1].index - self.list_of_point_namedtuples[n].index>N_horizon+1):
                    print('IF 1')
                    point_n = self.list_of_point_namedtuples[n]
                    point_j = self.list_of_point_namedtuples[n-1]
                    point_i = self.list_of_point_namedtuples[n-2]
                    v1 = (point_i[0]-point_j[0], point_i[1]- point_j[1])
                    v2 = (point_j[0]-point_n[0], point_j[1]- point_n[1])
                    sigma = find_sigma(v1, v2)
                    u = (point_n[0]- point_i[0], point_n[1], point_i[1])
                    p_mean = find_mean_coordinates_of_three_points(point_i, point_j, point_n)
                    w = p_mean
                    phi = find_sigma(w, u)
                    D_angle_0 = D_dist_0 + D_extra_max*function_to_evaluate_D_0_angle(phi, alpha_0)
                    if lenght_of_vector(v1) < lenght_of_vector(v2):
                        D_v_max = lenght_of_vector(v1)
                    else:
                        D_v_max = lenght_of_vector(v2)
                    if D_v_max < D_angle_0 and sigma > 1.4486232792: # 83 stopni
                        self.cluster_2_points(point_n, point_j)
                
                elif n-1 < 0 or (self.list_of_point_namedtuples[n].index - self.list_of_point_namedtuples[n-1].index> N_horizon+1 and self.list_of_point_namedtuples[n+1].index - self.list_of_point_namedtuples[n].index<N_horizon):
                    print('IF 2')
                    point_n = self.list_of_point_namedtuples[n]
                    point_k = self.list_of_point_namedtuples[n+1]
                    point_l = self.list_of_point_namedtuples[n+2]
                    
                    v1 = (point_n[0]-point_k[0], point_n[1]- point_k[1])
                    v2 = (point_k[0]-point_l[0], point_k[1]- point_l[1])
                    sigma = find_sigma(v1, v2)
                    u = (point_n[0]- point_l[0], point_n[1], point_l[1])
                    p_mean = find_mean_coordinates_of_three_points(point_l, point_n, point_k)
                    w = p_mean
                    phi = find_sigma(w, u)
                    D_angle_0 = D_dist_0 + D_extra_max*function_to_evaluate_D_0_angle(phi, alpha_0)
                    if lenght_of_vector(v1) < lenght_of_vector(v2):
                        D_v_max = lenght_of_vector(v1)
                    else:
                        D_v_max = lenght_of_vector(v2)
                    if D_v_max < D_angle_0 and sigma > 1.4486232792: # 83 stopni
                        self.cluster_2_points(point_n, point_k)
                    
                    print('n: ', point_n.index)
                    print('k: ',point_k.index)
                    print(sigma)
                    print(D_angle_0)
                # else:
                    # print('ELSE')


                v1 = (point_n[0]-point_j[0], point_n[1]- point_j[1])
                v2 = (point_m[0]-point_n[0], point_m[1]- point_n[1])
                u = (point_m[0]- point_j[0], point_m[1], point_j[1])
                sigma = find_sigma(v1, v2)
                p_mean = find_mean_coordinates_of_three_points(point_j, point_n, point_m)
                w = p_mean
                phi = find_sigma(w, u)

                

                D_angle_0 = D_dist_0 + D_extra_max*function_to_evaluate_D_0_angle(phi, alpha_0)

                if lenght_of_vector(v1) < lenght_of_vector(v2):
                    D_v_max = lenght_of_vector(v1)
                else:
                    D_v_max = lenght_of_vector(v2)
                print(point_j.index , point_n.index, point_m.index)
                print(sigma < sigma_0, phi < phi_0, D_v_max < D_angle_0)
                print('sigma < sigma_0: ', sigma ,'<', sigma_0, 'phi < phi_0: ', phi ,'<', phi_0, 'D_v_max < D_angle_0: ', D_v_max ,'<', D_angle_0, )
                print(sigma > 1.5184364492, D_v_max < D_angle_0)
                if (sigma < sigma_0 and phi < phi_0 and D_v_max < D_angle_0): # 12 #
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
    
    def find_cluster_for_point(self, p):
        for cluster in self.clusters_list:
            if p in cluster.get_list_of_points():
                return cluster

            
    def cluster_2_points(self, point_n, point_m):
        n_check = self.check_if_point_is_clustered(point_n)
        m_check = self.check_if_point_is_clustered(point_m)
        #print(n_check == False, m_check == False)
        if n_check == False and m_check==False:
            
        # if len(self.clusters_list) == 0:
            new_cluster = Cluster()
            new_cluster.add_point(point_n)
            new_cluster.add_point(point_m)
            self.add_cluster_to_clusters_list(new_cluster)
            return 0

        elif (n_check == True and m_check==False) or (n_check == False and m_check==True):
            for cluster in self.clusters_list:
                if point_n in cluster.get_list_of_points():
                    cluster.add_point(point_m)

                    #print(' - n juz ma')
                    #print(len(self.clusters_list))
                    return 0
                elif point_m in cluster.get_list_of_points():
                    cluster.add_point(point_n)
                    #print(' - m juz ma')
                    return 0

        elif n_check == True and m_check==True:
            c1 = self.find_cluster_for_point(point_m)
            c2 = self.find_cluster_for_point(point_n)
            if c1 == c2:
                return 0
            for point in c2.get_list_of_points():
                c1.add_point(point)
            self.clusters_list.remove(c2)
            return 0
        # else:
        #     for cluster in self.clusters_list:
        #         if point_n in cluster.get_list_of_points():
        #             cluster.add_point(point_m)
        #             #print(' - n juz ma')
        #             return 0
        #         elif point_m in cluster.get_list_of_points():
        #             cluster.add_point(point_n)
        #             #print(' - m juz ma')
        #             return 0
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
        sigma_r = 0.02 # ROSPARAM!
        r_n_minus_1 = self.scan.ranges[n_index-1]
        if math.isnan(r_n_minus_1):
            return 3*sigma_r 

        
        lambda_angle = self.find_angle_of_scan_index(n_index-1)
        delta_phi_angle = self.find_angle_of_scan_index(n_index) - lambda_angle


        d_max = abs(r_n_minus_1*(math.sin(delta_phi_angle)/math.sin(lambda_angle - delta_phi_angle)))
        
        #return d_max
        if d_max > 0.25:
            d_max = 0.25
            #print("r_n_minus_1: ", r_n_minus_1 , "licznik: ", math.sin(delta_phi_angle), 'mianownik: ',math.sin(lambda_angle - delta_phi_angle), 'lambda - deltaphi', lambda_angle - delta_phi_angle)
        return d_max + 3*sigma_r # sigma_r to odleglosc rozdzielczosci lidara 
        
    def find_angle_of_scan_index(self, scan_index):
        #angle_increment = 0.006135923322290182
        angle_increment = self.scan.angle_increment
        angle_min = self.scan.angle_min
        angle = angle_min + scan_index*angle_increment
        return angle

    def marker_function(self):
        myMarker = Marker()
        myMarker.header.frame_id = "laser"
        #myMarker.header.stamp = rospy.Time.now()
        myMarker.type = Marker.POINTS
        myMarker.action = Marker.ADD

        myMarker.id = 0

        myMarker.pose.position.x = 0
        myMarker.pose.position.y = 0
        myMarker.pose.position.z = 0

        myMarker.pose.orientation.x = 0.0
        myMarker.pose.orientation.y = 0.0
        myMarker.pose.orientation.z = 0.0
        myMarker.pose.orientation.w = 1.0

        # myMarker.scale.x = 0.2
        # myMarker.scale.y = 0.2
        # myMarker.scale.z = 0.2


        # myMarker.colors[0] = 1
        # myMarker.colors[1] = 0.9
        # myMarker.colors[2]= 0
        # myMarker.colors[3] = 0
        


        for cluster in self.clusters_list:
            #print(cluster._id())
            #print(cluster.get_list_of_points())
            #if len(self.clusters_list)>1:
                #if cluster == self.clusters_list[1]:            
                #if cluster == self.clusters_list[1]:   
                #if cluster == self.clusters_list[2]:
                #if cluster == self.clusters_list[3]:
                #if cluster == self.clusters_list[4]:
            
                
                # points.colors.a = 1
                # points.colors.r = 0.9
                # points.colors.g = 0
                # points.colors.b = 0
            for point in cluster.get_list_of_points():

                (x , y) = (point.x, point.y)
                p = Point()
                p.x = x
                p.y = y
                p.z = 0
                #myMarker.color = [1, 0.9, 0, 0]
                myMarker.points.append(p)



                # (x , y) = cluster.get_mean_point_of_all_points_in_cluster()
                # p = Point()
                # p.x = x
                # p.y = y
                # p.z = 0
                # points.points.append(p)
            #print(cluster.get_list_of_points())
            print('pierwszy: ',cluster.get_list_of_points()[0],'ostatni: ',cluster.get_list_of_points()[-1] )
            print('liczba punktow w clusterze: ' , len(cluster.get_list_of_points()),'\n')
    



        #myMarker.lifetime = rospy.Duration

        #points.color.g = 0.9
        #points.color.b = 0.2

        return myMarker


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