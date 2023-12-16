#!/usr/bin/env python

import rospy
import sensor_msgs.point_cloud2 as pc2
from sensor_msgs.msg import PointCloud2
import pcl
import numpy as np

class ColorDistanceFinder:
    def __init__(self):
        # Define color ranges in RGB
        self.lower_purple = np.array([150,  98, 117])
        self.upper_purple = np.array([161, 214, 240])
        
        # ROS Subscriber
        self.subscriber = rospy.Subscriber("/point_cloud2_topic", PointCloud2, self.callback)

    def callback(self, ros_point_cloud):
        # Convert PointCloud2 to PCL
        pcl_data = self.ros_to_pcl(ros_point_cloud)

        # Filter by color
        filtered_points = self.filter_by_color(pcl_data)

        # Calculate distances
        distances = self.calculate_distances(filtered_points)

        # Process distances (e.g., find the nearest point, average distance, etc.)
        # Here, you can process the distances as needed for your project
        print(distances)

    def ros_to_pcl(self, ros_cloud):
        # Convert ROS PointCloud2 to PCL Point Cloud
        points_list = []

        for data in pc2.read_points(ros_cloud, skip_nans=True, field_names=("x", "y", "z", "rgb")):
            rgb = int(data[3])
            # You might need to adjust the following unpacking based on your PointCloud2 format
            s = struct.pack('>f', rgb)
            i = struct.unpack('>l', s)[0]
            pack = ctypes.c_uint32(i).value
            r = int((pack >> 16) & 0x0000ff)
            g = int((pack >> 8) & 0x00ff00)
            b = int((pack) & 0x00ff)
            points_list.append([data[0], data[1], data[2], r, g, b])

        pcl_data = pcl.PointCloud_PointXYZRGB()
        pcl_data.from_list(points_list)
        return pcl_data

    def filter_by_color(self, pcl_data):
        # Filter points by the specified color range
        filtered_points = []
        for point in pcl_data:
            # Check if the point color is within the specified range
            if all(self.lower_purple <= np.array(point[3:6])) and all(np.array(point[3:6]) <= self.upper_purple):
                filtered_points.append(point)
        return filtered_points

    def calculate_distances(self, points):
        # Calculate distances from the sensor to the points
        distances = []
        for point in points:
            distance = np.sqrt(point[0]**2 + point[1]**2 + point[2]**2)
            distances.append(distance)
        return distances

# ROS Node initialization
rospy.init_node('color_distance_finder')
color_distance_finder = ColorDistanceFinder()
rospy.spin()
