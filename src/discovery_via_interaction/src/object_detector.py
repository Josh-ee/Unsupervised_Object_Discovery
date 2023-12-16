#!/usr/bin/env python

import rospy
import ros_numpy
import cv2
import numpy as np
from sensor_msgs.msg import PointCloud2
from cv_bridge import CvBridge
import os
import tf
from geometry_msgs.msg import PointStamped, Point
from std_msgs.msg import Header
from discovery_via_interaction.srv import cupLocation, cupLocationResponse


class ObjectDetector:
    def __init__(self):
        rospy.init_node('object_detector', anonymous=True)

        self.bridge = CvBridge()
        self.xyz = None
        self.rgb = None
        self.tf_listener = tf.TransformListener()

        # Initialize the service server
        self.service = rospy.Service('find_cup_location', cupLocation, self.handle_cup_location_request)

        # Subscribe to the point cloud topic
        self.point_cloud_sub = rospy.Subscriber("/camera/point_cloud", PointCloud2, self.point_cloud_callback)

    def handle_cup_location_request(self, _):
        if self.xyz is not None and self.rgb is not None:
            location = self.process_images()
            if location:
                X_base, Y_base, Z_base = location
                return cupLocationResponse(X_base, Y_base, Z_base)
        return cupLocationResponse()  # Return empty response if no cup is found

    def point_cloud_callback(self, msg):
        try:
            self.process_pc(msg)
            if self.rgb is not None:
                self.process_images()
                
        except Exception as e:
            print("Error1:", e)

    def process_pc(self, data):
        pc = ros_numpy.numpify(data)
        pc = ros_numpy.point_cloud2.split_rgb_field(pc)
        
        self.xyz = np.zeros((1, pc.shape[0], 3))
        self.xyz[0,:,0]=pc['x']
        self.xyz[0,:,1]=pc['y']
        self.xyz[0,:,2]=pc['z']
        
        self.rgb = np.zeros((1, pc.shape[0], 3))
        self.rgb[0,:,0]=pc['r']
        self.rgb[0,:,1]=pc['g']
        self.rgb[0,:,2]=pc['b']
        self.rgb = np.uint8(self.rgb)

    def process_images(self):
        hsv = cv2.cvtColor(self.rgb, cv2.COLOR_RGB2HSV)
        lower_hsv = np.array([150, 50, 50])
        upper_hsv = np.array([170, 255, 170])

        mask = cv2.inRange(hsv, lower_hsv, upper_hsv)
        _, indices = np.nonzero(mask)

        if len(indices) <= 10:
            return None

        center_x = np.mean(self.xyz[0, indices, 0])
        center_y = np.mean(self.xyz[0, indices, 1])
        center_z = np.mean(self.xyz[0, indices, 2])

        camera_x, camera_y, camera_z = center_x, center_y, center_z
        print("Camera coords: ", camera_x, camera_y, camera_z)
        camera_link_x, camera_link_y, camera_link_z = camera_x, -camera_y, -camera_z

        try:
            self.tf_listener.waitForTransform("/base", "/camera_face", rospy.Time(), rospy.Duration(10.0))
            point_base = self.tf_listener.transformPoint("/base", PointStamped(header=Header(stamp=rospy.Time(), frame_id="/camera_face"), point=Point(camera_link_x, camera_link_y, camera_link_z)))
            X_base, Y_base, Z_base = point_base.point.x, point_base.point.y, point_base.point.z
            
            return X_base, Y_base, Z_base
        except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException) as e:
            rospy.logerr("TF Error: %s", e)
            return None

if __name__ == '__main__':
    detector = ObjectDetector()
    rospy.spin()

