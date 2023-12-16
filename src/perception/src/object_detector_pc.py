#!/usr/bin/env python

import rospy
import ros_numpy
import cv2
import numpy as np
from sensor_msgs.msg import PointCloud2
from cv_bridge import CvBridge
import os
import tf
from geometry_msgs.msg import Point, PointStamped
from std_msgs.msg import Header


class ObjectDetector:

    def __init__(self):
        print("Running...")
        rospy.init_node('object_detector', anonymous=True)

        self.bridge = CvBridge()

        self.xyz = None
        self.rgb = None

        self.point_cloud_sub = rospy.Subscriber("/camera1/point_cloud", PointCloud2, self.point_cloud_callback)

        self.tf_listener = tf.TransformListener()  # Create a TransformListener object
        self.tfbr = tf.TransformBroadcaster()

        self.point_pub = rospy.Publisher("/cups/purple", Point, queue_size=10)

        rospy.spin()


    def point_cloud_callback(self, msg):
        try:
            # print('received')
            self.process_pc(msg)

            if self.rgb is not None:
                self.process_images()

        except Exception as e:
            print("Error1:", e)


    def process_pc(self, data):
        '''
        Extracts XYZ and RGB arrays from the point cloud data.
        '''
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
        '''
        Extracts cup coordinates from XYZ and RGB data extracted from the point cloud.
        '''
        lower_hsv = np.array([150, 50, 50])
        upper_hsv = np.array([170, 255, 170])

        hsv = cv2.cvtColor(self.rgb, cv2.COLOR_RGB2HSV)
        mask = cv2.inRange(hsv, lower_hsv, upper_hsv)

        masked = cv2.cvtColor(cv2.bitwise_and(hsv, hsv, mask=mask), cv2.COLOR_RGB2HSV)
        # masked = np.vstack((masked, masked, masked, masked, masked, masked, masked))
        # cv2.imshow('Res', np.vstack((masked, masked, masked, masked, masked, masked, masked)))
        # cv2.waitKey(3)

        _, indices = np.nonzero(mask)

        # If there are no detected cups, exit
        if len(indices) <= 10:
            print("No self.rgb detected. Is your color filter wrong?")
            return

        # Calculate the center of the detected region by 
        center_x = np.mean(self.xyz[0, indices, 0])
        center_y = np.mean(self.xyz[0, indices, 1])
        center_z = np.mean(self.xyz[0, indices, 2])

        camera_x, camera_y, camera_z = center_x, center_y, center_z
        # print("Camera coords: ", camera_x, camera_y, camera_z)
        camera_link_x, camera_link_y, camera_link_z = camera_x, -camera_y, -camera_z

        # Convert the (X, Y, Z) coordinates from camera frame to odom frame
        try:
            self.tf_listener.waitForTransform("/base", "/camera_face", rospy.Time(), rospy.Duration(10.0))
            point_base = self.tf_listener.transformPoint("/base", PointStamped(header=Header(stamp=rospy.Time(), frame_id="/camera_face"), point=Point(camera_link_x, camera_link_y, camera_link_z)))
            X_base, Y_base, Z_base = point_base.point.x, point_base.point.y, point_base.point.z
            #print("Real-world coordinates in odom frame: (X, Y, Z) = ({:.2f}m, {:.2f}m, {:.2f}m)".format(X_odom, Y_odom, Z_odom))

            self.point_pub.publish(Point(X_base, Y_base, Z_base))

            self.tfbr.sendTransform((X_base, Y_base, Z_base),
                        tf.transformations.quaternion_from_euler(0, 0, 0),
                        rospy.Time.now(),
                        "purple_cup",
                        "base")

        except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException) as e:
            print("TF Error: " + e)
            return


    def rgb_to_hsv(self, rgb_threshold):
        '''
        Convert the RGB numpy array to an HSV numpy array.
        '''
        hsv_threshold = cv2.cvtColor(np.uint8([[rgb_threshold]]), cv2.COLOR_RGB2HSV)[0][0]
        return hsv_threshold






if __name__ == '__main__':
    print('Starting Point Cloud cup detection ')
    ObjectDetector()