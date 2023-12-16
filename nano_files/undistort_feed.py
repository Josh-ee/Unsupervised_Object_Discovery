import cv2
assert cv2.__version__[0] >= '3', 'The fisheye module requires opencv version >= 3.0.0'
import numpy as np
import os
import glob
import rospy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge

DIM = (480, 400)

K = np.array([[125.3635717115587, 0.0, 247.85179292490665],
            [0.0, 124.90414364619001, 191.88842916650333],
            [0.0, 0.0, 1.0]])

D = np.array([[0.11063715466802057],
            [-0.06033490436539556],
            [0.019655604870762667],
            [-0.002362081471026174]])

newK = cv2.fisheye.estimateNewCameraMatrixForUndistortRectify(K, D, DIM, np.eye(3), balance=1)


class TestRos(object):

    def __init__(self):
        self.image = None
        self.br = CvBridge()
        self.loop_rate = rospy.Rate(10)
        self.pub = rospy.Publisher('/camera1/front_undistorted', Image, queue_size=1)

        self.cam_port = 1
        #print(cv2.getBuildInformation())
        self.cam = cv2.VideoCapture(self.cam_port, cv2.CAP_V4L2)

    def init_capture(self):
        while not rospy.is_shutdown():
            result, image = self.cam.read()
            #print(image.shape)
            if result:
                self.image = image
                # print(self.image.shape)
                h,w = self.image.shape[:2]
                map1, map2 = cv2.fisheye.initUndistortRectifyMap(K, D, np.eye(3), newK, DIM, cv2.CV_16SC2)
                self.undistorted_img = cv2.remap(self.image, map1, map2, interpolation=cv2.INTER_LINEAR, borderMode=cv2.BORDER_CONSTANT)
                
                #self.undistorted_img = cv2.undistort(self.image, K, D)

                #print("new image received")
                self.pub.publish(self.br.cv2_to_imgmsg(self.undistorted_img))
                #print("new image published.")
            else:
                print("warning: camera image not published properly.")
            self.loop_rate.sleep()
        self.cam.release()



if __name__ == '__main__':
    rospy.init_node("test_camera", anonymous=True)
    cam_node = TestRos()
    cam_node.init_capture()