import rospy

from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import os
import numpy as np

class TestRos(object):
    def __init__(self):
        self.image = None
        self.br = CvBridge()
        self.loop_rate = rospy.Rate(10)
        self.pub = rospy.Publisher('/camera/front', Image, queue_size=1)

        self.cam_port = 1
        #print(cv2.getBuildInformation())
        self.cam = cv2.VideoCapture(self.cam_port, cv2.CAP_V4L2)
        
    def init_capture(self):
        while not rospy.is_shutdown():
            result, image = self.cam.read()
            print(image.shape)
            if result:
                image = cv2.resize(image, (640, 480))
                self.image = image[20:380, 40:440]
                
                print("new image received")
                self.pub.publish(self.br.cv2_to_imgmsg(self.image))
                print("new image published.")
            else:
                print("warning: camera image not published properly.")
            self.loop_rate.sleep()
        self.cam.release()

if __name__ == '__main__':
    print(1)
    rospy.init_node("test_camera", anonymous=True)
    print(2)
    cam_node = TestRos()
    print(3)
    cam_node.init_capture()
    print(4)

# cam_port = 1
# cam = cv2.VideoCapture(cam_port)
# result, image = cam.read()

# if result:
#    print("sucess")
#    imwrite("test.png", image)
# else:
#    print("error")
