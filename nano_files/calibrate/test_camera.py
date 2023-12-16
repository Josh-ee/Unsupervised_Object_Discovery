from cv2 import *

cam_port = 1
cam = VideoCapture(cam_port)
result, image = cam.read()

if result:
    print("sucess")
    imwrite("test.png", image)
else:
    print("error")
