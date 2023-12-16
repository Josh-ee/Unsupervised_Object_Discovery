
# program to capture single image from webcam in python 
  
# importing OpenCV library 
from cv2 import *
import time
  
# initialize the camera 
# If you have multiple camera connected with  
# current device, assign a value in cam_port  
# variable according to that 
cam_port = 1
cam = VideoCapture(cam_port, CAP_V4L2) 


for i in range(10):  
    # reading the input using the camera 
    result, image = cam.read() 
        
    # If image will detected without any error,  
    # show result 
    if result: 
      
        # showing result, it take frame name and image  
        # output 
        #imshow("calibrationCheckerboard" + str(i), image) 
      
        # saving image in local storage 
       
        imwrite("Distorted" + str(i) + ".png", image) 
        print(i, "You have 5 seconds")
        time.sleep(5)
        # If keyboard interrupt occurs, destroy image  
        # window 
        #waitKey(5000) 
        #destroyWindow("calibrationCheckerboard" + str(i)) 
      
    # If captured image is corrupted, moving to else part 
    else: 
        print("No image detected. Please! try again")
        i -= 1 
