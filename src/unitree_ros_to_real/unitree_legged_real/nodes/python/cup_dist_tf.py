import cv2
import cv_bridge
import numpy as np
import rospy
from sensor_msgs.msg import Image
from std_msgs.msg import Header
from geometry_msgs.msg import Point, PointStamped
#import tf2_ros as tf
import tf


class CupDetection:
    
    def __init__(self):
        print('class init')
        rospy.init_node('cup_detection', anonymous=True)

        self.tfListener = tf.TransformListener()
        self.tfbr = tf.TransformBroadcaster()

        rospy.sleep(0.1)

        self.bridge = cv_bridge.CvBridge()

        self.K = np.array([[125.894421836448332, 0.0, 246.15821859333772],
                    [0.0, 126.73176655050774, 188.87559531697855],
                    [0.0, 0.0, 1.0]])

        # Replace with the actual focal length of your webcam in pixels
        self.focal_length_px = 126 # This is a placeholder value

        # Expected aspect ratio (height/width) of the cup
        self.expected_aspect_ratio = 12 / 8

        # HSV color range for the purple cup
        self.lower_purple = np.array([157,  98, 117])
        self.upper_purple = np.array([161, 214, 240])

        self.lower_purple = np.array([150,  98, 117])
        self.upper_purple = np.array([161, 214, 240])

        self.sub = rospy.Subscriber('/camera1/undist_image_feed', Image, self.callback)
        self.pub_purple = rospy.Publisher('/cups/purple', Point, queue_size=1)


    def callback(self, rosimg):
        #print("Callback")
        #print(vars(rosimg))

        frame = self.bridge.imgmsg_to_cv2(rosimg, "8UC3")

        # # Convert frame to HSV
        hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)


        # Create a mask for purple color
        mask = cv2.inRange(hsv, self.lower_purple, self.upper_purple)

        # Find contours
        contours, _ = cv2.findContours(mask, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)

        if contours:
            
            cup_contour = self.find_cup_contour(contours)
            x, y, w, h = cv2.boundingRect(cup_contour)
            #print(x,y, w, h)

            # Draw the bounding box
            cv2.rectangle(frame, (x, y), (x+w, y+h), (0, 255, 0), 2)

            centroid = self.find_centroid(cup_contour)

            camera_height = 12
            frame_height, frame_width = frame.shape[:2]
            #frame_width = 480
            #frame_height = 400
        #     # Calculate the distance to the cup
        #     distance = self.calculate_distance_to_cup(centroid, camera_height, frame_width, frame_height)
            distance = self.calculate_distance_to_cup(centroid, camera_height, frame_width, frame_height)
            distance_text = "Distance: {} inches".format(distance)
            cv2.circle(frame, centroid, 5, (0, 255, 0), -1)
            cv2.putText(frame, distance_text, (50, 50), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 0), 2, cv2.LINE_AA)

            pointWorld = self.publish_purple_coords(centroid[0], centroid[1], distance)

            self.tfbr.sendTransform((pointWorld.x, pointWorld.y, pointWorld.z),
                            tf.transformations.quaternion_from_euler(0, 0, 0),
                            rospy.Time.now(),
                            "purple_cup",
                            "base")

            # if self.is_aspect_ratio_valid(w, h, self.expected_aspect_ratio):
            #     # Estimate and display distance only if aspect ratio is valid
            #     distance = self.estimate_distance(h, 12, self.focal_length_px)
            #     cv2.putText(frame, "Distance: {} cm".format(distance), (x, y-10), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)

            #     self.publish_purple_coords = (x+w//2, y+h//2, distance)
            # else:
            #     cv2.putText(frame, "Invalid Aspect Ratio", (x, y-10), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 255), 2)

        # Display the result
        cv2.imshow('Frame', frame)
        cv2.waitKey(3)

        # # Break the loop if 'q' is pressed
        # if cv2.waitKey(1) & 0xFF == ord('q'):
        #     break


    def publish_purple_coords(self, u, v, depth):

        x = (u-self.K[0][2]) * depth / self.K[0][0] # function of u and K
        y = (v-self.K[1][2]) * depth / self.K[1][1] # function of v and K
        z = depth # distance?

        purple_pose_bodyframe = Point()
        purple_pose_bodyframe.x = z
        purple_pose_bodyframe.y = x
        purple_pose_bodyframe.z = y

        try:
            camera_link = 'camera_face' #TODO
            self.tfListener.waitForTransform('base', camera_link, rospy.Time(), rospy.Duration(10.0))
            purple_pose_world = self.tfListener.transformPoint('base', PointStamped(header=Header(stamp=rospy.Time(), frame_id=camera_link), point=purple_pose_bodyframe)) # Convert using tf

            self.pub_purple.publish(purple_pose_world.point)
        
        except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException) as e:
            print("TF Error: " + e)

        return purple_pose_world.point
            




    def find_cup_contour(self, contours):
        # Assuming the largest contour by area is the cup
        return max(contours, key=cv2.contourArea)

    def estimate_distance(self, apparent_height_px, actual_height_cm, focal_length_px):
        # Using the formula: Distance = (Focal Length * Actual Height) / Apparent Height
        return (focal_length_px * actual_height_cm) / apparent_height_px

    def is_aspect_ratio_valid(self, width, height, expected_ratio, tolerance=0.4):  # Increased tolerance
        # Check if the aspect ratio of the box is within the tolerance of the expected ratio
        return abs((height / width) - expected_ratio) <= tolerance
    
    def calculate_distance_to_cup(self, centroid, camera_height, frame_width, frame_height):
        """
        Calculate the distance to the cup using ray-plane intersection.
        Assumes the camera is parallel to the ground and the floor is the plane.
        """
        # Assuming a standard webcam focal length and field of view
        focal_length =1 # This is an assumption, replace with actual focal length if known
        fov_horizontal = 160  # degrees, typical for webcams

        # Calculate the angle to the centroid from the center of the frame
        dx = centroid[0] - frame_width / 2
        dy = frame_height / 2 - centroid[1]

        # Angle in radians
        theta_x = np.radians(dx / (frame_width / 2) * (fov_horizontal / 2))

        # Adjust the calculation of theta_y to consider the inverted y-coordinate
        theta_y = np.arctan((frame_height / 2 - centroid[1]) / focal_length)

        # Calculate the distance on the floor plane
        distance_x = camera_height * np.tan(theta_x)
        distance_y = camera_height / np.tan(theta_y)

        # Total distance on the floor
        total_distance = np.sqrt(distance_x**2 + distance_y**2)
        return total_distance

    def find_centroid(self, contour):
        """Find the centroid of a contour."""
        M = cv2.moments(contour)
        if M["m00"] != 0:
            cx = int(M["m10"] / M["m00"])
            cy = int(M["m01"] / M["m00"])
        else:
            cx, cy = 0, 0
        return (cx, cy)

# Start video capture
# cap = cv2.VideoCapture(0)

# Release the capture and destroy all windows
# cap.release()
# cv2.destroyAllWindows()


if __name__ == '__main__':
    print("start")


    cup_detector = CupDetection()
    rospy.spin()