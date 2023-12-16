import cv2
import numpy as np

def find_centroid(contour):
    """Find the centroid of a contour."""
    M = cv2.moments(contour)
    if M["m00"] != 0:
        cx = int(M["m10"] / M["m00"])
        cy = int(M["m01"] / M["m00"])
    else:
        cx, cy = 0, 0
    return (cx, cy)

# def calculate_distance_to_cup(centroid, camera_height, frame_width, frame_height):
#     """
#     Calculate the distance to the cup using ray-plane intersection.
#     Assumes the camera is parallel to the ground and the floor is the plane.
#     """
#     # Assuming a standard webcam focal length and field of view
#     focal_length = frame_height / 2  # This is an assumption, replace with actual focal length if known
#     fov_horizontal = 60  # degrees, typical for webcams

#     # Calculate the angle to the centroid from the center of the frame
#     dx = centroid[0] - frame_width / 2
#     dy = frame_height / 2 - centroid[1]

#     # Angle in radians
#     theta_x = np.radians(dx / (frame_width / 2) * (fov_horizontal / 2))
#     theta_y = np.arctan(dy / focal_length)

#     # Calculate the distance on the floor plane
#     distance_x = camera_height * np.tan(theta_x)
#     distance_y = camera_height * np.tan(theta_y)

#     # Total distance on the floor
#     total_distance = np.sqrt(distance_x**2 + distance_y**2)
#     return total_distance

def calculate_distance_to_cup(centroid, camera_height, frame_width, frame_height):
    """
    Calculate the distance to the cup using ray-plane intersection.
    Assumes the camera is parallel to the ground and the floor is the plane.
    """
    # Assuming a standard webcam focal length and field of view
    focal_length = 1050# This is an assumption, replace with actual focal length if known
    fov_horizontal = 54  # degrees, typical for webcams

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


# Initialize the webcam
cap = cv2.VideoCapture(0)

# Define purple color range
lower_purple = np.array([155, 111, 154])
upper_purple = np.array([174, 182, 223])

camera_height = 8.5  # in inches

# Get frame dimensions
ret, frame = cap.read()
frame_height, frame_width = frame.shape[:2]

while True:
    # Capture an image
    #ret, frame = cap.read()

    frame = self.bridge.imgmsg_to_cv2(rosimg, "8UC3")

    # Convert to HSV color space
    hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)

    # Create a mask for purple color
    mask = cv2.inRange(hsv, lower_purple, upper_purple)

    # Find contours
    contours, _ = cv2.findContours(mask, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)

    if contours:
        # Find the largest contour which we assume to be the cup
        cup_contour = max(contours, key=cv2.contourArea)
        centroid = find_centroid(cup_contour)

        # Calculate the distance to the cup
        distance = calculate_distance_to_cup(centroid, camera_height, frame_width, frame_height)
        distance_text = f"Distance: {distance:.2f} inches"

        # Draw the centroid and distance on the frame
        cv2.circle(frame, centroid, 5, (0, 255, 0), -1)
        cv2.putText(frame, distance_text, (50, 50), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 0), 2, cv2.LINE_AA)
    else:
        cv2.putText(frame, "No purple cup detected", (50, 50), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 0, 255), 2, cv2.LINE_AA)

    # Display the frame
    cv2.imshow('Cup Detection', frame)

    # Break the loop if 'q' is pressed
    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

# Release the camera and close all windows
cap.release()
cv2.destroyAllWindows()
