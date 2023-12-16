import cv2
import numpy as np

def find_cup_contour(contours):
    # Assuming the largest contour by area is the cup
    return max(contours, key=cv2.contourArea)

def estimate_distance(apparent_height_px, actual_height_cm, focal_length_px):
    # Using the formula: Distance = (Focal Length * Actual Height) / Apparent Height
    return (focal_length_px * actual_height_cm) / apparent_height_px

def is_aspect_ratio_valid(width, height, expected_ratio, tolerance=0.25):  # Increased tolerance
    # Check if the aspect ratio of the box is within the tolerance of the expected ratio
    return abs((height / width) - expected_ratio) <= tolerance

# Replace with the actual focal length of your webcam in pixels
focal_length_px = 1600 # This is a placeholder value

# Expected aspect ratio (height/width) of the cup
expected_aspect_ratio = 12 / 8

# HSV color range for the purple cup
lower_purple = np.array([157,  98, 117])
upper_purple = np.array([161, 214, 240])

lower_purple = np.array([150,  98, 117])
upper_purple = np.array([161, 214, 240])

# Start video capture
cap = cv2.VideoCapture(0)

while True:
    ret, frame = cap.read()
    if not ret:
        break

    # Convert frame to HSV
    hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)

    # Create a mask for purple color
    mask = cv2.inRange(hsv, lower_purple, upper_purple)

    # Find contours
    contours, _ = cv2.findContours(mask, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)

    if contours:
        cup_contour = find_cup_contour(contours)
        x, y, w, h = cv2.boundingRect(cup_contour)

        # Draw the bounding box
        cv2.rectangle(frame, (x, y), (x+w, y+h), (0, 255, 0), 2)

        if is_aspect_ratio_valid(w, h, expected_aspect_ratio):
            # Estimate and display distance only if aspect ratio is valid
            distance = estimate_distance(h, 12, focal_length_px)
            cv2.putText(frame, f"Distance: {distance:.2f} cm", (x, y-10), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)
        else:
            cv2.putText(frame, "Invalid Aspect Ratio", (x, y-10), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 255), 2)

    # Display the result
    cv2.imshow('Frame', frame)

    # Break the loop if 'q' is pressed
    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

# Release the capture and destroy all windows
cap.release()
cv2.destroyAllWindows()
