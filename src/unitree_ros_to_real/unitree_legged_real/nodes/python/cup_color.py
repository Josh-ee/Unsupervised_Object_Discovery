import cv2
import numpy as np

# Initialize min and max HSV values
min_hsv = np.array([255, 255, 255])
max_hsv = np.array([0, 0, 0])

def update_hsv_ranges(hsv_value):
    global min_hsv, max_hsv
    # Update min and max HSV values
    min_hsv = np.minimum(min_hsv, hsv_value)
    max_hsv = np.maximum(max_hsv, hsv_value)

def get_hsv_value(event, x, y, flags, param):
    if event == cv2.EVENT_LBUTTONDOWN:
        # Get the HSV value at the clicked point
        hsv_value = hsv[y, x]
        print("HSV Value at (", x, ",", y, "):", hsv_value)
        update_hsv_ranges(hsv_value)

# Start capturing video from the webcam
cap = cv2.VideoCapture(0)

# Set a mouse callback function for the window
cv2.namedWindow('Webcam')
cv2.setMouseCallback('Webcam', get_hsv_value)

while True:
    # Read a frame from the webcam
    ret, frame = cap.read()

    if not ret:
        break

    # Convert the frame to HSV
    hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)

    # Show the frame
    cv2.imshow('Webcam', frame)

    # Break the loop if 'q' is pressed
    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

# Release the capture and close any open windows
cap.release()
cv2.destroyAllWindows()

# Print the min and max HSV values
print("Minimum HSV Value:", min_hsv)
print("Maximum HSV Value:", max_hsv)
