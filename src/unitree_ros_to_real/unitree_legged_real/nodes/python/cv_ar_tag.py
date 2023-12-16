import cv2 as cv
import numpy as np

# Instantiate ArucoDetector object
dictionary = cv.aruco.getPredefinedDictionary(cv.aruco.DICT_4X4_1000)
parameters = cv.aruco.DetectorParameters()

parameters.adaptiveThreshWinSizeMin = 3
parameters.adaptiveThreshWinSizeMax = 23
parameters.adaptiveThreshWinSizeStep = 10
parameters.adaptiveThreshConstant = 7
parameters.minMarkerPerimeterRate = 0.03
parameters.maxMarkerPerimeterRate = 4.0

detector = cv.aruco.ArucoDetector(dictionary, parameters)

def track(matrix_coefficients, distortion_coefficients):
    cap = cv.VideoCapture(0)  # Assuming you're using a webcam
    # cap = cv.VideoCapture(0)
    desired_frame_rate = 15  # Set this to your desired frame rate
    cap.set(cv.CAP_PROP_FPS, desired_frame_rate)

    while True:
        ret, frame = cap.read()
        if not ret:
            break

        # Operations on the frame come here
        gray = cv.cvtColor(frame, cv.COLOR_BGR2GRAY)  # Change to grayscale

        # Detect markers
        markerCorners, markerIds, rejectedCandidates = detector.detectMarkers(gray)
        if markerIds is not None:  # If there are markers found by detector
            for i in range(len(markerIds)):  # Iterate through markers
                # Estimate pose of each marker
                rvec, tvec, _ = cv.aruco.estimatePoseSingleMarkers(markerCorners[i], 0.02, 
                                                                   matrix_coefficients, distortion_coefficients)
                cv.aruco.drawDetectedMarkers(frame, markerCorners)  # Draw a square around the markers
                cv.drawFrameAxes(frame, matrix_coefficients, distortion_coefficients, rvec, tvec, 0.01)  # Draw Axis

        # Display the resulting frame
        cv.imshow('frame', frame)

        # Wait 3 milliseconds for an interaction. Check the key and do the corresponding job.
        key = cv.waitKey(3) & 0xFF
        if key == ord('q'):  # Quit
            break

    # When everything is done, release the capture
    cap.release()
    cv.destroyAllWindows()


    # When everything is done, release the capture
    cap.release()
    cv.destroyAllWindows()

# Assuming you have the matrix_coefficients and distortion_coefficients
# You can call the function like this:
# track(your_matrix_coefficients, your_distortion_coefficients)



focal_length_mm = 800  # in mm
image_width_in_pixels = 1920
image_height_in_pixels = 1080

# Approximate conversion (assuming sensor width ~ image width)
fx = fy = (focal_length_mm / image_width_in_pixels) * image_width_in_pixels

# Principal point (center of the image)
cx = image_width_in_pixels / 2
cy = image_height_in_pixels / 2

# Camera matrix
cameraMatrix = np.array([[fx, 0, cx],
                        [0, fy, cy],
                        [0,  0,  1]], dtype=float)

distortion_coefficients = np.zeros((4,1))  # Assuming no lens distortion

track(cameraMatrix, distortion_coefficients)