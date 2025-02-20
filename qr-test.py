from simple_pid import PID
import cv2
from picamera2 import Picamera2
import time
import RPi.GPIO as GPIO
from classes import Robot
import numpy as np
import os

# Initialize the robot
robot = Robot()

# Initialize the camera
picam2 = Picamera2()
picam2.start()

# Video settings
frame_rate = 30
video_duration = 45
frame_width, frame_height = 640, 480

# Robot parameters
turn_speed = 0x5FFF
straight_speed = 0x5FFF
max_speed = 0x5FFF

# PID Controller for direction
pid_direction = PID(Kp=264, Ki=528, Kd=33, setpoint=frame_width // 2)
pid_direction.output_limits = (-max_speed // 2, max_speed // 2) 

# Load reference QR codes
reference_qr_paths = ["ref_1.png", "ref_2.png", "ref_0.png"]
reference_qrs = [(path, cv2.imread(path, cv2.IMREAD_GRAYSCALE)) for path in reference_qr_paths]

# Helper function: preprocess the image
def preprocess_image(frame):
    frame = cv2.resize(frame, (480, 480), interpolation=cv2.INTER_AREA)
    height, width, _ = frame.shape
    roi = frame[int(height * 10 / 11):, :]  # Focus on lower third of the frame

    gray = cv2.cvtColor(roi, cv2.COLOR_RGB2GRAY)
    gray_blurred = cv2.GaussianBlur(gray, (7, 7), 0)

    # Thresholding to detect the black line
    _, binary = cv2.threshold(gray_blurred, 60, 255, cv2.THRESH_BINARY_INV)
    contours = cv2.findContours(binary, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    contours = contours[0] if len(contours) == 2 else contours[1]
    largest = max(contours, key=cv2.contourArea) if contours else None
    if largest is not None:
        M = cv2.moments(largest)
        if M["m00"] != 0:
            cx = int(M["m10"] / M["m00"])
            return cx
    return None

# Function to detect QR code in the frame
def detect_qr_code(frame):
    qr_detector = cv2.QRCodeDetector()
    data, bbox, _ = qr_detector.detectAndDecode(frame)
    return data if data else None

# Function to match QR code with pre-existing QR codes
def match_qr_code(seen_qr):
    akaze = cv2.AKAZE_create()  # Faster alternative to SIFT
    kp1, des1 = akaze.detectAndCompute(seen_qr, None)

    best_match = None
    best_score = 0

    for path, ref_qr in reference_qrs:
        kp2, des2 = akaze.detectAndCompute(ref_qr, None)

        if des1 is None or des2 is None:
            continue

        bf = cv2.BFMatcher()
        matches = bf.knnMatch(des1, des2, k=2)

        good_matches = [m for m, n in matches if m.distance < 0.75 * n.distance]

        if len(good_matches) > best_score:
            best_score = len(good_matches)
            best_match = path

    return os.path.basename(best_match) if best_match else None

# Main loop
prev_speed_left = 0
prev_speed_right = 0

start_time = time.time()
try:
    while time.time() - start_time < video_duration:
        # Capture frame from the camera
        frame = picam2.capture_array()
        
        qr_data = detect_qr_code(frame)
        # Check for QR codes first
        if qr_data:
            print(f"QR Code Detected: {qr_data}")
            
            # STOP the robot
            robot.stopcar()
            print("Robot Stopped for QR Processing")
            
            # Convert frame to grayscale and match QR code
            qr_match = match_qr_code(cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY))
            if qr_match:
                print(f"Best Matched QR Code: {qr_match}")


        # Preprocess the frame to find the line's centroid
        cx = preprocess_image(frame)

        if cx is not None:
            # Calculate the PID output for direction control
            direction_speed = pid_direction(cx)
            print(f"Direction speed: {direction_speed}")

            # Calculate motor speeds
            turn_scale = 1.0
            if cx < 80 or cx > 220:
                turn_scale = 1.2

            left_speed = int(max(0, min(max_speed, straight_speed - direction_speed * turn_scale)))
            right_speed = int(max(0, min(max_speed, straight_speed + direction_speed * turn_scale)))
            prev_speed_left = left_speed
            prev_speed_right = right_speed

            print(f"Left Speed: {left_speed}, Right Speed: {right_speed}")

            # Send commands to the robot
            robot.changespeed(left_speed, right_speed)
            robot.forward()
        else:
            print("No line detected, stopping.")
            robot.stopcar()
            robot.changespeed(turn_speed, turn_speed)
            if prev_speed_left - prev_speed_right > -25000:
                robot.turnRight()
            elif prev_speed_left - prev_speed_right < 25000:
                robot.turnLeft()
            else:
                robot.stopcar()

        # Break loop on 'q' key press
        if cv2.waitKey(1) & 0xFF == ord('q'):
            break
finally:
    # Cleanup resources
    picam2.stop()
    cv2.destroyAllWindows()
    robot.stopcar()
    print("Driving done!")
