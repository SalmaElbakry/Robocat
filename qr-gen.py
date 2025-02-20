import cv2
import os
import numpy as np
import time

def process_qr_code(frame):
    qr_detector = cv2.QRCodeDetector()
    frame = cv2.convertScaleAbs(frame, alpha=1.5, beta=-90)  # Increase contrast
    frame = cv2.filter2D(frame, -1, np.array([[0, -1, 0], [-1, 5, -1], [0, -1, 0]]))  # Sharpen

    data, _, _ = qr_detector.detectAndDecode(frame)
    if data:
        print("QR Code detected:", data)
        return execute_instruction(data)

    print("QR code not readable, using ORB matching...")
    return match_with_orb(frame)

def match_with_orb(frame):
    orb = cv2.ORB_create()
    _, descriptors_image = orb.detectAndCompute(frame, None)

    reference_images = {
        "car_rotate_720": "d:/MSc/Bio-robotics/QRDetection/rotate.jpg",
        "car_stop_10s": "d:/MSc/Bio-robotics/QRDetection/stop.jpg",
        "car_turn_around": "d:/MSc/Bio-robotics/QRDetection/turn.jpg"
    }

    best_match, best_score = None, float("inf")
    bf = cv2.BFMatcher(cv2.NORM_HAMMING, crossCheck=True)

    for label, path in reference_images.items():
        if not os.path.exists(path):
            print(f"Error: File does not exist at {path}")
            continue

        ref_img = cv2.imread(path, cv2.IMREAD_GRAYSCALE)
        if ref_img is None:
            print(f"Error: Unable to load image at {path}")
            continue

        _, descriptors_ref = orb.detectAndCompute(ref_img, None)
        matches = bf.match(descriptors_ref, descriptors_image)
        if matches:
            avg_distance = np.mean([m.distance for m in sorted(matches, key=lambda x: x.distance)[:10]])
            if avg_distance < best_score:
                best_score, best_match = avg_distance, label

    if best_match:
        print("Best ORB match found:", best_match)
        return execute_instruction(best_match)

    print("No match found.")
    return False

def execute_instruction(data):
    print("Executing instruction:", data)
    #robot.stopcar()

    if data == "car_stop_10s":
        time.sleep(10)
    elif data == "car_turn_around":
        qr_turn_speed = 0x5FFF
        #robot.changespeed(qr_turn_speed, qr_turn_speed)
        time.sleep(0.5)
    elif data == "car_rotate_720":
        qr_turn_speed = 0x5FFF
        #robot.changespeed(qr_turn_speed, qr_turn_speed)
        time.sleep(2)
        return True

    return False

# Load the image to process
image_path = "d:/MSc/Bio-robotics/QRDetection/qr.jpg"
if not os.path.exists(image_path):
    print(f"Error: File does not exist at {image_path}")
else:
    frame = cv2.imread(image_path, cv2.IMREAD_GRAYSCALE)
    if frame is None:
        print(f"Error: Unable to load image at {image_path}")
    else:
        process_qr_code(frame)