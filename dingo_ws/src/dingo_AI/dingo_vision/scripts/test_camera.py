#!/usr/bin/env python3
# Simple CSI camera test

import cv2
import numpy as np
import sys

print("Testing CSI camera with cv2.VideoCapture...")

# Try different CSI camera formats
camera_formats = [
    "csi://0",
    "v4l2src device=/dev/video0 ! video/x-raw,format=UYVY ! videoconvert ! appsink",
    "nvarguscamerasrc sensor_id=0 ! video/x-raw(memory:NVMM),format=NV12 ! nvvidconv ! appsink",
]

cap = None

for fmt in camera_formats:
    print(f"Trying: {fmt}")
    cap = cv2.VideoCapture(fmt, cv2.CAP_GSTREAMER)
    if cap.isOpened():
        print(f"  -> SUCCESS with {fmt}")
        break

if not cap or not cap.isOpened():
    print("FAILED: No camera format worked")
    sys.exit(1)

# Read one frame
ret, frame = cap.read()
if ret and frame is not None:
    print(f"SUCCESS! Frame shape: {frame.shape}")
    cv2.imwrite("/home/bao-tg/Capstone/brain/AI/debug_raw/test_csi.jpg", frame)
    print("Saved to test_csi.jpg")
else:
    print("FAILED: Could not read frame")

cap.release()