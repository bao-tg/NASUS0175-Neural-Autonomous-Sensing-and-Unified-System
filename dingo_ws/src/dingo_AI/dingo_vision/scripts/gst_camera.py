"""
GStreamer-based CSI camera capture for Jetson Nano.
Works WITHOUT OpenCV GStreamer support - uses subprocess + GStreamer CLI.

The green image problem:
  When using cv2.VideoCapture(0) directly on Jetson Nano with IMX219,
  the raw V4L2 output is in Bayer RG10 format. OpenCV interprets this
  as BGR, resulting in a green-tinted image.
  
  The fix: Use nvarguscamerasrc (Argus daemon) which handles debayering
  and outputs proper NV12 -> BGRx -> BGR conversion.
"""

import subprocess
import numpy as np
import time
import os
import cv2


class GStreamerCSICamera:
    """
    CSI Camera capture using GStreamer subprocess pipeline.
    
    Usage:
        cam = GStreamerCSICamera(width=1280, height=720, fps=30)
        cam.open()
        ret, frame = cam.read()
        cam.release()
    """

    def __init__(self, sensor_id=0, width=1280, height=720, fps=30,
                 flip_method=0, sensor_mode=None):
        self.sensor_id = sensor_id
        self.width = width
        self.height = height
        self.fps = fps
        self.flip_method = flip_method
        self.sensor_mode = sensor_mode
        self.process = None
        self.frame_size = width * height * 3
        self._opened = False

    def _build_pipeline_args(self):
        args = [
            'gst-launch-1.0',
            'nvarguscamerasrc',
            f'sensor-id={self.sensor_id}',
        ]
        if self.sensor_mode is not None:
            args.append(f'sensor-mode={self.sensor_mode}')
        args.extend([
            '!',
            f'video/x-raw(memory:NVMM),width={self.width},height={self.height},format=NV12,framerate={self.fps}/1',
            '!', 'nvvidconv',
            '!', 'video/x-raw,format=BGRx',
            '!', 'videoconvert',
            '!', 'video/x-raw,format=BGR',
        ])
        if self.flip_method != 0:
            args.extend(['!', 'videoflip', f'method={self.flip_method}'])
        args.extend(['!', 'fdsink', 'fd=1'])
        return args

    def open(self):
        args = self._build_pipeline_args()
        print(f"[GStreamerCSICamera] Pipeline: {' '.join(args)}")

        try:
            self.process = subprocess.Popen(
                args,
                stdout=subprocess.PIPE,
                stderr=subprocess.PIPE,
                bufsize=self.frame_size * 2,
            )
            time.sleep(2.0)

            if self.process.poll() is not None:
                stderr_output = self.process.stderr.read().decode('utf-8', errors='replace')
                raise RuntimeError(
                    f"GStreamer pipeline failed to start:\n{stderr_output}"
                )

            self._opened = True
            print(f"[GStreamerCSICamera] Camera opened: {self.width}x{self.height} @ {self.fps}fps")
            return True

        except Exception as e:
            print(f"[GStreamerCSICamera] Failed to open: {e}")
            self._opened = False
            return False

    def read(self):
        if not self._opened or self.process is None:
            return False, None

        if self.process.poll() is not None:
            self._opened = False
            return False, None

        try:
            raw_data = self.process.stdout.read(self.frame_size)
            if len(raw_data) != self.frame_size:
                print(f"[GStreamerCSICamera] Incomplete frame: got {len(raw_data)}, expected {self.frame_size}")
                return False, None

            frame = np.frombuffer(raw_data, dtype=np.uint8).reshape(
                self.height, self.width, 3
            ).copy()

            return True, frame

        except Exception as e:
            print(f"[GStreamerCSICamera] Read error: {e}")
            return False, None

    def isOpened(self):
        return self._opened and self.process is not None and self.process.poll() is None

    def release(self):
        self._opened = False
        if self.process is not None:
            try:
                self.process.stdout.close()
                self.process.stderr.close()
                self.process.terminate()
                self.process.wait(timeout=5)
            except:
                try:
                    self.process.kill()
                except:
                    pass
            self.process = None
        print("[GStreamerCSICamera] Camera released")

    def get(self, prop_id):
        if prop_id == 3:
            return self.width
        elif prop_id == 4:
            return self.height
        elif prop_id == 5:
            return self.fps
        return 0

    def __del__(self):
        self.release()


class OpenCVCamera:
    """
    Camera capture using OpenCV's VideoCapture with GStreamer pipeline.
    Requires OpenCV built with GStreamer support.
    """

    def __init__(self, width=1280, height=720, fps=30, sensor_id=0, flip=0):
        self.width = width
        self.height = height
        self.fps = fps
        self.cap = None

        pipeline = (
            f"nvarguscamerasrc sensor-id={sensor_id} ! "
            f"video/x-raw(memory:NVMM), width={width}, height={height}, format=NV12, framerate={fps}/1 ! "
            f"nvvidconv ! video/x-raw, format=BGRx ! "
            f"videoconvert ! video/x-raw, format=BGR ! "
            f"appsink drop=true sync=false"
        )

        print(f"[OpenCVCamera] Opening GStreamer pipeline: {pipeline}")
        self.cap = cv2.VideoCapture(pipeline, cv2.CAP_GSTREAMER)

        if not self.cap.isOpened():
            raise RuntimeError(f"Failed to open camera with GStreamer pipeline")

    def read(self):
        if self.cap is None:
            return False, None
        return self.cap.read()

    def isOpened(self):
        return self.cap is not None and self.cap.isOpened()

    def release(self):
        if self.cap is not None:
            self.cap.release()
            self.cap = None

    def get(self, prop_id):
        if self.cap is not None:
            return self.cap.get(prop_id)
        return 0


def create_camera(width=1280, height=720, fps=30, sensor_id=0, flip=0):
    """
    Create the best available camera capture.
    
    Tries in order:
    1. OpenCV with GStreamer pipeline (fastest, needs OpenCV+GStreamer)
    2. GStreamer subprocess-based capture (works without OpenCV+GStreamer)
    """
    gst_available = False
    try:
        info = cv2.getBuildInformation()
        gst_available = "GStreamer:                   YES" in info
    except:
        pass

    if gst_available:
        try:
            cam = OpenCVCamera(width=width, height=height, fps=fps, sensor_id=sensor_id, flip=flip)
            print("[Camera] Using OpenCV+GStreamer capture")
            return cam
        except Exception as e:
            print(f"[Camera] OpenCV+GStreamer failed: {e}")
            print("[Camera] Falling back to subprocess GStreamer capture")
    
    cam = GStreamerCSICamera(sensor_id=sensor_id, width=width, height=height, fps=fps, flip_method=flip)
    if cam.open():
        print("[Camera] Using subprocess GStreamer capture")
        return cam
    
    raise RuntimeError("Failed to create camera capture. Is the CSI camera connected and Argus daemon running?")
