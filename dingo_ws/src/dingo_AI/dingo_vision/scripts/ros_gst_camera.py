#!/usr/bin/env python3
import os
import subprocess
import time

import numpy as np
import rospy
from sensor_msgs.msg import CameraInfo, Image


class GstFdCamera:
    def __init__(self, sensor_id, width, height, fps, flip_method):
        self.sensor_id = sensor_id
        self.width = width
        self.height = height
        self.fps = fps
        self.flip_method = flip_method
        self.frame_size = width * height * 3
        self.process = None

    def pipeline_args(self):
        args = [
            "gst-launch-1.0",
            "nvarguscamerasrc",
            f"sensor-id={self.sensor_id}",
            "!",
            f"video/x-raw(memory:NVMM),width={self.width},height={self.height},format=NV12,framerate={self.fps}/1",
            "!",
            "nvvidconv",
            f"flip-method={self.flip_method}",
            "!",
            "video/x-raw,format=BGRx",
            "!",
            "videoconvert",
            "!",
            "video/x-raw,format=BGR",
            "!",
            "fdsink",
            "fd=1",
        ]
        return args

    def open(self):
        args = self.pipeline_args()
        rospy.loginfo("Starting CSI camera pipeline: %s", " ".join(args))
        self.process = subprocess.Popen(
            args,
            stdout=subprocess.PIPE,
            stderr=subprocess.PIPE,
            bufsize=self.frame_size * 2,
        )
        time.sleep(2.0)
        if self.process.poll() is not None:
            stderr = self.process.stderr.read().decode("utf-8", errors="replace")
            raise RuntimeError(f"GStreamer camera pipeline exited early:\n{stderr}")

    def read(self):
        if self.process is None or self.process.poll() is not None:
            return None
        data = self.process.stdout.read(self.frame_size)
        if len(data) != self.frame_size:
            rospy.logwarn_throttle(2.0, "Incomplete camera frame: got %d bytes, expected %d", len(data), self.frame_size)
            return None
        return np.frombuffer(data, dtype=np.uint8).reshape((self.height, self.width, 3))

    def close(self):
        if self.process is None:
            return
        try:
            self.process.terminate()
            self.process.wait(timeout=3)
        except Exception:
            self.process.kill()
        self.process = None


def make_image_msg(frame, frame_id):
    msg = Image()
    msg.header.stamp = rospy.Time.now()
    msg.header.frame_id = frame_id
    msg.height, msg.width = frame.shape[:2]
    msg.encoding = "bgr8"
    msg.is_bigendian = False
    msg.step = msg.width * 3
    msg.data = frame.tobytes()
    return msg


def main():
    rospy.init_node("dingo_csi_camera")

    sensor_id = rospy.get_param("~sensor_id", 0)
    width = rospy.get_param("~width", 1280)
    height = rospy.get_param("~height", 720)
    fps = rospy.get_param("~fps", 30)
    flip_method = rospy.get_param("~flip_method", 0)
    frame_id = rospy.get_param("~frame_id", "csi_camera")
    image_topic = rospy.get_param("~image_topic", "/camera/image_raw")
    info_topic = rospy.get_param("~camera_info_topic", "/camera/camera_info")

    pub = rospy.Publisher(image_topic, Image, queue_size=1)
    info_pub = rospy.Publisher(info_topic, CameraInfo, queue_size=1)

    camera = GstFdCamera(sensor_id, width, height, fps, flip_method)
    rospy.on_shutdown(camera.close)
    camera.open()

    info = CameraInfo()
    info.width = width
    info.height = height
    info.header.frame_id = frame_id

    while not rospy.is_shutdown():
        frame = camera.read()
        if frame is None:
            if camera.process is not None and camera.process.poll() is not None:
                raise RuntimeError("GStreamer camera process stopped")
            continue
        image_msg = make_image_msg(frame, frame_id)
        info.header.stamp = image_msg.header.stamp
        pub.publish(image_msg)
        info_pub.publish(info)


if __name__ == "__main__":
    main()
