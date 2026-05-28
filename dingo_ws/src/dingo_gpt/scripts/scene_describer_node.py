#!/usr/bin/env python3

import os
import sys
import tempfile

sys.path.insert(0, os.path.dirname(os.path.realpath(__file__)))

import rospy
from std_msgs.msg import String

from dingo_openai_utils import chat_completion, image_to_data_url


class SceneDescriberNode(object):
    def __init__(self):
        self.model = rospy.get_param("~model", "gpt-4o-mini")
        self.capture_width = int(rospy.get_param("~capture_width", 1280))
        self.capture_height = int(rospy.get_param("~capture_height", 720))
        self.display_width = int(rospy.get_param("~display_width", 640))
        self.display_height = int(rospy.get_param("~display_height", 360))
        self.framerate = int(rospy.get_param("~framerate", 30))
        self.flip_method = int(rospy.get_param("~flip_method", 0))
        self.camera_source = rospy.get_param("~camera_source", "csi").lower()
        self.camera_device = rospy.get_param("~camera_device", "/dev/video0")
        self.pub_tts = rospy.Publisher("/tts_input", String, queue_size=10)
        self.pub_description = rospy.Publisher("/scene_description", String, queue_size=10)
        rospy.Subscriber("/scene_describe_request", String, self.describe_callback, queue_size=1)

    def describe_callback(self, msg):
        rospy.loginfo("Scene: capture requested")
        image_path = None
        try:
            image_path = self.capture_frame()
            description = self.describe_image(image_path)
            rospy.loginfo("Scene: %s", description)
            self.pub_description.publish(description)
            self.pub_tts.publish(description)
        except Exception as exc:
            rospy.logerr("Scene: failed: %s", exc)
            self.pub_tts.publish("I could not capture or describe the camera image.")
        finally:
            if image_path:
                try:
                    os.remove(image_path)
                except OSError:
                    pass

    def capture_frame(self):
        if self.camera_source == "csi":
            return self.capture_csi_frame()
        if self.camera_source == "v4l2":
            return self.capture_v4l2_frame()
        raise RuntimeError("unsupported camera_source %r; use csi or v4l2" % self.camera_source)

    def capture_csi_frame(self):
        import cv2

        pipeline = (
            "nvarguscamerasrc ! "
            "video/x-raw(memory:NVMM), width=(int)%d, height=(int)%d, framerate=(fraction)%d/1 ! "
            "nvvidconv flip-method=%d ! "
            "video/x-raw, width=(int)%d, height=(int)%d, format=(string)BGRx ! "
            "videoconvert ! video/x-raw, format=(string)BGR ! appsink"
            % (
                self.capture_width,
                self.capture_height,
                self.framerate,
                self.flip_method,
                self.display_width,
                self.display_height,
            )
        )
        cap = cv2.VideoCapture(pipeline, cv2.CAP_GSTREAMER)
        return self.capture_from_video_capture(
            cv2,
            cap,
            "could not open CSI camera. The container needs Jetson Argus GStreamer support, including nvarguscamerasrc. For a USB camera, launch with camera_source:=v4l2 camera_device:=/dev/video0",
            "could not read frame from CSI camera",
        )

    def capture_v4l2_frame(self):
        import cv2

        cap = cv2.VideoCapture(self.camera_device, cv2.CAP_V4L2)
        if cap.isOpened():
            cap.set(cv2.CAP_PROP_FRAME_WIDTH, self.display_width)
            cap.set(cv2.CAP_PROP_FRAME_HEIGHT, self.display_height)
            cap.set(cv2.CAP_PROP_FPS, self.framerate)
        return self.capture_from_video_capture(
            cv2,
            cap,
            "could not open V4L2 camera %s" % self.camera_device,
            "could not read frame from V4L2 camera %s" % self.camera_device,
        )

    def capture_from_video_capture(self, cv2, cap, open_error, read_error):
        if not cap.isOpened():
            raise RuntimeError(open_error)
        try:
            frame = None
            for _ in range(8):
                ok, frame = cap.read()
                if not ok:
                    frame = None
            if frame is None:
                raise RuntimeError(read_error)
            fd, path = tempfile.mkstemp(prefix="dingo_scene_", suffix=".jpg")
            os.close(fd)
            if not cv2.imwrite(path, frame):
                raise RuntimeError("could not write captured image")
            return path
        finally:
            cap.release()

    def describe_image(self, image_path):
        data_url = image_to_data_url(image_path)
        messages = [
            {
                "role": "system",
                "content": "You describe what a quadruped robot camera sees. Be concise and practical.",
            },
            {
                "role": "user",
                "content": [
                    {"type": "text", "text": "Describe the scene in one or two sentences."},
                    {"type": "image_url", "image_url": {"url": data_url}},
                ],
            },
        ]
        return chat_completion(messages, model=self.model, max_tokens=120, temperature=0.2)


if __name__ == "__main__":
    rospy.init_node("scene_describer_node")
    SceneDescriberNode()
    rospy.loginfo("Scene: listening for requests on /scene_describe_request")
    rospy.spin()
