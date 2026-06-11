#!/usr/bin/env python3
import signal
import sys
import time
from typing import Optional, Tuple

import cv2
import numpy as np
import rospy
from sensor_msgs.msg import Image
from std_msgs.msg import Float32MultiArray, String


def image_msg_to_numpy(msg: Image) -> np.ndarray:
    channels_by_encoding = {
        "rgb8": 3,
        "bgr8": 3,
        "mono8": 1,
    }
    if msg.encoding not in channels_by_encoding:
        raise ValueError(f"Unsupported image encoding: {msg.encoding}")

    channels = channels_by_encoding[msg.encoding]
    dtype = np.uint8
    height = msg.height
    width = msg.width
    expected_step = width * channels * np.dtype(dtype).itemsize

    frame = np.frombuffer(msg.data, dtype=dtype)
    frame = frame.reshape((height, msg.step))[:, :expected_step]
    if channels == 1:
        frame = frame.reshape((height, width))
        frame = cv2.cvtColor(frame, cv2.COLOR_GRAY2BGR)
    else:
        frame = frame.reshape((height, width, channels))
        if msg.encoding == "rgb8":
            frame = cv2.cvtColor(frame, cv2.COLOR_RGB2BGR)
    return frame.copy()


def numpy_to_image_msg(frame: np.ndarray, header) -> Image:
    msg = Image()
    msg.header = header
    msg.height = frame.shape[0]
    msg.width = frame.shape[1]
    msg.encoding = "bgr8"
    msg.is_bigendian = 0
    msg.step = frame.shape[1] * 3
    msg.data = frame.tobytes()
    return msg


def create_tracker(name: str):
    tracker_type = name.upper()
    if tracker_type == "MIL":
        return cv2.TrackerMIL_create()
    if tracker_type == "KCF":
        return cv2.TrackerKCF_create()
    if tracker_type == "CSRT":
        return cv2.legacy.TrackerCSRT_create()
    if tracker_type == "MOSSE":
        return cv2.legacy.TrackerMOSSE_create()
    raise ValueError(f"Unsupported tracker '{name}'. Use MIL, KCF, CSRT, or MOSSE.")


class RosImageTracker:
    def __init__(self):
        self.tracker_type = rospy.get_param("~tracker_type", "MIL")
        self.image_topic = rospy.get_param("~image_topic", "/camera/image_raw")
        self.show_window = rospy.get_param("~show_window", False)
        self.interactive_roi = rospy.get_param("~interactive_roi", False)
        self.selection_mode = rospy.get_param("~selection_mode", "bbox").lower()
        if self.selection_mode == "point":
            rospy.logwarn("selection_mode=point is deprecated; use point_bbox for bbox tracking from a clicked point.")
            self.selection_mode = "point_bbox"
        self.point_roi_size = max(2, int(rospy.get_param("~point_roi_size", 40)))
        self.lk_win_size = max(3, int(rospy.get_param("~lk_win_size", 21)))
        self.lk_max_level = max(0, int(rospy.get_param("~lk_max_level", 3)))
        self.lk_min_eig_threshold = float(rospy.get_param("~lk_min_eig_threshold", 1e-4))
        self.publish_debug_image = rospy.get_param("~publish_debug_image", True)
        self.roi_param = rospy.get_param("~roi", [])
        self.max_width = int(rospy.get_param("~max_width", 640))

        self.tracker = None
        self.prev_gray = None
        self.prev_point = None
        self.scale = 1.0
        self.frame_count = 0
        self.last_log_time = time.time()
        self.window_name = "Dingo ROS Image Tracker"

        self.bbox_pub = rospy.Publisher("~bbox", Float32MultiArray, queue_size=1)
        self.point_pub = rospy.Publisher("~point", Float32MultiArray, queue_size=1)
        self.status_pub = rospy.Publisher("~status", String, queue_size=1)
        self.debug_pub = rospy.Publisher("~debug_image", Image, queue_size=1) if self.publish_debug_image else None
        self.sub = rospy.Subscriber(self.image_topic, Image, self.image_callback, queue_size=1, buff_size=2**24)

        rospy.loginfo(
            "Dingo tracker subscribed to %s with selection_mode=%s tracker=%s",
            self.image_topic,
            self.selection_mode,
            self.tracker_type,
        )

    def point_to_roi(self, frame: np.ndarray, point: Tuple[int, int]) -> Tuple[int, int, int, int]:
        h, w = frame.shape[:2]
        roi_size = min(self.point_roi_size, w, h)
        half = roi_size // 2
        x = max(0, min(int(point[0]) - half, w - roi_size))
        y = max(0, min(int(point[1]) - half, h - roi_size))
        return (x, y, roi_size, roi_size)

    def select_point_roi(self, frame: np.ndarray) -> Optional[Tuple[int, int, int, int]]:
        selected = []
        preview = frame.copy()

        def on_mouse(event, x, y, _flags, _param):
            if event == cv2.EVENT_LBUTTONDOWN:
                selected[:] = [(x, y)]

        cv2.namedWindow(self.window_name, cv2.WINDOW_NORMAL)
        cv2.setMouseCallback(self.window_name, on_mouse)
        rospy.loginfo("Click one target point, then press ENTER or SPACE. Press c to cancel.")

        while not rospy.is_shutdown():
            display = preview.copy()
            if selected:
                x, y, rw, rh = self.point_to_roi(frame, selected[0])
                cv2.circle(display, selected[0], 4, (0, 255, 255), -1)
                cv2.rectangle(display, (x, y), (x + rw, y + rh), (0, 255, 255), 2)
            cv2.imshow(self.window_name, display)
            key = cv2.waitKey(20) & 0xFF
            if key in (13, 32) and selected:
                cv2.destroyWindow(self.window_name)
                return self.point_to_roi(frame, selected[0])
            if key in (ord("c"), 27):
                cv2.destroyWindow(self.window_name)
                return None

        cv2.destroyWindow(self.window_name)
        return None

    def select_point(self, frame: np.ndarray) -> Optional[Tuple[int, int]]:
        selected = []
        preview = frame.copy()

        def on_mouse(event, x, y, _flags, _param):
            if event == cv2.EVENT_LBUTTONDOWN:
                selected[:] = [(x, y)]

        cv2.namedWindow(self.window_name, cv2.WINDOW_NORMAL)
        cv2.setMouseCallback(self.window_name, on_mouse)
        rospy.loginfo("Click one optical-flow point, then press ENTER or SPACE. Press c to cancel.")

        while not rospy.is_shutdown():
            display = preview.copy()
            if selected:
                cv2.circle(display, selected[0], 6, (0, 255, 255), -1)
                cv2.circle(display, selected[0], 14, (0, 255, 255), 2)
            cv2.imshow(self.window_name, display)
            key = cv2.waitKey(20) & 0xFF
            if key in (13, 32) and selected:
                cv2.destroyWindow(self.window_name)
                return selected[0]
            if key in (ord("c"), 27):
                cv2.destroyWindow(self.window_name)
                return None

        cv2.destroyWindow(self.window_name)
        return None

    def initial_roi(self, frame: np.ndarray) -> Optional[Tuple[int, int, int, int]]:
        if isinstance(self.roi_param, list) and len(self.roi_param) == 4:
            return tuple(int(v) for v in self.roi_param)

        if self.interactive_roi:
            if self.selection_mode == "point_bbox":
                return self.select_point_roi(frame)
            if self.selection_mode != "bbox":
                rospy.logwarn("Unknown selection_mode %s for bbox tracker; falling back to bbox ROI selection", self.selection_mode)
            cv2.namedWindow(self.window_name, cv2.WINDOW_NORMAL)
            rospy.loginfo("Select ROI in the OpenCV window, then press ENTER or SPACE. Press c to cancel.")
            roi = cv2.selectROI(self.window_name, frame, fromCenter=False, showCrosshair=True)
            cv2.destroyWindow(self.window_name)
            roi = tuple(int(v) for v in roi)
            if roi[2] <= 0 or roi[3] <= 0:
                return None
            return roi

        h, w = frame.shape[:2]
        return (w // 4, h // 4, w // 2, h // 2)

    def initial_point(self, frame: np.ndarray) -> Optional[Tuple[int, int]]:
        if isinstance(self.roi_param, list):
            if len(self.roi_param) == 2:
                return tuple(int(v) for v in self.roi_param)
            if len(self.roi_param) == 4:
                x, y, w, h = [int(v) for v in self.roi_param]
                return (x + w // 2, y + h // 2)

        if self.interactive_roi:
            return self.select_point(frame)

        h, w = frame.shape[:2]
        return (w // 2, h // 2)

    def maybe_resize(self, frame: np.ndarray) -> np.ndarray:
        h, w = frame.shape[:2]
        if self.max_width <= 0 or w <= self.max_width:
            self.scale = 1.0
            return frame
        self.scale = self.max_width / float(w)
        return cv2.resize(frame, (self.max_width, int(h * self.scale)))

    def initialize_optical_flow(self, frame: np.ndarray) -> bool:
        point = self.initial_point(frame)
        if point is None:
            rospy.logwarn("No optical-flow point selected; waiting for another frame.")
            return False

        h, w = frame.shape[:2]
        x = max(0, min(int(point[0]), w - 1))
        y = max(0, min(int(point[1]), h - 1))
        self.prev_gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        self.prev_point = np.array([[[float(x), float(y)]]], dtype=np.float32)
        rospy.loginfo("Initialized optical-flow tracker at point (%d, %d)", x, y)
        return True

    def publish_optical_flow_result(self, success: bool, point: Optional[Tuple[float, float]], display: np.ndarray, header):
        point_msg = Float32MultiArray()
        bbox_msg = Float32MultiArray()

        if success and point is not None:
            x, y = point
            roi_size = float(self.point_roi_size)
            half = roi_size / 2.0
            point_msg.data = [1.0, x / self.scale, y / self.scale]
            bbox_msg.data = [1.0, (x - half) / self.scale, (y - half) / self.scale, roi_size / self.scale, roi_size / self.scale]
            cv2.circle(display, (int(x), int(y)), 5, (0, 255, 255), -1)
            cv2.circle(display, (int(x), int(y)), 14, (0, 255, 255), 2)
            cv2.putText(display, "LK optical flow", (int(x) + 10, max(20, int(y) - 10)), cv2.FONT_HERSHEY_SIMPLEX, 0.55, (0, 255, 255), 2)
        else:
            point_msg.data = [0.0, 0.0, 0.0]
            bbox_msg.data = [0.0, 0.0, 0.0, 0.0, 0.0]
            cv2.putText(display, "POINT LOST", (20, 40), cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 0, 255), 2)

        self.point_pub.publish(point_msg)
        self.bbox_pub.publish(bbox_msg)
        if self.debug_pub is not None:
            self.debug_pub.publish(numpy_to_image_msg(display, header))

    def process_optical_flow(self, frame: np.ndarray, msg: Image):
        if self.prev_gray is None or self.prev_point is None:
            if self.initialize_optical_flow(frame):
                display = frame.copy()
                x, y = self.prev_point.reshape(2)
                self.publish_optical_flow_result(True, (float(x), float(y)), display, msg.header)
            return

        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        next_point, status, _err = cv2.calcOpticalFlowPyrLK(
            self.prev_gray,
            gray,
            self.prev_point,
            None,
            winSize=(self.lk_win_size, self.lk_win_size),
            maxLevel=self.lk_max_level,
            criteria=(cv2.TERM_CRITERIA_EPS | cv2.TERM_CRITERIA_COUNT, 20, 0.03),
            minEigThreshold=self.lk_min_eig_threshold,
        )

        self.frame_count += 1
        display = frame.copy()
        success = bool(next_point is not None and status is not None and status[0][0] == 1)
        tracked_point = None
        if success:
            x, y = next_point.reshape(2)
            h, w = frame.shape[:2]
            success = 0 <= x < w and 0 <= y < h
            if success:
                tracked_point = (float(x), float(y))
                self.prev_gray = gray
                self.prev_point = next_point

        self.publish_optical_flow_result(success, tracked_point, display, msg.header)
        self.status_pub.publish(String(data=f"frames={self.frame_count} algorithm=optical_flow success={success}"))

        if self.show_window:
            cv2.imshow(self.window_name, display)
            cv2.waitKey(1)

        now = time.time()
        if now - self.last_log_time >= 5.0:
            rospy.loginfo("Optical-flow tracker processed %d frames", self.frame_count)
            self.last_log_time = now

    def image_callback(self, msg: Image):
        try:
            frame = self.maybe_resize(image_msg_to_numpy(msg))
        except Exception as exc:
            rospy.logwarn_throttle(5.0, "Failed to convert image: %s", exc)
            return

        if self.selection_mode in ("optical_flow", "lk"):
            self.process_optical_flow(frame, msg)
            return

        if self.tracker is None:
            roi = self.initial_roi(frame)
            if roi is None:
                rospy.logwarn("No ROI selected; waiting for another frame.")
                return
            try:
                self.tracker = create_tracker(self.tracker_type)
                self.tracker.init(frame, roi)
            except Exception as exc:
                rospy.logerr("Failed to initialize %s tracker: %s", self.tracker_type, exc)
                rospy.signal_shutdown("tracker initialization failed")
                return
            rospy.loginfo("Initialized %s tracker with ROI %s", self.tracker_type, roi)

        self.frame_count += 1
        success, bbox = self.tracker.update(frame)
        display = frame.copy()

        bbox_msg = Float32MultiArray()
        if success:
            x, y, w, h = [float(v) for v in bbox]
            bbox_msg.data = [1.0, x / self.scale, y / self.scale, w / self.scale, h / self.scale]
            cv2.rectangle(display, (int(x), int(y)), (int(x + w), int(y + h)), (0, 255, 0), 2)
            cv2.putText(display, self.tracker_type, (int(x), max(20, int(y) - 8)), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 2)
        else:
            bbox_msg.data = [0.0, 0.0, 0.0, 0.0, 0.0]
            cv2.putText(display, "TRACK LOST", (20, 40), cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 0, 255), 2)

        self.bbox_pub.publish(bbox_msg)
        self.status_pub.publish(String(data=f"frames={self.frame_count} success={success}"))

        if self.debug_pub is not None:
            self.debug_pub.publish(numpy_to_image_msg(display, msg.header))

        if self.show_window:
            cv2.imshow(self.window_name, display)
            cv2.waitKey(1)

        now = time.time()
        if now - self.last_log_time >= 5.0:
            rospy.loginfo("Tracker processed %d frames", self.frame_count)
            self.last_log_time = now


def shutdown_handler(*_):
    cv2.destroyAllWindows()
    sys.exit(0)


def main():
    signal.signal(signal.SIGINT, shutdown_handler)
    rospy.init_node("dingo_image_tracker")
    RosImageTracker()
    rospy.spin()


if __name__ == "__main__":
    main()
