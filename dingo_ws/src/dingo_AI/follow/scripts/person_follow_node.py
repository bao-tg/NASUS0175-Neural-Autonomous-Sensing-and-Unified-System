#!/usr/bin/env python3
import json
import math
import threading
import time
from dataclasses import dataclass
from typing import Dict, List, Optional

import cv2
import numpy as np
import rospy
from cv_bridge import CvBridge, CvBridgeError
from sensor_msgs.msg import Image, Joy
from std_msgs.msg import Bool, Float32MultiArray, String


@dataclass
class Track:
    track_id: int
    x1: float
    y1: float
    x2: float
    y2: float
    score: float
    cls: int

    @property
    def cx(self) -> float:
        return 0.5 * (self.x1 + self.x2)

    @property
    def cy(self) -> float:
        return 0.5 * (self.y1 + self.y2)

    @property
    def width(self) -> float:
        return max(1.0, self.x2 - self.x1)

    @property
    def height(self) -> float:
        return max(1.0, self.y2 - self.y1)

    @property
    def area(self) -> float:
        return self.width * self.height


class PersonFollowNode:
    def __init__(self):
        self.tracks_topic = rospy.get_param("~tracks_topic", "/yolo_tracker/tracks")
        self.image_topic = rospy.get_param("~image_topic", "/camera/image_raw")
        self.image_width = float(rospy.get_param("~image_width", 640.0))
        self.image_height = float(rospy.get_param("~image_height", 480.0))
        self.lock_delay = float(rospy.get_param("~lock_delay", 10.0))
        self.command_rate = float(rospy.get_param("~command_rate", 1.0))
        self.target_timeout = float(rospy.get_param("~target_timeout", 2.0))
        self.reacquire_timeout = float(rospy.get_param("~reacquire_timeout", 50.0))

        self.center_deadband_ratio = float(rospy.get_param("~center_deadband_ratio", 0.08))
        self.distance_deadband_ratio = float(rospy.get_param("~distance_deadband_ratio", 0.12))
        self.yaw_gain = float(rospy.get_param("~yaw_gain", 0.55))
        self.max_yaw_cmd = float(rospy.get_param("~max_yaw_cmd", 0.35))
        self.area_gain = float(rospy.get_param("~area_gain", 1.2))
        self.target_height_ratio = float(rospy.get_param("~target_height_ratio", 0.8))
        self.target_area_ratio = float(rospy.get_param("~target_area_ratio", 0.25))
        self.step_scale = float(rospy.get_param("~step_scale", 5.0))
        self.max_steps = int(rospy.get_param("~max_steps", 2))
        self.min_command_interval = float(rospy.get_param("~min_command_interval", 1.0))
        self.smooth_alpha = float(rospy.get_param("~smooth_alpha", 0.35))
        self.min_track_score = float(rospy.get_param("~min_track_score", 0.25))
        self.person_class = int(rospy.get_param("~person_class", 0))
        self.publish_zero_yaw = bool(rospy.get_param("~publish_zero_yaw", False))
        self.publish_debug_info = bool(rospy.get_param("~publish_debug_info", False))
        self.enabled = bool(rospy.get_param("~enabled", True))

        self.joy_topic = rospy.get_param("~joy_topic", "joy")
        self.joy_rate_hz = float(rospy.get_param("~joy_rate_hz", 30.0))
        self.linear_axis_value = float(rospy.get_param("~linear_axis_value", 0.5))
        self.linear_axis_index = int(rospy.get_param("~linear_axis_index", 1))
        self.step_duration = float(rospy.get_param("~step_duration", 0.35))
        self.yaw_duration = float(rospy.get_param("~yaw_duration", 0.45))
        self.button_pulse_duration = float(rospy.get_param("~button_pulse_duration", 0.18))
        self.button_trot = int(rospy.get_param("~button_trot", 5))
        self.yaw_axis_sign = float(rospy.get_param("~yaw_axis_sign", -1.0))
        self.yaw_axis_scale = float(rospy.get_param("~yaw_axis_scale", 2.0))
        self.yaw_axis_index = int(rospy.get_param("~yaw_axis_index", 2))
        self.return_to_rest_after_command = bool(rospy.get_param("~return_to_rest_after_command", True))

        self.enable_appearance_reacquire = bool(rospy.get_param("~enable_appearance_reacquire", True))
        self.appearance_match_threshold = float(rospy.get_param("~appearance_match_threshold", 0.55))
        self.appearance_update_alpha = float(rospy.get_param("~appearance_update_alpha", 0.03))
        self.min_hist_crop_area = int(rospy.get_param("~min_hist_crop_area", 400))

        self.start_time = time.time()
        self.target_id: Optional[int] = None
        self.goal_width: Optional[float] = None
        self.last_tracks: Dict[int, Track] = {}
        self.last_target_track: Optional[Track] = None
        self.last_track_time = 0.0
        self.last_target_seen_time = 0.0
        self.last_command_time = 0.0
        self.smoothed_yaw = 0.0
        self.state = "waiting_to_lock"
        self.command_lock = threading.Lock()
        self.command_busy = False
        self.in_trot = False

        self.bridge = CvBridge()
        self.lock = threading.Lock()
        self.latest_image: Optional[np.ndarray] = None
        self.image_sub = None
        self.target_hist: Optional[np.ndarray] = None
        self.last_hist_score: Optional[float] = None

        self.joy_pub = rospy.Publisher(self.joy_topic, Joy, queue_size=10)
        self.target_bbox_pub = rospy.Publisher("~target_bbox", Float32MultiArray, queue_size=1)
        self.status_pub = rospy.Publisher("~status", String, queue_size=1)
        self.debug_info_pub = rospy.Publisher("~debug_info", String, queue_size=1) if self.publish_debug_info else None
        self.sub = rospy.Subscriber(self.tracks_topic, Float32MultiArray, self.tracks_callback, queue_size=1)
        self.enable_sub = rospy.Subscriber("/person_follow/enable", Bool, self.enable_callback, queue_size=1)
        self.timer = rospy.Timer(rospy.Duration(1.0 / max(self.command_rate, 0.1)), self.control_tick)
        rospy.loginfo(
            "Person follow enabled=%s waits %.1fs before lock; tracks=%s image=%s appearance_reacquire=%s threshold=%.2f",
            self.enabled,
            self.lock_delay,
            self.tracks_topic,
            self.image_topic,
            self.enable_appearance_reacquire,
            self.appearance_match_threshold,
        )


    def reset_follow_state(self):
        self.start_time = time.time()
        self.target_id = None
        self.goal_width = None
        self.last_target_track = None
        self.last_target_seen_time = 0.0
        self.last_command_time = 0.0
        self.smoothed_yaw = 0.0
        self.target_hist = None
        self.last_hist_score = None
        self.state = "waiting_to_lock"
        self.stop_image_subscription()
        self.publish_neutral()

    def enable_callback(self, msg: Bool):
        requested = bool(msg.data)
        if requested:
            self.enabled = True
            self.reset_follow_state()
            rospy.loginfo("Person follow enabled by /person_follow/enable; starting %.1fs lock countdown", self.lock_delay)
            return
        if not self.enabled:
            return
        self.enabled = False
        self.reset_follow_state()
        self.ensure_rest()
        self.state = "disabled"
        self.publish_target_bbox(None)
        self.status_pub.publish(String(data=self.state))
        self.publish_debug_command("disabled", None)
        rospy.loginfo("Person follow disabled by /person_follow/enable")


    def ensure_image_subscription(self):
        if not self.enable_appearance_reacquire or self.image_sub is not None:
            return
        self.image_sub = rospy.Subscriber(self.image_topic, Image, self.image_callback, queue_size=1, buff_size=2**24)
        rospy.loginfo("Subscribed to %s for appearance matching", self.image_topic)

    def stop_image_subscription(self):
        if self.image_sub is None:
            return
        self.image_sub.unregister()
        self.image_sub = None
        with self.lock:
            self.latest_image = None
        rospy.loginfo("Unsubscribed from %s after appearance matching", self.image_topic)

    def image_callback(self, msg: Image):
        try:
            image = self.bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
        except CvBridgeError as exc:
            rospy.logwarn_throttle(5.0, "Failed to convert follow image: %s", exc)
            return
        with self.lock:
            self.latest_image = image

    def parse_tracks(self, msg: Float32MultiArray) -> List[Track]:
        data = list(msg.data)
        tracks = []
        stride = 7
        for i in range(0, len(data) - stride + 1, stride):
            track = Track(
                track_id=int(data[i]),
                x1=float(data[i + 1]),
                y1=float(data[i + 2]),
                x2=float(data[i + 3]),
                y2=float(data[i + 4]),
                score=float(data[i + 5]),
                cls=int(data[i + 6]),
            )
            if track.cls == self.person_class and track.score >= self.min_track_score:
                tracks.append(track)
        return tracks

    def tracks_callback(self, msg: Float32MultiArray):
        tracks = self.parse_tracks(msg)
        with self.lock:
            self.last_tracks = {track.track_id: track for track in tracks}
            self.last_track_time = time.time()

    def current_image(self) -> Optional[np.ndarray]:
        with self.lock:
            return None if self.latest_image is None else self.latest_image.copy()

    def current_tracks(self) -> Dict[int, Track]:
        with self.lock:
            return dict(self.last_tracks)

    def crop_track(self, image: np.ndarray, track: Track) -> Optional[np.ndarray]:
        height, width = image.shape[:2]
        x1 = max(0, min(width - 1, int(round(track.x1))))
        y1 = max(0, min(height - 1, int(round(track.y1))))
        x2 = max(0, min(width, int(round(track.x2))))
        y2 = max(0, min(height, int(round(track.y2))))
        if x2 <= x1 or y2 <= y1:
            return None
        crop = image[y1:y2, x1:x2]
        if crop.shape[0] * crop.shape[1] < self.min_hist_crop_area:
            return None
        return crop

    def compute_histogram(self, image: np.ndarray, track: Track) -> Optional[np.ndarray]:
        crop = self.crop_track(image, track)
        if crop is None:
            return None
        hsv = cv2.cvtColor(crop, cv2.COLOR_BGR2HSV)
        mask = cv2.inRange(hsv, np.array((0, 30, 30), dtype=np.uint8), np.array((180, 255, 255), dtype=np.uint8))
        hist = cv2.calcHist([hsv], [0, 1], mask, [32, 32], [0, 180, 0, 256])
        cv2.normalize(hist, hist, alpha=0.0, beta=1.0, norm_type=cv2.NORM_MINMAX)
        if not np.isfinite(hist).all() or float(hist.sum()) <= 0.0:
            return None
        return hist

    def update_target_histogram(self, track: Track, allow_initialize: bool = False):
        if not self.enable_appearance_reacquire:
            return
        self.ensure_image_subscription()
        image = self.current_image()
        if image is None:
            return
        hist = self.compute_histogram(image, track)
        if hist is None:
            return
        if self.target_hist is None or allow_initialize:
            self.target_hist = hist
            self.last_hist_score = 1.0
            self.stop_image_subscription()
            rospy.loginfo("Stored target appearance histogram for id=%s", track.track_id)
            return
        alpha = max(0.0, min(1.0, self.appearance_update_alpha))
        if alpha > 0.0:
            self.target_hist = cv2.normalize((1.0 - alpha) * self.target_hist + alpha * hist, None, 0.0, 1.0, cv2.NORM_MINMAX)

    def histogram_similarity(self, track: Track) -> Optional[float]:
        if not self.enable_appearance_reacquire or self.target_hist is None:
            return None
        self.ensure_image_subscription()
        image = self.current_image()
        if image is None:
            return None
        hist = self.compute_histogram(image, track)
        if hist is None:
            return None
        return float(cv2.compareHist(self.target_hist.astype(np.float32), hist.astype(np.float32), cv2.HISTCMP_CORREL))

    def select_initial_target(self) -> Optional[Track]:
        tracks = self.current_tracks()
        if not tracks:
            return None
        image_center = 0.5 * self.image_width

        def score(track: Track) -> float:
            center_error = abs(track.cx - image_center) / max(self.image_width, 1.0)
            area_score = track.area / max(self.image_width * self.image_width, 1.0)
            return area_score - 1.5 * center_error

        return max(tracks.values(), key=score)

    def maybe_lock_target(self):
        if self.target_id is not None:
            return
        elapsed = time.time() - self.start_time
        if elapsed < self.lock_delay:
            self.state = f"waiting_to_lock elapsed={elapsed:.1f}/{self.lock_delay:.1f}"
            if self.enable_appearance_reacquire:
                self.ensure_image_subscription()
            return
        target = self.select_initial_target()
        if target is None:
            self.state = "lock_waiting_no_tracks"
            return
        self.target_id = target.track_id
        self.goal_width = target.width
        self.last_target_track = target
        self.last_target_seen_time = time.time()
        self.update_target_histogram(target, allow_initialize=True)
        if not self.enable_appearance_reacquire or self.target_hist is not None:
            self.stop_image_subscription()
        self.state = f"locked id={self.target_id} target_area_ratio={self.target_area_ratio:.3f}"
        rospy.loginfo("Locked follow target id=%s bbox=(%.1f %.1f %.1f %.1f) target_area_ratio=%.3f", self.target_id, target.x1, target.y1, target.x2, target.y2, self.target_area_ratio)

    def reacquire_by_appearance(self, tracks: Dict[int, Track]) -> Optional[Track]:
        if not tracks:
            return None
        best_track = None
        best_score = -2.0
        for track in tracks.values():
            score = self.histogram_similarity(track)
            if score is not None and score > best_score:
                best_score = score
                best_track = track
        self.last_hist_score = best_score if best_track is not None else None
        if best_track is not None and best_score >= self.appearance_match_threshold:
            self.target_id = best_track.track_id
            self.last_target_track = best_track
            self.last_target_seen_time = time.time()
            self.update_target_histogram(best_track)
            self.stop_image_subscription()
            rospy.logwarn_throttle(2.0, "Appearance reacquired target as id=%s hist=%.3f", self.target_id, best_score)
            return best_track
        if best_track is not None:
            self.state = f"appearance_reject best_id={best_track.track_id} hist={best_score:.3f} threshold={self.appearance_match_threshold:.3f}"
        return None

    def reacquire_by_position(self, tracks: Dict[int, Track]) -> Optional[Track]:
        if self.last_target_track is None or not tracks:
            return None
        last = self.last_target_track
        nearest = min(tracks.values(), key=lambda t: math.hypot(t.cx - last.cx, t.cy - last.cy))
        self.last_target_track = nearest
        self.target_id = nearest.track_id
        self.last_target_seen_time = time.time()
        self.update_target_histogram(nearest)
        self.stop_image_subscription()
        rospy.logwarn_throttle(2.0, "Position reacquired target as id=%s", self.target_id)
        return nearest

    def get_target_track(self) -> Optional[Track]:
        if self.target_id is None:
            return None
        tracks = self.current_tracks()
        if self.target_id in tracks:
            self.last_target_track = tracks[self.target_id]
            self.last_target_seen_time = time.time()
            self.stop_image_subscription()
            return self.last_target_track
        if self.last_target_track is None:
            return None
        if time.time() - self.last_target_seen_time > self.reacquire_timeout:
            self.stop_image_subscription()
            self.state = f"target_lost id={self.target_id}"
            return None
        self.ensure_image_subscription()
        match = self.reacquire_by_appearance(tracks)
        if match is not None:
            return match
        if self.target_hist is None:
            return self.reacquire_by_position(tracks)
        return None

    def publish_target_bbox(self, track: Optional[Track]):
        msg = Float32MultiArray()
        if track is None:
            msg.data = [0.0, 0.0, 0.0, 0.0, 0.0]
        else:
            msg.data = [1.0, track.x1, track.y1, track.width, track.height]
        self.target_bbox_pub.publish(msg)

    def publish_debug_command(self, action, track: Optional[Track], center_error_norm=None, yaw_axis=0.0, steps=0, raw_steps=0.0, height_ratio=None, area_ratio=None, distance_error=None):
        if self.debug_info_pub is None:
            return
        payload = {
            "stamp": rospy.Time.now().to_sec(),
            "state": self.state,
            "action": action,
            "target_id": self.target_id,
            "enabled": self.enabled,
            "center_error_norm": center_error_norm,
            "smoothed_yaw": self.smoothed_yaw,
            "yaw_axis": yaw_axis,
            "steps": int(steps),
            "raw_steps": raw_steps,
            "step_scale": self.step_scale,
            "height_ratio": height_ratio,
            "area_ratio": area_ratio,
            "target_area_ratio": self.target_area_ratio,
            "target_height_ratio": self.target_height_ratio,
            "distance_error": distance_error,
            "command_busy": self.command_busy,
            "in_trot": self.in_trot,
        }
        if track is None:
            payload["bbox"] = None
        else:
            payload["bbox"] = {
                "x1": track.x1,
                "y1": track.y1,
                "x2": track.x2,
                "y2": track.y2,
                "cx": track.cx,
                "cy": track.cy,
                "width": track.width,
                "height": track.height,
                "area": track.area,
                "score": track.score,
                "class": track.cls,
            }
        self.debug_info_pub.publish(String(data=json.dumps(payload, sort_keys=True)))

    def neutral_axes(self):
        return [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]

    def neutral_buttons(self):
        return [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0]

    def publish_joy(self, axes, buttons):
        msg = Joy()
        msg.header.stamp = rospy.Time.now()
        msg.axes = list(axes)
        msg.buttons = list(buttons)
        self.joy_pub.publish(msg)

    def publish_neutral(self):
        self.publish_joy(self.neutral_axes(), self.neutral_buttons())

    def hold_message(self, axes, buttons, duration):
        period = 1.0 / max(self.joy_rate_hz, 1.0)
        end_time = time.time() + max(duration, period)
        while not rospy.is_shutdown() and time.time() < end_time:
            self.publish_joy(axes, buttons)
            time.sleep(period)

    def pulse_button(self, button_index):
        buttons = self.neutral_buttons()
        buttons[button_index] = 1
        self.hold_message(self.neutral_axes(), buttons, self.button_pulse_duration)
        self.publish_neutral()

    def ensure_trot(self):
        if not self.in_trot:
            self.pulse_button(self.button_trot)
            self.in_trot = True
            rospy.sleep(0.15)

    def ensure_rest(self):
        if self.in_trot:
            self.pulse_button(self.button_trot)
            self.in_trot = False
            rospy.sleep(0.15)

    def run_joy_command(self, axes, duration):
        try:
            self.ensure_trot()
            self.hold_message(axes, self.neutral_buttons(), duration)
            self.publish_neutral()
            if self.return_to_rest_after_command:
                self.ensure_rest()
        finally:
            with self.command_lock:
                self.command_busy = False

    def start_joy_command(self, axes, duration):
        with self.command_lock:
            if self.command_busy:
                return False
            self.command_busy = True
        worker = threading.Thread(target=self.run_joy_command, args=(list(axes), duration))
        worker.daemon = True
        worker.start()
        return True

    def control_tick(self, _event):
        if not self.enabled:
            self.state = "disabled"
            self.publish_target_bbox(None)
            self.status_pub.publish(String(data=self.state))
            self.publish_debug_command("disabled", None)
            return

        self.maybe_lock_target()
        target = self.get_target_track()
        self.publish_target_bbox(target)

        if target is None:
            self.status_pub.publish(String(data=self.state))
            self.publish_debug_command("no_target", None)
            return

        now = time.time()
        if now - self.last_command_time < self.min_command_interval:
            hist_text = "" if self.last_hist_score is None else f" hist={self.last_hist_score:.3f}"
            self.status_pub.publish(String(data=f"locked id={self.target_id} holding{hist_text}"))
            self.publish_debug_command("holding", target)
            return

        image_center = 0.5 * self.image_width
        center_error_norm = (target.cx - image_center) / max(image_center, 1.0)
        yaw_cmd = 0.0
        if abs(center_error_norm) > self.center_deadband_ratio:
            raw_yaw = self.yaw_gain * center_error_norm
            yaw_cmd = max(-self.max_yaw_cmd, min(self.max_yaw_cmd, raw_yaw))
            self.smoothed_yaw = (1.0 - self.smooth_alpha) * self.smoothed_yaw + self.smooth_alpha * yaw_cmd
            axes = self.neutral_axes()
            axes[self.yaw_axis_index] = max(-1.0, min(1.0, self.yaw_axis_sign * self.smoothed_yaw * self.yaw_axis_scale))
            if self.start_joy_command(axes, self.yaw_duration):
                self.last_command_time = now
                self.state = f"turn id={self.target_id} yaw_axis={axes[self.yaw_axis_index]:.3f} err={center_error_norm:.3f}"
                action = "turn"
            else:
                self.state = f"turn_wait id={self.target_id} err={center_error_norm:.3f}"
                action = "turn_wait"
            self.status_pub.publish(String(data=self.state))
            self.publish_debug_command(action, target, center_error_norm=center_error_norm, yaw_axis=axes[self.yaw_axis_index])
            return

        if self.publish_zero_yaw and abs(self.smoothed_yaw) > 1e-3:
            self.smoothed_yaw = 0.0
            self.publish_neutral()

        height_ratio = target.height / max(self.image_height, 1.0)
        area_ratio = target.area / max(self.image_width * self.image_height, 1.0)
        distance_error = self.target_area_ratio - area_ratio
        steps = 0
        if distance_error > self.distance_deadband_ratio:
            raw_steps = self.area_gain * distance_error * self.step_scale
            steps = int(round(raw_steps))
            steps = max(1, min(self.max_steps, steps))
            axes = self.neutral_axes()
            axes[self.linear_axis_index] = self.linear_axis_value
            if self.start_joy_command(axes, max(steps, 1) * self.step_duration):
                self.last_command_time = now
                self.state = f"forward id={self.target_id} steps={steps} axis={axes[self.linear_axis_index]:.2f} area_ratio={area_ratio:.3f} target={self.target_area_ratio:.3f}"
                action = "forward"
            else:
                self.state = f"forward_wait id={self.target_id} steps={steps} area_ratio={area_ratio:.3f}"
                action = "forward_wait"
            self.publish_debug_command(action, target, center_error_norm=center_error_norm, yaw_axis=0.0, steps=steps, raw_steps=raw_steps, height_ratio=height_ratio, area_ratio=area_ratio, distance_error=distance_error)
        else:
            self.state = f"aligned id={self.target_id} area_ratio={area_ratio:.3f} target={self.target_area_ratio:.3f} height_ratio={height_ratio:.2f} center_err={center_error_norm:.3f}"
            self.publish_debug_command("aligned", target, center_error_norm=center_error_norm, yaw_axis=0.0, steps=0, raw_steps=0.0, height_ratio=height_ratio, area_ratio=area_ratio, distance_error=distance_error)

        self.status_pub.publish(String(data=self.state))


def main():
    rospy.init_node("person_follow")
    PersonFollowNode()
    rospy.spin()


if __name__ == "__main__":
    main()
