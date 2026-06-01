#!/usr/bin/env python3
import os
import signal
import sys
import time
from dataclasses import dataclass
from typing import List, Optional, Sequence, Tuple

import cv2
import numpy as np
import rospy
from sensor_msgs.msg import Image
from std_msgs.msg import Float32MultiArray, String

try:
    from ultralytics import YOLO
except Exception as exc:  # imported after conda activation by run_yolo_tracker.sh
    YOLO = None
    YOLO_IMPORT_ERROR = exc
else:
    YOLO_IMPORT_ERROR = None


@dataclass
class Track:
    track_id: int
    bbox: np.ndarray
    missed: int = 0


def image_msg_to_numpy(msg: Image) -> np.ndarray:
    channels_by_encoding = {"rgb8": 3, "bgr8": 3, "mono8": 1}
    if msg.encoding not in channels_by_encoding:
        raise ValueError(f"Unsupported image encoding: {msg.encoding}")

    channels = channels_by_encoding[msg.encoding]
    height = msg.height
    width = msg.width
    expected_step = width * channels
    frame = np.frombuffer(msg.data, dtype=np.uint8).reshape((height, msg.step))[:, :expected_step]
    if channels == 1:
        return cv2.cvtColor(frame.reshape((height, width)), cv2.COLOR_GRAY2BGR).copy()
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


def parse_classes(value) -> Optional[List[int]]:
    if value in (None, "", "all"):
        return None
    if isinstance(value, int):
        return [value]
    if isinstance(value, str):
        return [int(part.strip()) for part in value.split(",") if part.strip()]
    if isinstance(value, Sequence):
        return [int(v) for v in value]
    return None


def xyxy_iou(a: np.ndarray, b: np.ndarray) -> float:
    ix1 = max(float(a[0]), float(b[0]))
    iy1 = max(float(a[1]), float(b[1]))
    ix2 = min(float(a[2]), float(b[2]))
    iy2 = min(float(a[3]), float(b[3]))
    iw = max(0.0, ix2 - ix1)
    ih = max(0.0, iy2 - iy1)
    inter = iw * ih
    area_a = max(0.0, float(a[2] - a[0])) * max(0.0, float(a[3] - a[1]))
    area_b = max(0.0, float(b[2] - b[0])) * max(0.0, float(b[3] - b[1]))
    union = area_a + area_b - inter
    return inter / union if union > 0 else 0.0


class SimpleIouTracker:
    def __init__(self, iou_threshold: float = 0.3, max_missed: int = 10):
        self.iou_threshold = iou_threshold
        self.max_missed = max_missed
        self.next_id = 1
        self.tracks: List[Track] = []

    def update(self, detections: List[np.ndarray]) -> List[Track]:
        unmatched_dets = set(range(len(detections)))
        matched_tracks = set()

        pairs = []
        for ti, track in enumerate(self.tracks):
            for di, det in enumerate(detections):
                pairs.append((xyxy_iou(track.bbox, det), ti, di))
        pairs.sort(reverse=True, key=lambda item: item[0])

        for score, ti, di in pairs:
            if score < self.iou_threshold or ti in matched_tracks or di not in unmatched_dets:
                continue
            self.tracks[ti].bbox = detections[di]
            self.tracks[ti].missed = 0
            matched_tracks.add(ti)
            unmatched_dets.remove(di)

        for ti, track in enumerate(self.tracks):
            if ti not in matched_tracks:
                track.missed += 1

        self.tracks = [track for track in self.tracks if track.missed <= self.max_missed]
        for di in sorted(unmatched_dets):
            self.tracks.append(Track(self.next_id, detections[di]))
            self.next_id += 1
        return list(self.tracks)


class YoloTrackerNode:
    def __init__(self):
        if YOLO is None:
            raise RuntimeError(f"Failed to import ultralytics.YOLO: {YOLO_IMPORT_ERROR}")

        self.image_topic = rospy.get_param("~image_topic", "/camera/image_raw")
        self.model_path = rospy.get_param("~model_path", "yolov8n.engine")
        self.export_pt_path = rospy.get_param("~export_pt_path", "yolov8n.pt")
        self.export_engine_if_missing = bool(rospy.get_param("~export_engine_if_missing", False))
        self.tracking_backend = rospy.get_param("~tracking_backend", "bytetrack").lower()
        self.classes = parse_classes(rospy.get_param("~classes", "0"))
        self.conf = float(rospy.get_param("~conf", 0.25))
        self.iou = float(rospy.get_param("~iou", 0.7))
        self.imgsz = int(rospy.get_param("~imgsz", 640))
        self.device = rospy.get_param("~device", "0")
        self.half = bool(rospy.get_param("~half", True))
        self.max_width = int(rospy.get_param("~max_width", 640))
        self.publish_debug_image = bool(rospy.get_param("~publish_debug_image", True))
        self.show_window = bool(rospy.get_param("~show_window", False))
        self.iou_tracker_threshold = float(rospy.get_param("~iou_tracker_threshold", 0.3))
        self.iou_tracker_max_missed = int(rospy.get_param("~iou_tracker_max_missed", 10))

        self.scale = 1.0
        self.frame_count = 0
        self.last_log_time = time.time()
        self.window_name = "Dingo YOLO Tracker"
        self.iou_tracker = SimpleIouTracker(self.iou_tracker_threshold, self.iou_tracker_max_missed)

        self.model = self.load_model()
        self.bbox_pub = rospy.Publisher("~bbox", Float32MultiArray, queue_size=1)
        self.tracks_pub = rospy.Publisher("~tracks", Float32MultiArray, queue_size=1)
        self.status_pub = rospy.Publisher("~status", String, queue_size=1)
        self.debug_pub = rospy.Publisher("~debug_image", Image, queue_size=1) if self.publish_debug_image else None
        self.sub = rospy.Subscriber(self.image_topic, Image, self.image_callback, queue_size=1, buff_size=2**24)
        rospy.loginfo("YOLO tracker subscribed to %s using %s backend model=%s", self.image_topic, self.tracking_backend, self.model_path)

    def load_model(self):
        if self.export_engine_if_missing and self.model_path.endswith(".engine") and not os.path.exists(self.model_path):
            rospy.loginfo("TensorRT engine %s not found; exporting from %s", self.model_path, self.export_pt_path)
            YOLO(self.export_pt_path).export(format="engine", device=self.device, half=self.half)
        return YOLO(self.model_path)

    def maybe_resize(self, frame: np.ndarray) -> np.ndarray:
        h, w = frame.shape[:2]
        if self.max_width <= 0 or w <= self.max_width:
            self.scale = 1.0
            return frame
        self.scale = self.max_width / float(w)
        return cv2.resize(frame, (self.max_width, int(h * self.scale)))

    def run_ultralytics_track(self, frame: np.ndarray):
        tracker_file = "botsort.yaml" if self.tracking_backend == "botsort" else "bytetrack.yaml"
        results = self.model.track(
            frame,
            persist=True,
            classes=self.classes,
            conf=self.conf,
            iou=self.iou,
            imgsz=self.imgsz,
            device=self.device,
            half=self.half,
            tracker=tracker_file,
            verbose=False,
        )
        return results[0] if results else None

    def run_iou_track(self, frame: np.ndarray) -> List[Track]:
        results = self.model.predict(
            frame,
            classes=self.classes,
            conf=self.conf,
            iou=self.iou,
            imgsz=self.imgsz,
            device=self.device,
            half=self.half,
            verbose=False,
        )
        if not results or results[0].boxes is None or len(results[0].boxes) == 0:
            return self.iou_tracker.update([])
        xyxy = results[0].boxes.xyxy.cpu().numpy()
        return self.iou_tracker.update([box.astype(np.float32) for box in xyxy])

    def publish_tracks(self, tracks: List[Tuple[int, np.ndarray]], display: np.ndarray, header):
        tracks_msg = Float32MultiArray()
        bbox_msg = Float32MultiArray()
        flat = []

        for track_id, box in tracks:
            x1, y1, x2, y2 = [float(v) for v in box]
            flat.extend([float(track_id), x1 / self.scale, y1 / self.scale, x2 / self.scale, y2 / self.scale])
            cv2.rectangle(display, (int(x1), int(y1)), (int(x2), int(y2)), (0, 200, 255), 2)
            cv2.putText(display, f"id={track_id}", (int(x1), max(20, int(y1) - 8)), cv2.FONT_HERSHEY_SIMPLEX, 0.55, (0, 200, 255), 2)

        tracks_msg.data = flat
        if tracks:
            x1, y1, x2, y2 = [float(v) for v in tracks[0][1]]
            bbox_msg.data = [1.0, x1 / self.scale, y1 / self.scale, (x2 - x1) / self.scale, (y2 - y1) / self.scale]
        else:
            bbox_msg.data = [0.0, 0.0, 0.0, 0.0, 0.0]
            cv2.putText(display, "NO YOLO TRACK", (20, 40), cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 0, 255), 2)

        self.tracks_pub.publish(tracks_msg)
        self.bbox_pub.publish(bbox_msg)
        if self.debug_pub is not None:
            self.debug_pub.publish(numpy_to_image_msg(display, header))

    def image_callback(self, msg: Image):
        try:
            frame = self.maybe_resize(image_msg_to_numpy(msg))
        except Exception as exc:
            rospy.logwarn_throttle(5.0, "Failed to convert image: %s", exc)
            return

        self.frame_count += 1
        display = frame.copy()

        try:
            if self.tracking_backend == "iou":
                tracks = [(track.track_id, track.bbox) for track in self.run_iou_track(frame)]
            elif self.tracking_backend in ("bytetrack", "botsort"):
                result = self.run_ultralytics_track(frame)
                tracks = self.extract_ultralytics_tracks(result)
            else:
                rospy.logwarn_throttle(5.0, "Unknown YOLO tracking_backend=%s; using bytetrack", self.tracking_backend)
                result = self.run_ultralytics_track(frame)
                tracks = self.extract_ultralytics_tracks(result)
        except Exception as exc:
            rospy.logerr_throttle(5.0, "YOLO tracking failed: %s", exc)
            return

        self.publish_tracks(tracks, display, msg.header)
        self.status_pub.publish(String(data=f"frames={self.frame_count} backend={self.tracking_backend} tracks={len(tracks)}"))

        if self.show_window:
            cv2.imshow(self.window_name, display)
            cv2.waitKey(1)

        now = time.time()
        if now - self.last_log_time >= 5.0:
            rospy.loginfo("YOLO tracker processed %d frames; tracks=%d", self.frame_count, len(tracks))
            self.last_log_time = now

    @staticmethod
    def extract_ultralytics_tracks(result) -> List[Tuple[int, np.ndarray]]:
        if result is None or result.boxes is None or len(result.boxes) == 0:
            return []
        boxes = result.boxes.xyxy.cpu().numpy()
        if result.boxes.id is None:
            ids = np.arange(1, len(boxes) + 1)
        else:
            ids = result.boxes.id.cpu().numpy().astype(int)
        return [(int(track_id), box.astype(np.float32)) for track_id, box in zip(ids, boxes)]


def shutdown_handler(*_):
    cv2.destroyAllWindows()
    sys.exit(0)


def main():
    signal.signal(signal.SIGINT, shutdown_handler)
    rospy.init_node("dingo_yolo_tracker")
    YoloTrackerNode()
    rospy.spin()


if __name__ == "__main__":
    main()
