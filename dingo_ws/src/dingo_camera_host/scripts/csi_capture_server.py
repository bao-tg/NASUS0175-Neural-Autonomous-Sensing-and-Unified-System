#!/usr/bin/env python3

import argparse
import os
import subprocess
import tempfile
import threading
from http.server import BaseHTTPRequestHandler, HTTPServer



def build_csi_pipeline(capture_width, capture_height, display_width, display_height, framerate, flip_method):
    return (
        "nvarguscamerasrc ! "
        "video/x-raw(memory:NVMM), width=(int)%d, height=(int)%d, framerate=(fraction)%d/1 ! "
        "nvvidconv flip-method=%d ! "
        "video/x-raw, width=(int)%d, height=(int)%d, format=(string)BGRx ! "
        "videoconvert ! video/x-raw, format=(string)BGR ! appsink drop=true max-buffers=1"
        % (capture_width, capture_height, framerate, flip_method, display_width, display_height)
    )


def build_gst_launch_command(args, output_path):
    return [
        "gst-launch-1.0",
        "-q",
        "nvarguscamerasrc",
        "num-buffers=1",
        "!",
        "video/x-raw(memory:NVMM),width=%d,height=%d,framerate=%d/1"
        % (args.capture_width, args.capture_height, args.framerate),
        "!",
        "nvvidconv",
        "flip-method=%d" % args.flip_method,
        "!",
        "video/x-raw,width=%d,height=%d,format=BGRx" % (args.display_width, args.display_height),
        "!",
        "videoconvert",
        "!",
        "jpegenc",
        "quality=%d" % args.jpeg_quality,
        "!",
        "filesink",
        "location=%s" % output_path,
    ]


class CsiCamera:
    def __init__(self, args):
        self.args = args
        self.lock = threading.Lock()
        self.capture = None
        if self.args.backend == "opencv":
            self.open_opencv()

    def open_opencv(self):
        import cv2

        pipeline = build_csi_pipeline(
            self.args.capture_width,
            self.args.capture_height,
            self.args.display_width,
            self.args.display_height,
            self.args.framerate,
            self.args.flip_method,
        )
        if self.args.print_pipeline:
            print("OpenCV pipeline: %s" % pipeline, flush=True)
        self.capture = cv2.VideoCapture(pipeline, cv2.CAP_GSTREAMER)
        if not self.capture.isOpened():
            raise RuntimeError("could not open CSI camera with OpenCV GStreamer backend")
        for _ in range(self.args.warmup_frames):
            self.capture.read()

    def capture_jpeg(self):
        with self.lock:
            if self.args.backend == "gst-launch":
                return self.capture_jpeg_gst_launch()
            return self.capture_jpeg_opencv()

    def capture_jpeg_opencv(self):
        import cv2

        frame = None
        for _ in range(self.args.read_frames):
            ok, frame = self.capture.read()
            if not ok:
                frame = None
        if frame is None:
            self.capture.release()
            self.open_opencv()
            ok, frame = self.capture.read()
            if not ok:
                raise RuntimeError("could not read frame from CSI camera")

        ok, encoded = cv2.imencode(".jpg", frame, [int(cv2.IMWRITE_JPEG_QUALITY), self.args.jpeg_quality])
        if not ok:
            raise RuntimeError("could not encode camera frame as JPEG")
        return encoded.tobytes()

    def capture_jpeg_gst_launch(self):
        fd, output_path = tempfile.mkstemp(prefix="dingo_csi_capture_", suffix=".jpg")
        os.close(fd)
        try:
            cmd = build_gst_launch_command(self.args, output_path)
            if self.args.print_pipeline:
                print("gst-launch command: %s" % " ".join(cmd), flush=True)
            result = subprocess.run(cmd, stdout=subprocess.PIPE, stderr=subprocess.PIPE, timeout=self.args.capture_timeout)
            if result.returncode != 0:
                stderr = result.stderr.decode("utf-8", "replace").strip()
                raise RuntimeError("gst-launch capture failed: %s" % stderr)
            with open(output_path, "rb") as image_file:
                data = image_file.read()
            if not data:
                raise RuntimeError("gst-launch produced an empty JPEG")
            return data
        finally:
            try:
                os.remove(output_path)
            except OSError:
                pass


def make_handler(camera):
    class CaptureHandler(BaseHTTPRequestHandler):
        def do_GET(self):
            if self.path in ["/health", "/healthz"]:
                self.send_response(200)
                self.send_header("Content-Type", "text/plain")
                self.end_headers()
                self.wfile.write(b"ok\n")
                return

            if self.path not in ["/capture.jpg", "/capture"]:
                self.send_response(404)
                self.end_headers()
                return

            try:
                jpeg = camera.capture_jpeg()
            except Exception as exc:
                message = str(exc).encode("utf-8", "replace")
                self.send_response(500)
                self.send_header("Content-Type", "text/plain")
                self.send_header("Content-Length", str(len(message)))
                self.end_headers()
                self.wfile.write(message)
                return

            self.send_response(200)
            self.send_header("Content-Type", "image/jpeg")
            self.send_header("Content-Length", str(len(jpeg)))
            self.end_headers()
            self.wfile.write(jpeg)

        def log_message(self, fmt, *args):
            if camera.args.verbose:
                super().log_message(fmt, *args)

    return CaptureHandler


def parse_args():
    parser = argparse.ArgumentParser(description="Single-frame CSI capture service for Jetson host")
    parser.add_argument("--host", default="127.0.0.1")
    parser.add_argument("--port", type=int, default=8081)
    parser.add_argument("--backend", choices=["gst-launch", "opencv"], default="gst-launch")
    parser.add_argument("--capture-width", type=int, default=1280)
    parser.add_argument("--capture-height", type=int, default=720)
    parser.add_argument("--display-width", type=int, default=640)
    parser.add_argument("--display-height", type=int, default=360)
    parser.add_argument("--framerate", type=int, default=30)
    parser.add_argument("--flip-method", type=int, default=0)
    parser.add_argument("--jpeg-quality", type=int, default=85)
    parser.add_argument("--warmup-frames", type=int, default=8)
    parser.add_argument("--read-frames", type=int, default=2)
    parser.add_argument("--capture-timeout", type=float, default=8.0)
    parser.add_argument("--print-pipeline", action="store_true")
    parser.add_argument("--verbose", action="store_true")
    return parser.parse_args()


def main():
    args = parse_args()
    camera = CsiCamera(args)
    server = HTTPServer((args.host, args.port), make_handler(camera))
    print("CSI capture server listening on http://%s:%d/capture.jpg" % (args.host, args.port), flush=True)
    server.serve_forever()


if __name__ == "__main__":
    main()
