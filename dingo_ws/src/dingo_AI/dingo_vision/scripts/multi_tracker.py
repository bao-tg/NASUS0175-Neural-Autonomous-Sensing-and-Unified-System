import cv2
import numpy as np
from typing import List
import time
import os
import sys

# Configuration:
# Option 1 - Run in dingo-ros container with jetson-inference mounted:
#   sudo docker run -it --rm -v /home/bao-tg/Capstone/brain/bao-tg/jetson-inference/build/aarch64:/jetson-inference/build/aarch64 --device /dev/video0 --runtime nvidia dingo-ros bash
#   cd /jetson-inference/build/aarch64/bin && python3 /home/bao-tg/Capstone/brain/AI/optical_flow/Multi_tracker.py csi://0 MOSSE
#
# Option 2 - Run in jetson-inference container:
#   bash /home/bao-tg/Capstone/brain/bao-tg/jetson-inference/docker/run.sh
#   cd /jetson-inference/build/aarch64/bin && python3 /home/bao-tg/Capstone/brain/AI/optical_flow/Multi_tracker.py csi://0 MOSSE
#
# Option 3 - Run in conda env (phuc) with GStreamer subprocess:
#   conda activate phuc
#   python Multi_tracker.py csi://0 MIL

USE_JETSON_INFERENCE = True

try:
    from jetson_utils import videoSource
    JETSON_INFERENCE_AVAILABLE = True
    print("[INFO] jetson_utils available")
except ImportError:
    JETSON_INFERENCE_AVAILABLE = False
    print("[WARN] jetson_utils not available, will use GStreamer pipeline for CSI camera")
    USE_JETSON_INFERENCE = False

try:
    from gst_camera import GStreamerCSICamera, create_camera
    GST_CAMERA_AVAILABLE = True
    print("[INFO] gst_camera module available")
except ImportError:
    GST_CAMERA_AVAILABLE = False
    print("[WARN] gst_camera module not available")

HEADLESS_MODE = False
DEBUG_SAVE = False


def initialize_tracker(tracker_type: str, frame, roi):
    tracker_type = tracker_type.upper()

    if tracker_type == "MIL":
        tracker = cv2.TrackerMIL_create()
    elif tracker_type == "KCF":
        tracker = cv2.TrackerKCF_create()
    elif tracker_type == "CSRT":
        tracker = cv2.legacy.TrackerCSRT_create()
    elif tracker_type == "MOSSE":
        tracker = cv2.legacy.TrackerMOSSE_create()
    elif tracker_type == "GOTURN":
        tracker = cv2.TrackerGOTURN_create()
    else:
        raise ValueError(f"Tracker {tracker_type} not available. Use MIL, KCF, CSRT, MOSSE, or GOTURN.")

    tracker.init(frame, roi)
    return tracker


def select_object(cap, is_camera=False):
    ret, frame = cap.read()
    if not ret:
        raise RuntimeError("Cannot read the first frame from video.")

    if DEBUG_SAVE:
        debug_dir = "/home/bao-tg/Capstone/brain/AI/debug_raw"
        os.makedirs(debug_dir, exist_ok=True)
        cv2.imwrite(f"{debug_dir}/select_roi_raw.jpg", frame)
        print(f"[DEBUG] Saved select_object raw frame, shape: {frame.shape}")

    if is_camera:
        target_width = 640
        h, w = frame.shape[:2]
        scale = target_width / w
        target_height = int(h * scale)
        frame = cv2.resize(frame, (target_width, target_height))
        print(f"[INFO] Camera frame resized from {w}x{h} to {target_width}x{target_height}")

    if HEADLESS_MODE:
        print("[INFO] Headless mode - skipping ROI selection, using default center ROI")
        roi_h, roi_w = frame.shape[:2]
        roi = (roi_w//4, roi_h//4, roi_w//2, roi_h//2)
    else:
        print("[INFO] Select a bounding box and press ENTER or SPACE to confirm.")
        print("[INFO] Press 'c' to cancel selection.")

        roi = cv2.selectROI("Select Object to Track", frame, fromCenter=False, showCrosshair=True)
        cv2.destroyWindow("Select Object to Track")

        if roi == (0, 0, 0, 0):
            raise ValueError("No ROI selected. Exiting.")

    return roi, frame


def track_object(cap, tracker_types: List[str], roi, frame, is_camera=False):
    """
    Tracks the selected object using multiple trackers and compares their performance.
    Displays all tracker results side-by-side and prints per-tracker FPS.
    """
    # Resize camera frame for faster processing
    target_width = 640
    target_height = 480
    scale = 1.0
    
    if is_camera:
        h, w = frame.shape[:2]
        scale = target_width / w
        target_height = int(h * scale)
        frame = cv2.resize(frame, (target_width, target_height))
        roi = (int(roi[0] * scale), int(roi[1] * scale), int(roi[2] * scale), int(roi[3] * scale))
    # Assign a distinct color to each tracker
    colors = [
        (0, 255, 0),      # Green
        (255, 0, 0),      # Blue
        (0, 0, 255),      # Red
        (255, 255, 0),    # Cyan
        (255, 0, 255),    # Magenta
        (0, 255, 255),    # Yellow
        (128, 0, 255),    # Purple
        (0, 128, 255),    # Orange
    ]

    # Initialize all trackers
    trackers = []
    for t_type in tracker_types:
        print(f"[INFO] Initializing {t_type} tracker...")
        tracker = initialize_tracker(t_type, frame, roi)
        trackers.append(tracker)

    # Per-tracker timing accumulators
    frame_times = [[] for _ in tracker_types]
    frame_count = 0

    print("[INFO] Tracking started. Press 'q' to quit.")

    output_dir = "/home/bao-tg/Capstone/brain/AI/headless_output/tracker_output"
    if HEADLESS_MODE:
        os.makedirs(output_dir, exist_ok=True)
        print(f"[INFO] Headless mode: saving frames to {output_dir}")
    else:
        cv2.namedWindow("Multi-Tracker Comparison")

    while True:
        ret, frame = cap.read()
        if not ret:
            break

        # DEBUG: Save raw frame before processing
        if DEBUG_SAVE and frame_count < 3:
            debug_dir = "/home/bao-tg/Capstone/brain/AI/debug_raw"
            os.makedirs(debug_dir, exist_ok=True)
            debug_path = f"{debug_dir}/raw_{frame_count:04d}.jpg"
            cv2.imwrite(debug_path, frame)
            print(f"[DEBUG] Saved raw frame to {debug_path}")

        if is_camera:
            frame = cv2.resize(frame, (target_width, target_height))

        frame_count += 1
        display = frame.copy()

        for i, (tracker, t_type) in enumerate(zip(trackers, tracker_types)):
            t_start = time.perf_counter()
            success, bbox = tracker.update(frame)
            t_end = time.perf_counter()

            elapsed = t_end - t_start
            frame_times[i].append(elapsed)

            color = colors[i % len(colors)]

            if success:
                x, y, w, h = map(int, bbox)
                cv2.rectangle(display, (x, y), (x + w, y + h), color, 2)
                label = f"{t_type}"
                cv2.putText(display, label, (x, y - 8),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.6, color, 2)
            else:
                cv2.putText(display, f"{t_type}: LOST", (20, 40 + i * 25),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.6, color, 2)

        # Show overall frame count
        cv2.putText(display, f"Frame: {frame_count}", (20, 20),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 255), 2)

        if HEADLESS_MODE:
            output_path = f"{output_dir}/frame_{frame_count:04d}.jpg"
            cv2.imwrite(output_path, display)
        else:
            cv2.imshow("Multi-Tracker Comparison", display)

        if cv2.waitKey(1) & 0xFF == ord('q'):
            print("[INFO] Quit signal received.")
            break

    if HEADLESS_MODE:
        print(f"[INFO] Saved {frame_count} frames to {output_dir}")
    else:
        cv2.destroyAllWindows()

    # Report FPS for each tracker
    print("\n===== Tracker Performance =====")
    for i, t_type in enumerate(tracker_types):
        times = frame_times[i]
        if times:
            avg_time = np.mean(times)
            fps = 1.0 / avg_time if avg_time > 0 else 0
            print(f"  {t_type:<12} | Avg time/frame: {avg_time*1000:.2f} ms | FPS: {fps:.1f}")
    print("================================\n")


# ses_2600d633dffedWpLrt83AWTn44  Jetson hand pose model usage in Docker container setup  
# ses_2604d6793ffepWjtVEB0HbU56R  Fix cv2 TrackerKCF_create AttributeError             

def main(vid_path: str, tracker_types: List[str]):
    global HEADLESS_MODE
    
    if tracker_types is None:
        tracker_types = ["MIL"]

    print(f"[INFO] Opening video: {vid_path}")
    
    is_camera = vid_path in ["0", "csi://0"] or vid_path.isdigit()
    
    # Path 1: Use jetson-inference for camera capture (Docker environment)
    if is_camera and USE_JETSON_INFERENCE and JETSON_INFERENCE_AVAILABLE:
        print("[INFO] Using jetson-inference for camera capture")
        from jetson_utils import videoSource
        
        camera = videoSource("csi://0", argv=['--input-flip', 'rotate-180'])
        
        debug_dir = "/home/bao-tg/Capstone/brain/AI/debug_raw"
        os.makedirs(debug_dir, exist_ok=True)
        
        print("[INFO] Waiting for camera to initialize...")
        for attempt in range(10):
            frame = camera.Capture()
            if frame is not None:
                print(f"[INFO] Camera initialized on attempt {attempt+1}")
                break
            time.sleep(0.5)
        else:
            raise RuntimeError("Failed to capture from CSI camera after 10 attempts")
        
        from jetson_utils import videoOutput
        output = videoOutput("display://0")
        
        if hasattr(frame, 'tocpu'):
            frame = frame.tocpu()
        elif hasattr(frame, 'as_numpy'):
            frame = frame.as_numpy()
        if not isinstance(frame, np.ndarray):
            frame = np.array(frame)
        if not isinstance(frame, np.ndarray):
            frame = np.array(frame)
        
        print(f"[INFO] Captured frame shape: {frame.shape}")
        
        if DEBUG_SAVE:
            debug_dir = "/home/bao-tg/Capstone/brain/AI/debug_raw"
            os.makedirs(debug_dir, exist_ok=True)
            cv2.imwrite(f"{debug_dir}/jetson_frame.jpg", frame)
        
        if HEADLESS_MODE:
            roi_h, roi_w = frame.shape[:2]
            roi = (roi_w//4, roi_h//4, roi_w//2, roi_h//2)
            print(f"[INFO] Headless mode ROI: {roi}")
        else:
            try:
                roi = cv2.selectROI("Select Object to Track", frame, fromCenter=False, showCrosshair=True)
                cv2.destroyWindow("Select Object to Track")
            except Exception as e:
                print(f"[WARN] Cannot create GUI: {e}")
                print("[INFO] Using default center ROI")
                roi = None
            
            if roi is None or roi == (0, 0, 0, 0):
                roi_h, roi_w = frame.shape[:2]
                roi = (roi_w//4, roi_h//4, roi_w//2, roi_h//2)
                print(f"[INFO] Default center ROI: {roi}")
        
        print(f"[INFO] ROI selected: {roi}")
        
        trackers = []
        for t_type in tracker_types:
            print(f"[INFO] Initializing {t_type} tracker...")
            tracker = initialize_tracker(t_type, frame, roi)
            trackers.append(tracker)
        
        frame_times = [[] for _ in tracker_types]
        frame_count = 0
        
        from jetson_utils import cudaFromNumpy
        
        print("[INFO] Tracking started. Press 'q' to quit or close window.")
        
        while True:
            cuda_frame = camera.Capture()
            if cuda_frame is None:
                break
            
            if hasattr(cuda_frame, 'tocpu'):
                frame_np = cuda_frame.tocpu()
            elif hasattr(cuda_frame, 'as_numpy'):
                frame_np = cuda_frame.as_numpy()
            else:
                frame_np = np.array(cuda_frame)
            
            if not isinstance(frame_np, np.ndarray):
                frame_np = np.array(frame_np)
            
            frame_count += 1
            
            bboxes = []
            for i, (tracker, t_type) in enumerate(zip(trackers, tracker_types)):
                t_start = time.perf_counter()
                success, bbox = tracker.update(frame_np)
                t_end = time.perf_counter()
                frame_times[i].append(t_end - t_start)
                bboxes.append((success, bbox))
                
                if success:
                    x, y, w, h = map(int, bbox)
                    cv2.rectangle(frame_np, (x, y), (x+w, y+h), (0, 255, 0), 3)
                    cv2.putText(frame_np, t_type, (x, y-10), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)
            
            cuda_frame_with_bbox = cudaFromNumpy(frame_np)
            output.Render(cuda_frame_with_bbox)
            output.SetStatus(f"MIL Tracker | Frames: {frame_count}")
            
            if DEBUG_SAVE and frame_count % 30 == 0:
                output_path = f"{debug_dir}/track_{frame_count:04d}.jpg"
                cv2.imwrite(output_path, frame_np)
            
            if frame_count >= 300:
                print("[INFO] Reached 300 frames, stopping")
                break
        
        if frame_count % 30 == 0:
            print(f"[INFO] Processed {frame_count} frames, FPS: {1.0/np.mean(frame_times[-1]):.1f}")
        
        print("\n===== Tracker Performance =====")
        for i, t_type in enumerate(tracker_types):
            times = frame_times[i]
            if times:
                fps = 1.0 / np.mean(times)
                print(f"  {t_type}: {fps:.1f} FPS")
        print("================================")
        
        print("[INFO] Done.")
        return

    # Path 2: CSI camera with GStreamer pipeline (conda env, no jetson-inference)
    if is_camera and vid_path in ["csi://0", "0"] and GST_CAMERA_AVAILABLE:
        print("[INFO] Using GStreamer pipeline for CSI camera capture")
        
        cam = create_camera(width=1280, height=720, fps=30, sensor_id=0)
        debug_dir = "/home/bao-tg/Capstone/brain/AI/debug_raw"
        os.makedirs(debug_dir, exist_ok=True)
        
        print("[INFO] Waiting for camera to initialize...")
        time.sleep(2)
        
        ret, frame = cam.read()
        if not ret or frame is None:
            cam.release()
            raise RuntimeError("Failed to capture from CSI camera. Is Argus daemon running? Try: sudo systemctl restart nvargus-daemon")
        
        print(f"[INFO] Captured frame shape: {frame.shape}")
        
        if DEBUG_SAVE:
            cv2.imwrite(f"{debug_dir}/gst_csi_frame_raw.jpg", frame)
            b_mean = frame[:,:,0].mean()
            g_mean = frame[:,:,1].mean()
            r_mean = frame[:,:,2].mean()
            print(f"[DEBUG] Raw frame BGR means: B={b_mean:.1f}, G={g_mean:.1f}, R={r_mean:.1f}")
            is_green = (g_mean > b_mean * 2.5 and g_mean > r_mean * 2.5)
            if is_green:
                print("[WARN] Frame appears GREEN - color format issue detected!")
            else:
                print("[INFO] Frame colors look correct")
        
        if HEADLESS_MODE:
            roi_h, roi_w = frame.shape[:2]
            roi = (roi_w//4, roi_h//4, roi_w//2, roi_h//2)
            print(f"[INFO] Headless mode ROI: {roi}")
        else:
            try:
                roi = cv2.selectROI("Select Object to Track", frame, fromCenter=False, showCrosshair=True)
                cv2.destroyWindow("Select Object to Track")
                if roi == (0, 0, 0, 0):
                    roi_h, roi_w = frame.shape[:2]
                    roi = (roi_w//4, roi_h//4, roi_w//2, roi_h//2)
                    print(f"[INFO] Default center ROI: {roi}")
            except Exception as e:
                print(f"[WARN] Cannot create GUI: {e}")
                roi_h, roi_w = frame.shape[:2]
                roi = (roi_w//4, roi_h//4, roi_w//2, roi_h//2)
                print(f"[INFO] Default center ROI: {roi}")
        
        print(f"[INFO] ROI selected: {roi}")
        
        trackers = []
        for t_type in tracker_types:
            print(f"[INFO] Initializing {t_type} tracker...")
            tracker = initialize_tracker(t_type, frame, roi)
            trackers.append(tracker)
        
        frame_times = [[] for _ in tracker_types]
        frame_count = 0
        
        colors = [
            (0, 255, 0), (255, 0, 0), (0, 0, 255), (255, 255, 0),
            (255, 0, 255), (0, 255, 255), (128, 0, 255), (0, 128, 255),
        ]
        
        print("[INFO] Tracking started. Press 'q' to quit.")
        
        while True:
            ret, frame = cam.read()
            if not ret or frame is None:
                print("[WARN] Failed to read frame")
                break
            
            if DEBUG_SAVE and frame_count < 3:
                debug_dir = "/home/bao-tg/Capstone/brain/AI/debug_raw"
                cv2.imwrite(f"{debug_dir}/track_raw_{frame_count:04d}.jpg", frame)
            
            frame_count += 1
            display = frame.copy()
            
            for i, (tracker, t_type) in enumerate(zip(trackers, tracker_types)):
                t_start = time.perf_counter()
                success, bbox = tracker.update(frame)
                t_end = time.perf_counter()
                frame_times[i].append(t_end - t_start)
                
                color = colors[i % len(colors)]
                if success:
                    x, y, w, h = map(int, bbox)
                    cv2.rectangle(display, (x, y), (x + w, y + h), color, 2)
                    cv2.putText(display, t_type, (x, y - 8),
                                cv2.FONT_HERSHEY_SIMPLEX, 0.6, color, 2)
                else:
                    cv2.putText(display, f"{t_type}: LOST", (20, 40 + i * 25),
                                cv2.FONT_HERSHEY_SIMPLEX, 0.6, color, 2)
            
            cv2.putText(display, f"Frame: {frame_count}", (20, 20),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 255), 2)
            
            if HEADLESS_MODE:
                output_dir = "/home/bao-tg/Capstone/brain/AI/headless_output/tracker_output"
                os.makedirs(output_dir, exist_ok=True)
                cv2.imwrite(f"{output_dir}/frame_{frame_count:04d}.jpg", display)
            else:
                cv2.imshow("Multi-Tracker Comparison", display)
            
            if cv2.waitKey(1) & 0xFF == ord('q'):
                print("[INFO] Quit signal received.")
                break
        
        if not HEADLESS_MODE:
            cv2.destroyAllWindows()
        
        cam.release()
        
        print("\n===== Tracker Performance =====")
        for i, t_type in enumerate(tracker_types):
            times = frame_times[i]
            if times:
                avg_time = np.mean(times)
                fps = 1.0 / avg_time if avg_time > 0 else 0
                print(f"  {t_type:<12} | Avg time/frame: {avg_time*1000:.2f} ms | FPS: {fps:.1f}")
        print("================================\n")
        print("[INFO] Done.")
        return

    # Path 3: Original OpenCV path (video file or USB camera)
    if vid_path == "0" or vid_path.isdigit():
        cap = cv2.VideoCapture(0)
    else:
        cap = cv2.VideoCapture(vid_path)

    if not cap.isOpened():
        raise RuntimeError(f"Cannot open video: {vid_path}")

    roi, first_frame = select_object(cap, is_camera)
    print(f"[INFO] ROI selected: {roi}")

    track_object(cap, tracker_types, roi, first_frame, is_camera)

    cap.release()
    print("[INFO] Done.")


##############
if __name__ == "__main__":
    import sys
    print("chiiiii")
    # Usage: python lab04_p2.py [video_path] [tracker1 tracker2 ...]
    # Example: python Multi_tracker.py csi://0 MIL
    # Or with ROI: python Multi_tracker.py csi://0 MIL --roi 320 180 100 100
    if len(sys.argv) < 2:
        print("Usage: python Multi_tracker.py <video_path> [TRACKER1 TRACKER2 ...]")
        print("Available trackers: MIL, KCF, CSRT, MOSSE, GOTURN")
        print("Defaulting to: CSI camera with MIL")
        video = "csi://0"
        trackers = ["MIL"]
    else:
        print("hi")
        video = sys.argv[1]
        trackers = sys.argv[2:] if len(sys.argv) > 2 else ["MIL"]

    main(video, trackers)