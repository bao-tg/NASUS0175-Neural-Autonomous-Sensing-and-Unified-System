# Dingo Camera Host

Host-side CSI camera adapter for Jetson Nano JetPack 4.x. Run this on the Jetson host, outside the Ubuntu 20.04 ROS Noetic container, because `nvarguscamerasrc` depends on native L4T camera libraries.

## Start the service

```bash
cd ~/Documents/DingoQuadruped/dingo_ws/src/dingo_camera_host
python3 scripts/csi_capture_server.py --host 127.0.0.1 --port 8081 --backend gst-launch --print-pipeline
```

## Test one frame on the host

```bash
python3 scripts/test_capture.py --output /tmp/dingo_scene_test.jpg
ls -lh /tmp/dingo_scene_test.jpg
```

## Use from the container

The Docker run script uses host networking, so the container can request:

```text
http://127.0.0.1:8081/capture.jpg
```

Then launch the voice stack with `camera_source:=http`.

## Troubleshooting

If the server cannot open the camera, test native GStreamer on the host first:

```bash
gst-inspect-1.0 nvarguscamerasrc
gst-launch-1.0 nvarguscamerasrc num-buffers=1 ! \
  'video/x-raw(memory:NVMM),width=1280,height=720,framerate=30/1' ! \
  nvvidconv ! 'video/x-raw,format=BGRx' ! videoconvert ! jpegenc ! \
  filesink location=/tmp/dingo_csi_test.jpg
ls -lh /tmp/dingo_csi_test.jpg
```

If that fails, restart the Argus daemon on the Jetson host:

```bash
sudo systemctl restart nvargus-daemon
```

OpenCV capture is still available for comparison:

```bash
python3 scripts/csi_capture_server.py --backend opencv --print-pipeline
```
