#!/bin/bash
set -e

IMAGE_NAME="dingo-vision:latest"
CONTAINER_NAME="${DINGO_VISION_CONTAINER_NAME:-dingo_vision_$$}"
HOST_DIR="$HOME/Documents/DingoQuadruped/dingo_ws"
CONTAINER_DIR="/dingo_ws"
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
HOST_TEGRA_LIB_DIR="/usr/lib/aarch64-linux-gnu/tegra"
HOST_TEGRA_EGL_DIR="/usr/lib/aarch64-linux-gnu/tegra-egl"
HOST_GST_PLUGIN_DIR="/usr/lib/aarch64-linux-gnu/gstreamer-1.0"
CONTAINER_NVIDIA_GST_PLUGIN_DIR="/opt/nvidia-gstreamer-1.0"
HOST_TENSORRT_INCLUDE_DIR="/usr/include/aarch64-linux-gnu"
HOST_TENSORRT_LIB_DIR="/usr/lib/aarch64-linux-gnu"
HOST_CUDA_DIR="/usr/local/cuda"
CONTAINER_TENSORRT_INCLUDE_DIR="/host_tensorrt_include"
CONTAINER_TENSORRT_LIB_DIR="/host_tensorrt_lib"
CONTAINER_CUDA_DIR="/host_cuda"
HOST_YOLO_MODELS_DIR="$HOME/models"
CONTAINER_YOLO_MODELS_DIR="$HOME/models"

if [ "$#" -eq 0 ]; then
    CONTAINER_CMD=(/bin/bash)
else
    CONTAINER_CMD=("$@")
fi

DOCKER_X11_ARGS=()
if [ -n "${DISPLAY:-}" ] && [ -d /tmp/.X11-unix ]; then
    if command -v xhost >/dev/null 2>&1; then
        xhost +SI:localuser:root >/dev/null 2>&1 || \
            echo "Warning: could not authorize root for X11; GUI windows may not open." >&2
    fi

    XAUTH=/tmp/.docker.xauth
    XAUTH_SOURCE="${XAUTHORITY:-$HOME/.Xauthority}"
    if [ -f "$XAUTH_SOURCE" ]; then
        cp "$XAUTH_SOURCE" "$XAUTH"
    else
        touch "$XAUTH"
    fi
    chmod 600 "$XAUTH"

    DOCKER_X11_ARGS+=(
        --env="DISPLAY=$DISPLAY"
        --env="QT_X11_NO_MITSHM=1"
        --env="XAUTHORITY=$XAUTH"
        --volume="/tmp/.X11-unix:/tmp/.X11-unix:rw"
        --volume="$XAUTH:$XAUTH:rw"
    )
else
    echo "DISPLAY is not set; running container without X11 GUI forwarding." >&2
fi

mkdir -p "$HOST_DIR/build" "$HOST_DIR/devel"
mkdir -p "$HOST_YOLO_MODELS_DIR"

sudo docker run -it \
    --rm \
    --name "$CONTAINER_NAME" \
    --net=host \
    --pid=host \
    --privileged \
    "${DOCKER_X11_ARGS[@]}" \
    --env="OPENBLAS_CORETYPE=ARMV8" \
    --env="BLINKA_JETSON_NANO=1" \
    --env="LD_LIBRARY_PATH=$HOST_TEGRA_EGL_DIR:$HOST_TEGRA_LIB_DIR:${LD_LIBRARY_PATH:-}" \
    --env="__EGL_VENDOR_LIBRARY_FILENAMES=$HOST_TEGRA_EGL_DIR/nvidia.json" \
    --env="GST_PLUGIN_PATH=$CONTAINER_NVIDIA_GST_PLUGIN_DIR:${GST_PLUGIN_PATH:-}" \
    --env="GST_REGISTRY=/tmp/gst-registry-dingo-vision.bin" \
    --volume="/tmp/argus_socket:/tmp/argus_socket" \
    --volume="$HOST_TEGRA_LIB_DIR:$HOST_TEGRA_LIB_DIR:ro" \
    --volume="$HOST_TEGRA_EGL_DIR:$HOST_TEGRA_EGL_DIR:ro" \
    --volume="$HOST_GST_PLUGIN_DIR:$CONTAINER_NVIDIA_GST_PLUGIN_DIR:ro" \
    --volume="$HOST_TENSORRT_INCLUDE_DIR:$CONTAINER_TENSORRT_INCLUDE_DIR:ro" \
    --volume="$HOST_TENSORRT_LIB_DIR/libnvinfer.so.8.2.1:/usr/lib/aarch64-linux-gnu/libnvinfer.so.8.2.1:ro" \
    --volume="$HOST_TENSORRT_LIB_DIR/libnvinfer_plugin.so.8.2.1:/usr/lib/aarch64-linux-gnu/libnvinfer_plugin.so.8.2.1:ro" \
    --volume="$HOST_TENSORRT_LIB_DIR/libcudnn.so.8.2.1:/usr/lib/aarch64-linux-gnu/libcudnn.so.8.2.1:ro" \
    --volume="$HOST_TENSORRT_LIB_DIR/libcudnn_adv_infer.so.8.2.1:/usr/lib/aarch64-linux-gnu/libcudnn_adv_infer.so.8.2.1:ro" \
    --volume="$HOST_TENSORRT_LIB_DIR/libcudnn_cnn_infer.so.8.2.1:/usr/lib/aarch64-linux-gnu/libcudnn_cnn_infer.so.8.2.1:ro" \
    --volume="$HOST_TENSORRT_LIB_DIR/libcudnn_ops_infer.so.8.2.1:/usr/lib/aarch64-linux-gnu/libcudnn_ops_infer.so.8.2.1:ro" \
    --volume="$HOST_CUDA_DIR:$CONTAINER_CUDA_DIR:ro" \
    --volume="$HOST_YOLO_MODELS_DIR:$CONTAINER_YOLO_MODELS_DIR:ro" \
    --volume="$HOST_DIR/src:$CONTAINER_DIR/src" \
    --volume="$HOST_DIR/build:$CONTAINER_DIR/build:rw" \
    --volume="$HOST_DIR/devel:$CONTAINER_DIR/devel:rw" \
    --volume="$SCRIPT_DIR/ros_vision_entrypoint.sh:/ros_vision_entrypoint.sh:ro" \
    --entrypoint "/ros_vision_entrypoint.sh" \
    "$IMAGE_NAME" \
    "${CONTAINER_CMD[@]}"
