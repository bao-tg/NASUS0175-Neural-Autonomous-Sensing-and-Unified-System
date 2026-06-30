#!/bin/bash
set -e

IMAGE_NAME="${DINGO_IMAGE_NAME:-main:latest}"
CONTAINER_NAME="${DINGO_CONTAINER_NAME:-dingo_full_$$}"
HOST_DIR="${DINGO_HOST_WS:-$HOME/Documents/DingoQuadruped/dingo_ws}"
CONTAINER_DIR="/dingo_ws"
HOST_TEGRA_LIB_DIR="/usr/lib/aarch64-linux-gnu/tegra"
HOST_TEGRA_EGL_DIR="/usr/lib/aarch64-linux-gnu/tegra-egl"
HOST_GST_PLUGIN_DIR="/usr/lib/aarch64-linux-gnu/gstreamer-1.0"
CONTAINER_NVIDIA_GST_PLUGIN_DIR="/opt/nvidia-gstreamer-1.0"
HOST_TENSORRT_INCLUDE_DIR="/usr/include/aarch64-linux-gnu"
HOST_TENSORRT_LIB_DIR="/usr/lib/aarch64-linux-gnu"
HOST_CUDA_DIR="$(readlink -f /usr/local/cuda 2>/dev/null || true)"
HOST_YOLO_MODELS_DIR="${DINGO_HOST_MODELS:-$HOME/models}"
CONTAINER_YOLO_MODELS_DIR="${DINGO_CONTAINER_MODELS:-$HOME/models}"
JOYSTICK_DEV="${DINGO_JOYSTICK_DEV:-/dev/input/js0}"

if [ "$#" -eq 0 ]; then
    CONTAINER_CMD=(/bin/bash)
else
    CONTAINER_CMD=("$@")
fi

DOCKER_ARGS=()
DOCKER_X11_ARGS=()
DOCKER_RUNTIME_ARGS=()

add_dir_mount() {
    local source="$1"
    local target="$2"
    local mode="${3:-rw}"

    if [ -n "$source" ] && [ -d "$source" ]; then
        DOCKER_ARGS+=(--volume="$source:$target:$mode")
    else
        echo "[WARN] Skipping missing directory mount: $source" >&2
    fi
}

add_file_mount() {
    local source="$1"
    local target="$2"
    local mode="${3:-ro}"

    if [ -f "$source" ]; then
        DOCKER_ARGS+=(--volume="$source:$target:$mode")
    else
        echo "[WARN] Skipping missing file mount: $source" >&2
    fi
}

add_path_mount() {
    local source="$1"
    local target="$2"
    local mode="${3:-rw}"

    if [ -e "$source" ]; then
        DOCKER_ARGS+=(--volume="$source:$target:$mode")
    else
        echo "[WARN] Skipping missing path mount: $source" >&2
    fi
}

if [ -n "${DINGO_DOCKER_RUNTIME:-nvidia}" ]; then
    DOCKER_RUNTIME_ARGS+=(--runtime "${DINGO_DOCKER_RUNTIME:-nvidia}")
fi

export DISPLAY="${DISPLAY:-:0}"
if [ -n "${DISPLAY:-}" ] && [ -d /tmp/.X11-unix ]; then
    if command -v xhost >/dev/null 2>&1; then
        xhost +SI:localuser:root >/dev/null 2>&1 || \
            xhost +local:docker >/dev/null 2>&1 || \
            echo "[WARN] Could not authorize Docker for X11; GUI windows may not open." >&2
    fi

    XAUTH="/tmp/.docker.xauth.${UID}"
    XAUTH_SOURCE="${XAUTHORITY:-$HOME/.Xauthority}"
    if [ -e "$XAUTH" ] && { [ ! -f "$XAUTH" ] || [ ! -w "$XAUTH" ]; }; then
        echo "[WARN] $XAUTH is not writable; using a temporary Xauthority file." >&2
        XAUTH="$(mktemp "/tmp/.docker.xauth.${UID}.XXXXXX")"
    fi

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
    echo "[WARN] DISPLAY is not set or /tmp/.X11-unix is missing; GUI forwarding is disabled." >&2
fi

ensure_argus_socket() {
    if [ "${DINGO_SKIP_ARGUS_CHECK:-0}" = "1" ]; then
        return
    fi

    if [ -d /tmp/argus_socket ]; then
        echo "[WARN] Removing stale /tmp/argus_socket directory so nvargus-daemon can create its socket..." >&2
        sudo rm -rf /tmp/argus_socket
    fi

    if [ ! -S /tmp/argus_socket ]; then
        if command -v systemctl >/dev/null 2>&1 && systemctl list-unit-files nvargus-daemon.service >/dev/null 2>&1; then
            echo "[INFO] Starting nvargus-daemon.service on host..." >&2
            sudo systemctl daemon-reload
            sudo systemctl start nvargus-daemon.service
        fi
    fi

    if [ ! -S /tmp/argus_socket ]; then
        echo "[ERROR] nvargus-daemon did not create /tmp/argus_socket." >&2
        echo "[ERROR] Set DINGO_SKIP_ARGUS_CHECK=1 to run without the Jetson CSI camera socket." >&2
        exit 1
    fi
}

ensure_argus_socket

if [ "${DINGO_SKIP_JOYSTICK_CHECK:-0}" != "1" ] && [ ! -e "$JOYSTICK_DEV" ]; then
    echo "[WARN] $JOYSTICK_DEV was not found. Pair/connect the Bluetooth controller before starting ROS joystick control." >&2
fi

mkdir -p "$HOST_DIR/src" "$HOST_DIR/build" "$HOST_DIR/devel" "$HOST_YOLO_MODELS_DIR"

add_dir_mount "$HOST_TEGRA_LIB_DIR" "$HOST_TEGRA_LIB_DIR" ro
add_dir_mount "$HOST_TEGRA_EGL_DIR" "$HOST_TEGRA_EGL_DIR" ro
add_dir_mount "$HOST_GST_PLUGIN_DIR" "$CONTAINER_NVIDIA_GST_PLUGIN_DIR" ro
add_dir_mount "$HOST_TENSORRT_INCLUDE_DIR" /host_tensorrt_include ro
add_dir_mount "$HOST_CUDA_DIR" /host_cuda ro
add_dir_mount "$HOST_YOLO_MODELS_DIR" "$CONTAINER_YOLO_MODELS_DIR" ro
add_dir_mount "$HOST_DIR/src" "$CONTAINER_DIR/src" rw
add_dir_mount "$HOST_DIR/build" "$CONTAINER_DIR/build" rw
add_dir_mount "$HOST_DIR/devel" "$CONTAINER_DIR/devel" rw
add_dir_mount /dev /dev rw
add_path_mount /tmp/argus_socket /tmp/argus_socket rw
add_file_mount /etc/timezone /etc/timezone ro
add_file_mount /etc/localtime /etc/localtime ro

add_file_mount "$HOST_TENSORRT_LIB_DIR/libnvinfer.so.8.2.1" /usr/lib/aarch64-linux-gnu/libnvinfer.so.8.2.1 ro
add_file_mount "$HOST_TENSORRT_LIB_DIR/libnvinfer_plugin.so.8.2.1" /usr/lib/aarch64-linux-gnu/libnvinfer_plugin.so.8.2.1 ro
add_file_mount "$HOST_TENSORRT_LIB_DIR/libcudnn.so.8.2.1" /usr/lib/aarch64-linux-gnu/libcudnn.so.8.2.1 ro
add_file_mount "$HOST_TENSORRT_LIB_DIR/libcudnn_adv_infer.so.8.2.1" /usr/lib/aarch64-linux-gnu/libcudnn_adv_infer.so.8.2.1 ro
add_file_mount "$HOST_TENSORRT_LIB_DIR/libcudnn_cnn_infer.so.8.2.1" /usr/lib/aarch64-linux-gnu/libcudnn_cnn_infer.so.8.2.1 ro
add_file_mount "$HOST_TENSORRT_LIB_DIR/libcudnn_ops_infer.so.8.2.1" /usr/lib/aarch64-linux-gnu/libcudnn_ops_infer.so.8.2.1 ro

if [ -n "$HOST_CUDA_DIR" ]; then
    add_file_mount "$HOST_CUDA_DIR/targets/aarch64-linux/lib/libcublas.so.10.2.3.300" /usr/lib/aarch64-linux-gnu/libcublas.so.10.2.3.300 ro
    add_file_mount "$HOST_CUDA_DIR/targets/aarch64-linux/lib/libcublasLt.so.10.2.3.300" /usr/lib/aarch64-linux-gnu/libcublasLt.so.10.2.3.300 ro
fi

CONTAINER_TENSORRT_FIX='
set -e

cd /usr/lib/aarch64-linux-gnu

echo "[INFO] Creating TensorRT / CUDA / cuDNN runtime symlinks..."

[ -e libnvinfer.so.8.2.1 ] && ln -sf libnvinfer.so.8.2.1 libnvinfer.so.8
[ -e libnvinfer_plugin.so.8.2.1 ] && ln -sf libnvinfer_plugin.so.8.2.1 libnvinfer_plugin.so.8

[ -e libcudnn.so.8.2.1 ] && ln -sf libcudnn.so.8.2.1 libcudnn.so.8
[ -e libcudnn_adv_infer.so.8.2.1 ] && ln -sf libcudnn_adv_infer.so.8.2.1 libcudnn_adv_infer.so.8
[ -e libcudnn_cnn_infer.so.8.2.1 ] && ln -sf libcudnn_cnn_infer.so.8.2.1 libcudnn_cnn_infer.so.8
[ -e libcudnn_ops_infer.so.8.2.1 ] && ln -sf libcudnn_ops_infer.so.8.2.1 libcudnn_ops_infer.so.8

[ -e libcublas.so.10.2.3.300 ] && ln -sf libcublas.so.10.2.3.300 libcublas.so.10
[ -e libcublasLt.so.10.2.3.300 ] && ln -sf libcublasLt.so.10.2.3.300 libcublasLt.so.10

ldconfig

export LD_LIBRARY_PATH=/usr/lib/aarch64-linux-gnu:${LD_LIBRARY_PATH:-}

echo "[INFO] Checking unresolved dependencies for libyolo_trt_nodelet.so..."
if [ -f /dingo_ws/devel/lib/libyolo_trt_nodelet.so ]; then
    ldd /dingo_ws/devel/lib/libyolo_trt_nodelet.so | grep "not found" || echo "[INFO] No missing dependencies found."
else
    echo "[WARN] /dingo_ws/devel/lib/libyolo_trt_nodelet.so not found. Skipping ldd check."
fi

cd ~/

exec "$@"
'

sudo docker run -it \
    --rm \
    --name "$CONTAINER_NAME" \
    --net=host \
    --pid=host \
    --privileged \
    --device /dev/snd:/dev/snd \
    --group-add audio \
    "${DOCKER_RUNTIME_ARGS[@]}" \
    "${DOCKER_X11_ARGS[@]}" \
    "${DOCKER_ARGS[@]}" \
    --env="OPENBLAS_CORETYPE=ARMV8" \
    --env="BLINKA_JETSON_NANO=1" \
    --env="OPENAI_API_KEY=${OPENAI_API_KEY:-}" \
    --env="OPENAI_API_BASE=${OPENAI_API_BASE:-https://api.openai.com/v1}" \
    --env="LD_LIBRARY_PATH=/host_cuda/targets/aarch64-linux/lib:/host_cuda/lib64:$HOST_TEGRA_EGL_DIR:$HOST_TEGRA_LIB_DIR:/usr/lib/aarch64-linux-gnu:${LD_LIBRARY_PATH:-}" \
    --env="__EGL_VENDOR_LIBRARY_FILENAMES=$HOST_TEGRA_EGL_DIR/nvidia.json" \
    --env="GST_PLUGIN_PATH=$CONTAINER_NVIDIA_GST_PLUGIN_DIR:${GST_PLUGIN_PATH:-}" \
    --env="GST_REGISTRY=/tmp/gst-registry-dingo-vision.bin" \
    --entrypoint /ros_entrypoint.sh \
    "$IMAGE_NAME" \
    /bin/bash -lc "$CONTAINER_TENSORRT_FIX" dingo-container-cmd "${CONTAINER_CMD[@]}"
