#!/bin/bash
set -e

# Runtime debug fixes. Keep these here while iterating so the image does not need
# a full rebuild for small Python dependency changes.
if ! command -v arecord >/dev/null 2>&1; then
    echo "[ERROR] arecord is missing. Rebuild dingo-vision:latest so Dockerfile installs alsa-utils." >&2
    exit 1
fi

pip3 uninstall -y serial 2>/dev/null || true
# pip3 install --no-cache-dir \
#     Jetson.GPIO \
#     adafruit-blinka \
#     adafruit-circuitpython-servokit \
#     adafruit-circuitpython-bno055


if [ -f /usr/lib/aarch64-linux-gnu/libnvinfer.so.8.2.1 ]; then
    ln -sf /usr/lib/aarch64-linux-gnu/libnvinfer.so.8.2.1 /usr/lib/aarch64-linux-gnu/libnvinfer.so.8
    ln -sf /usr/lib/aarch64-linux-gnu/libnvinfer.so.8.2.1 /usr/lib/aarch64-linux-gnu/libnvinfer.so
fi
if [ -f /usr/lib/aarch64-linux-gnu/libnvinfer_plugin.so.8.2.1 ]; then
    ln -sf /usr/lib/aarch64-linux-gnu/libnvinfer_plugin.so.8.2.1 /usr/lib/aarch64-linux-gnu/libnvinfer_plugin.so.8
    ln -sf /usr/lib/aarch64-linux-gnu/libnvinfer_plugin.so.8.2.1 /usr/lib/aarch64-linux-gnu/libnvinfer_plugin.so
fi


if [ -f /usr/lib/aarch64-linux-gnu/libcudnn.so.8.2.1 ]; then
    ln -sf /usr/lib/aarch64-linux-gnu/libcudnn.so.8.2.1 /usr/lib/aarch64-linux-gnu/libcudnn.so.8
    ln -sf /usr/lib/aarch64-linux-gnu/libcudnn.so.8.2.1 /usr/lib/aarch64-linux-gnu/libcudnn.so
fi


for lib in libcudnn_ops_infer libcudnn_cnn_infer libcudnn_adv_infer; do
    if [ -f "/usr/lib/aarch64-linux-gnu/${lib}.so.8.2.1" ]; then
        ln -sf "/usr/lib/aarch64-linux-gnu/${lib}.so.8.2.1" "/usr/lib/aarch64-linux-gnu/${lib}.so.8"
        ln -sf "/usr/lib/aarch64-linux-gnu/${lib}.so.8.2.1" "/usr/lib/aarch64-linux-gnu/${lib}.so"
    fi
done

for lib in libcublas libcublasLt; do
    if [ -f "/usr/lib/aarch64-linux-gnu/${lib}.so.10.2.3.300" ]; then
        ln -sf "/usr/lib/aarch64-linux-gnu/${lib}.so.10.2.3.300" "/usr/lib/aarch64-linux-gnu/${lib}.so.10"
        ln -sf "/usr/lib/aarch64-linux-gnu/${lib}.so.10.2.3.300" "/usr/lib/aarch64-linux-gnu/${lib}.so"
    fi
done

export LD_LIBRARY_PATH="/host_cuda/targets/aarch64-linux/lib:/host_cuda/lib64:/usr/lib/aarch64-linux-gnu:${LD_LIBRARY_PATH:-}"

if [ ! -c /dev/spidev0.0 ]; then
    mknod /dev/spidev0.0 c 153 0 2>/dev/null || true
    chmod 666 /dev/spidev0.0 2>/dev/null || true
fi

if [ ! -c /dev/spidev0.1 ]; then
    mknod /dev/spidev0.1 c 153 1 2>/dev/null || true
    chmod 666 /dev/spidev0.1 2>/dev/null || true
fi

source "/opt/ros/${ROS_DISTRO}/setup.bash"

if [ "${DINGO_SKIP_RUNTIME_BUILD:-0}" != "1" ]     && [ -f /dingo_ws/src/dingo_AI/yolox/src/yolo_trt_node.cpp ]     && [ ! -x /dingo_ws/devel/lib/yolox/yolo_trt_node ]; then
    echo "[INFO] Building runtime ROS workspace for yolo_trt_node..."
    catkin_make --directory /dingo_ws -DCMAKE_BUILD_TYPE=Release
fi


if [ -f /dingo_ws/devel/setup.bash ]; then
    source /dingo_ws/devel/setup.bash
fi

if [ -f /opt/conda/etc/profile.d/conda.sh ]; then
    source /opt/conda/etc/profile.d/conda.sh
fi

export OPENBLAS_CORETYPE="${OPENBLAS_CORETYPE:-ARMV8}"
export BLINKA_JETSON_NANO="${BLINKA_JETSON_NANO:-1}"

exec "$@"
