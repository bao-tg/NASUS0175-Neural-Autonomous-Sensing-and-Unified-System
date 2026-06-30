#!/bin/bash
set -e

pip3 uninstall -y serial 2>/dev/null || true

link_versioned_library() {
    local full_path="$1"
    local soname="$2"
    local unversioned="$3"

    if [ -f "$full_path" ]; then
        ln -sf "$full_path" "/usr/lib/aarch64-linux-gnu/$soname"
        ln -sf "$full_path" "/usr/lib/aarch64-linux-gnu/$unversioned"
    fi
}

link_versioned_library /usr/lib/aarch64-linux-gnu/libnvinfer.so.8.2.1 libnvinfer.so.8 libnvinfer.so
link_versioned_library /usr/lib/aarch64-linux-gnu/libnvinfer_plugin.so.8.2.1 libnvinfer_plugin.so.8 libnvinfer_plugin.so
link_versioned_library /usr/lib/aarch64-linux-gnu/libcudnn.so.8.2.1 libcudnn.so.8 libcudnn.so

for lib in libcudnn_ops_infer libcudnn_cnn_infer libcudnn_adv_infer; do
    link_versioned_library "/usr/lib/aarch64-linux-gnu/${lib}.so.8.2.1" "${lib}.so.8" "${lib}.so"
done

for lib in libcublas libcublasLt; do
    link_versioned_library "/usr/lib/aarch64-linux-gnu/${lib}.so.10.2.3.300" "${lib}.so.10" "${lib}.so"
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

if [ "${DINGO_SKIP_RUNTIME_BUILD:-0}" != "1" ] \
    && [ -f /dingo_ws/src/dingo_AI/yolox/src/yolo_trt_node.cpp ] \
    && [ ! -x /dingo_ws/devel/lib/yolox/yolo_trt_node ]; then
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
