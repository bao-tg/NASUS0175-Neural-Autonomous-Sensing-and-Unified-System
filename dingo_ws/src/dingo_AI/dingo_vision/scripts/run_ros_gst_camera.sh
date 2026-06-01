#!/bin/bash
set -e

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
VISION_CONDA_ENV="${VISION_CONDA_ENV:-phuc}"
ROS_DISTRO="${ROS_DISTRO:-noetic}"

if [ -f /opt/conda/etc/profile.d/conda.sh ]; then
    source /opt/conda/etc/profile.d/conda.sh
else
    eval "$(conda shell.bash hook)"
fi

conda activate "${VISION_CONDA_ENV}"
export PYTHONPATH="/opt/ros/${ROS_DISTRO}/lib/python3/dist-packages:/dingo_ws/devel/lib/python3/dist-packages:${SCRIPT_DIR}:${PYTHONPATH:-}"
export OPENBLAS_CORETYPE="${OPENBLAS_CORETYPE:-ARMV8}"

exec python "${SCRIPT_DIR}/ros_gst_camera.py" "$@"
