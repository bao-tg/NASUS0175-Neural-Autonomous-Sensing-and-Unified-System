#!/bin/bash
set -e

VISION_CONDA_ENV="${VISION_CONDA_ENV:-phuc}"
ROS_DISTRO="${ROS_DISTRO:-noetic}"
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
YOLO_NODE="${SCRIPT_DIR}/yolo_tracker_node.py"

if command -v conda >/dev/null 2>&1; then
    eval "$(conda shell.bash hook)"
elif [ -f /opt/conda/etc/profile.d/conda.sh ]; then
    source /opt/conda/etc/profile.d/conda.sh
elif [ -f "$HOME/miniforge3/etc/profile.d/conda.sh" ]; then
    source "$HOME/miniforge3/etc/profile.d/conda.sh"
elif [ -f /home/bao-tg/miniforge3/etc/profile.d/conda.sh ]; then
    source /home/bao-tg/miniforge3/etc/profile.d/conda.sh
else
    echo "[ERROR] conda was not found. Expected /opt/conda in dingo-vision or host Miniforge."
    exit 1
fi

conda activate "${VISION_CONDA_ENV}"

export PYTHONPATH="/opt/ros/${ROS_DISTRO}/lib/python3/dist-packages:/dingo_ws/devel/lib/python3/dist-packages:${SCRIPT_DIR}:${PYTHONPATH}"
export OPENBLAS_CORETYPE="${OPENBLAS_CORETYPE:-ARMV8}"

exec python "${YOLO_NODE}" "$@"
