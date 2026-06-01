#!/bin/bash
# Run Multi_tracker.py with CSI camera in the phuc conda environment
#
# Prerequisites:
#   1. Argus daemon must be running:
#      sudo systemctl restart nvargus-daemon
#   2. CSI camera (IMX219) must be connected
#
# Usage:
#   bash run_tracker.sh              # Default: MIL tracker
#   bash run_tracker.sh KCF          # KCF tracker
#   bash run_tracker.sh MIL KCF CSRT # Multiple trackers

set -e

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
TRACKER_SCRIPT="${SCRIPT_DIR}/multi_tracker.py"
TRACKERS="${@:-MIL}"
VISION_CONDA_ENV="${VISION_CONDA_ENV:-phuc}"

echo "=============================================="
echo "Running Multi_tracker.py in phuc conda env"
echo "Tracker(s): ${TRACKERS}"
echo "=============================================="

if ! pgrep -x nvargus-daemon > /dev/null; then
    echo "[ERROR] nvargus-daemon is not running!"
    echo "        Start it with: sudo systemctl restart nvargus-daemon"
    exit 1
fi

if [ ! -e /dev/video0 ]; then
    echo "[ERROR] /dev/video0 not found! Is the CSI camera connected?"
    exit 1
fi

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

export PYTHONPATH="${SCRIPT_DIR}:${PYTHONPATH}"
export OPENBLAS_CORETYPE="${OPENBLAS_CORETYPE:-ARMV8}"

echo "[INFO] Python: $(which python)"
echo "[INFO] OpenCV: $(python -c 'import cv2; print(cv2.__version__)' 2>/dev/null || echo 'not found')"
echo ""

python "${TRACKER_SCRIPT}" csi://0 ${TRACKERS}
