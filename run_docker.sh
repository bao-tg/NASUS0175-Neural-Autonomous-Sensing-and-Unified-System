#!/bin/bash
# chmod +x /home/bao-tg/Documents/DingoQuadruped/ros_entrypoint.sh
# source /dingo_ws/devel/setup.bash && sudo env PYTHONPATH=$PYTHONPATH OPENBLAS_CORETYPE=ARMV8 BLINKA_JETSON_NANO=1 python3 /dingo_ws/src/dingo/scripts/dingo_driver.py
# roslaunch dingo dingo.launch is_physical:=1 is_sim:=0 use_keyboard:=1 use_joystick:=0
# Configuration
IMAGE_NAME="dingo-ros:base"
CONTAINER_NAME="dingo_dev"
HOST_DIR="$HOME/Documents/DingoQuadruped/dingo_ws"
CONTAINER_DIR="/dingo_ws"
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"

# Fix for the display error
export DISPLAY=${DISPLAY:-:0}

# Grant docker permission to the X server
if [ -x "$(command -v xhost)" ]; then
    xhost +local:docker > /dev/null
fi

# Prepare Xauthority for sharing with container
XAUTH=/tmp/.docker.xauth
if [ -f "$HOME/.Xauthority" ]; then
    cp "$HOME/.Xauthority" "$XAUTH"
fi
touch "$XAUTH"
chmod 777 "$XAUTH"

# The Run Command
sudo docker run -it \
    --rm \
    --name "$CONTAINER_NAME" \
    --net=host \
    --privileged \
    --env="DISPLAY=$DISPLAY" \
    --env="QT_X11_NO_MITSHM=1" \
    --env="XAUTHORITY=$XAUTH" \
    --volume="/tmp/.X11-unix:/tmp/.X11-unix:rw" \
    --volume="$XAUTH:$XAUTH:rw" \
    --volume="$HOST_DIR/src:$CONTAINER_DIR/src" \
    --volume="$SCRIPT_DIR/ros_entrypoint.sh:/ros_entrypoint.sh:ro" \
    --entrypoint "/ros_entrypoint.sh" \
    "$IMAGE_NAME" \
    /bin/bash
