#!/bin/bash
# chmod +x /home/bao-tg/Documents/DingoQuadruped/ros_entrypoint.sh
# source /dingo_ws/devel/setup.bash && sudo env PYTHONPATH=$PYTHONPATH OPENBLAS_CORETYPE=ARMV8 BLINKA_JETSON_NANO=1 python3 /dingo_ws/src/dingo/scripts/dingo_driver.py
# roslaunch dingo dingo.launch is_physical:=1 is_sim:=0 use_keyboard:=1 use_joystick:=0
# Configuration
IMAGE_NAME="dingo-ros:base"
CONTAINER_NAME="dingo_devdan_test10"
HOST_DIR="$HOME/Documents/DingoQuadruped/dingo_ws"
CONTAINER_DIR="/dingo_ws"
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"

# Fix for the display error
export DISPLAY=${DISPLAY:-:0}

# Grant docker permission to the X server
if [ -x "$(command -v xhost)" ]; then
    xhost +local:docker > /dev/null
fi

# Prepare Xauthority for sharing with container.
# Use a per-user file so a stale root-owned /tmp/.docker.xauth cannot block startup.
XAUTH="/tmp/.docker.xauth.${UID}"
if [ -e "$XAUTH" ] && { [ ! -f "$XAUTH" ] || [ ! -w "$XAUTH" ]; }; then
    echo "Warning: $XAUTH is not a writable file; using a temporary Xauthority file instead."
    XAUTH="$(mktemp "/tmp/.docker.xauth.${UID}.XXXXXX")"
fi
if [ -f "$HOME/.Xauthority" ]; then
    cp "$HOME/.Xauthority" "$XAUTH"
else
    touch "$XAUTH"
fi
chmod 600 "$XAUTH"

# # The Run Command
sudo docker run -it \
    --runtime nvidia \
    --name "$CONTAINER_NAME" \
    --net=host \
    --privileged \
    --device /dev/snd:/dev/snd \
    --group-add audio \
    --env="DISPLAY=$DISPLAY" \
    --env="QT_X11_NO_MITSHM=1" \
    --env="XAUTHORITY=$XAUTH" \
    --volume="/tmp/.X11-unix:/tmp/.X11-unix:rw" \
    --volume="/tmp/argus_socket:/tmp/argus_socket" \
    --volume="$XAUTH:$XAUTH:rw" \
    --volume="$HOST_DIR/src:$CONTAINER_DIR/src" \
    --volume="$SCRIPT_DIR/ros_entrypoint.sh:/ros_entrypoint.sh:ro" \
    --entrypoint "/ros_entrypoint.sh" \
    "$IMAGE_NAME" \
    /bin/bash
# sudo docker run -it \
#     --rm \
#     --name "$CONTAINER_NAME" \
#     --net=host \
#     --privileged \
#     --device /dev/snd:/dev/snd \
#     --group-add audio \
#     --env="DISPLAY=$DISPLAY" \
#     --env="QT_X11_NO_MITSHM=1" \
#     --env="XAUTHORITY=$XAUTH" \
#     --volume="/tmp/.X11-unix:/tmp/.X11-unix:rw" \
#     --volume="$XAUTH:$XAUTH:rw" \
#     --volume="$HOST_DIR/src:$CONTAINER_DIR/src" \
#     --volume="$SCRIPT_DIR/ros_entrypoint.sh:/ros_entrypoint.sh:ro" \
#     --entrypoint "/ros_entrypoint.sh" \
#     "$IMAGE_NAME" \
#     /bin/bash
