#!/bin/bash
#  chmod +x /home/bao-tg/Documents/DingoQuadruped/ros_entrypoint.sh
# Fix: uninstall conflicting 'serial' pip package that shadows 'pyserial'
# The 'serial' pip package is a serialization library, NOT for serial ports
# 'pyserial' provides the actual 'serial' module with Serial class

# Fix: create SPI device nodes if they don't exist (for LCD/spidev)
if [ ! -c /dev/spidev0.0 ]; then
    mknod /dev/spidev0.0 c 153 0 2>/dev/null || true
    chmod 666 /dev/spidev0.0 2>/dev/null || true
fi
if [ ! -c /dev/spidev0.1 ]; then
    mknod /dev/spidev0.1 c 153 1 2>/dev/null || true
    chmod 666 /dev/spidev0.1 2>/dev/null || true
fi

# setup ros environment
source "/opt/ros/$ROS_DISTRO/setup.bash"
if [ -f "/dingo_ws/devel/setup.bash" ]; then
    source "/dingo_ws/devel/setup.bash"
fi
exec "$@"
