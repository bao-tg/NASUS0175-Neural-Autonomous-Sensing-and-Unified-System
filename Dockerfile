FROM arm64v8/ros:noetic-ros-base-focal

SHELL ["/bin/bash", "-o", "pipefail", "-c"]

ENV DEBIAN_FRONTEND=noninteractive
ENV CONDA_DIR=/opt/conda
ENV VISION_CONDA_ENV=dev
ENV LANG=C.UTF-8
ENV LC_ALL=C.UTF-8

RUN apt-get update && apt-get install -y --no-install-recommends \
    apt-utils \
    alsa-utils \
    build-essential \
    bzip2 \
    ca-certificates \
    curl \
    gdb \
    git \
    gstreamer1.0-libav \
    gstreamer1.0-plugins-bad \
    gstreamer1.0-plugins-base \
    gstreamer1.0-plugins-good \
    gstreamer1.0-tools \
    i2c-tools \
    libgl1 \
    libglib2.0-0 \
    libgstreamer-plugins-base1.0-dev \
    libgstreamer1.0-dev \
    libsm6 \
    libvlc-dev \
    libvlccore-dev \
    libxext6 \
    libxrender1 \
    nano \
    pkg-config \
    procps \
    python3-catkin-tools \
    python3-opencv \
    python3-pip \
    python3-pymodbus \
    python3-rosdep \
    python3-vcstool \
    ros-noetic-camera-calibration-parsers \
    ros-noetic-camera-info-manager \
    ros-noetic-catkin \
    ros-noetic-cv-bridge \
    ros-noetic-image-transport \
    ros-noetic-joy \
    ros-noetic-nodelet \
    ros-noetic-plotjuggler-ros \
    ros-noetic-ros-controllers \
    ros-noetic-rosserial \
    ros-noetic-rosserial-arduino \
    ros-noetic-rqt-image-view \
    ros-noetic-soem \
    && groupadd -f i2c \
    && groupadd -f ros \
    && usermod -aG i2c root \
    && usermod -aG ros root \
    && rm -rf /var/lib/apt/lists/*

RUN pip3 uninstall -y serial 2>/dev/null || true

RUN pip3 install --no-cache-dir \
    UDPComms \
    Jetson.GPIO \
    adafruit-blinka \
    adafruit-circuitpython-bno055 \
    adafruit-circuitpython-servokit \
    matplotlib \
    numpy==1.24.4 \
    openai==0.28 \
    pigpio \
    pynput \
    pyserial \
    regex \
    spidev \
    transforms3d

RUN curl -fsSL -o /tmp/miniforge.sh \
    https://github.com/conda-forge/miniforge/releases/latest/download/Miniforge3-Linux-aarch64.sh \
    && bash /tmp/miniforge.sh -b -p "${CONDA_DIR}" \
    && rm -f /tmp/miniforge.sh

RUN source "${CONDA_DIR}/etc/profile.d/conda.sh" \
    && conda create -y -n "${VISION_CONDA_ENV}" -c conda-forge \
        python=3.10 \
        pip \
        pyyaml \
        rospkg \
        catkin_pkg \
    && conda run -n "${VISION_CONDA_ENV}" pip install --no-cache-dir \
        numpy==2.2.6 \
        opencv-contrib-python==4.13.0.92 \
        pillow \
        matplotlib \
    && conda clean -afy

WORKDIR /dingo_ws
COPY dingo_ws/src /dingo_ws/src

RUN rosdep update \
    && rosdep install --rosdistro noetic --from-paths src --ignore-src -r -y \
    --skip-keys="time socket enum logging os numpy matplotlib transforms3d pyserial spidev pigpio regex gazebo gazebo11"

RUN /bin/bash -c 'source /opt/ros/${ROS_DISTRO}/setup.bash \
    && if [ -d /dingo_ws/src/dingo_AI/yolox ]; then touch /dingo_ws/src/dingo_AI/yolox/CATKIN_IGNORE; fi \
    && catkin_make --directory /dingo_ws -DCMAKE_BUILD_TYPE=Release \
    && rm -f /dingo_ws/src/dingo_AI/yolox/CATKIN_IGNORE'

RUN echo "source /opt/ros/\$ROS_DISTRO/setup.bash" >> /etc/bash.bashrc \
    && echo "source /dingo_ws/devel/setup.bash" >> /etc/bash.bashrc \
    && echo 'source /opt/conda/etc/profile.d/conda.sh 2>/dev/null || true' > /etc/profile.d/dingo_vision.sh \
    && echo "PS1='\${debian_chroot:+(\$debian_chroot)}\u@:\w\$ '" >> /etc/bash.bashrc \
    && echo "exec sg ros bash" >> /root/.bashrc

COPY ros_entrypoint.sh /ros_entrypoint.sh
RUN chmod +x /ros_entrypoint.sh

ENTRYPOINT ["/ros_entrypoint.sh"]
CMD ["bash"]
