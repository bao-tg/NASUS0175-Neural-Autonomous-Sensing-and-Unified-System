FROM dingo-ros:base

SHELL ["/bin/bash", "-o", "pipefail", "-c"]

ENV DEBIAN_FRONTEND=noninteractive
ENV CONDA_DIR=/opt/conda
ENV VISION_CONDA_ENV=phuc

RUN apt-get update && apt-get install -y --no-install-recommends \
    alsa-utils \
    bzip2 \
    ca-certificates \
    curl \
    pkg-config \
    procps \
    python3-pip \
    gstreamer1.0-tools \
    gstreamer1.0-plugins-base \
    gstreamer1.0-plugins-good \
    gstreamer1.0-plugins-bad \
    gstreamer1.0-libav \
    libgstreamer1.0-dev \
    libgstreamer-plugins-base1.0-dev \
    libgl1 \
    libglib2.0-0 \
    libsm6 \
    libxext6 \
    libxrender1 \
    ros-noetic-camera-calibration-parsers \
    ros-noetic-camera-info-manager \
    ros-noetic-cv-bridge \
    ros-noetic-image-transport \
    ros-noetic-nodelet \
    ros-noetic-rqt-image-view \
    && rm -rf /var/lib/apt/lists/*

RUN pip3 install --no-cache-dir \
    Jetson.GPIO \
    adafruit-blinka \
    adafruit-circuitpython-servokit \
    adafruit-circuitpython-bno055 \
    openai==0.28

RUN curl -fsSL -o /tmp/miniforge.sh \
    https://github.com/conda-forge/miniforge/releases/latest/download/Miniforge3-Linux-aarch64.sh \
    && bash /tmp/miniforge.sh -b -p "${CONDA_DIR}" \
    && rm -f /tmp/miniforge.sh

COPY docker/environment-phuc.yml /tmp/environment-phuc.yml
RUN source "${CONDA_DIR}/etc/profile.d/conda.sh" \
    && conda env create -f /tmp/environment-phuc.yml \
    && conda clean -afy \
    && rm -f /tmp/environment-phuc.yml

COPY dingo_ws/src /dingo_ws/src
RUN source "/opt/ros/${ROS_DISTRO}/setup.bash" \
    && catkin_make --directory /dingo_ws -DCMAKE_BUILD_TYPE=Release

COPY docker/ros_vision_entrypoint.sh /ros_vision_entrypoint.sh
RUN chmod +x /ros_vision_entrypoint.sh \
    && echo 'source /opt/conda/etc/profile.d/conda.sh 2>/dev/null || true' > /etc/profile.d/dingo_vision.sh

ENTRYPOINT ["/ros_vision_entrypoint.sh"]
CMD ["bash"]
