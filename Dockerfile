FROM arm64v8/ubuntu:20.04

FROM arm64v8/ros:noetic-ros-base-focal

RUN sudo sh -c 'echo "deb http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros-latest.list'

RUN sudo apt-key adv --keyserver keyserver.ubuntu.com --recv-keys F42ED6FBAB17C654

RUN apt-get update && \
    apt-get install -y --no-install-recommends \
    gdb \
    apt-utils \
    alsa-utils \
    python3-rosdep \
    python3-pip \
    python3-opencv \
    python3-vcstool \
    python3-pymodbus \
    build-essential \
    ros-noetic-catkin \
    python3-catkin-tools \
    ros-noetic-ros-controllers \
    ros-noetic-plotjuggler-ros \
    nano \
    ros-noetic-soem \
    libvlccore-dev \
    libvlc-dev \
    ros-noetic-joy \
    ros-noetic-rosserial \
    ros-noetic-rosserial-arduino \
    git 

# Install tools and setup groups
RUN apt-get update && apt-get install -y i2c-tools && \
    groupadd ros && \
    usermod -aG i2c root && \
    usermod -aG ros root && \
    rm -rf /var/lib/apt/lists/*

RUN pip3 uninstall -y serial 2>/dev/null || true

RUN pip3 install \
    transforms3d \
    UDPComms \
    pyserial \
    pigpio \
    regex \
    matplotlib \
    pynput \
    spidev \
    adafruit-circuitpython-servokit \
    adafruit-circuitpython-bno055 \
    Jetson.GPIO \
    openai==0.28 

# Make the prompt a little nicer
RUN echo "PS1='${debian_chroot:+($debian_chroot)}\u@:\w\$ '" >> /etc/bash.bashrc 

WORKDIR /dingo_ws
COPY /dingo_ws/src /dingo_ws/src
RUN rosdep update
RUN rosdep install --from-paths src --ignore-src -r -y

RUN echo "source /opt/ros/$ROS_DISTRO/setup.bash" >> /etc/bash.bashrc

RUN /bin/bash -c 'source /opt/ros/$ROS_DISTRO/setup.bash &&\
catkin_make --directory /dingo_ws -DCMAKE_BUILD_TYPE=Debug'

RUN echo "source /opt/ros/$ROS_DISTRO/setup.bash" >> /etc/bash.bashrc
RUN echo "source /dingo_ws/devel/setup.bash" >> /etc/bash.bashrc

COPY ros_entrypoint.sh /ros_entrypoint.sh
RUN chmod +x /ros_entrypoint.sh

RUN groupadd -r ros && usermod -aG ros root
RUN sudo groupadd i2c
RUN sudo chown :i2c /dev/i2c-1
RUN sudo chmod g+rw /dev/i2c-1
RUN echo "exec sg ros bash" >> /root/.bashrc
RUN echo "exec sg i2c "sg ros bash"" >> /root/.bashrc

ENTRYPOINT ["ros_entrypoint.sh"]
CMD [ "bash" ]



