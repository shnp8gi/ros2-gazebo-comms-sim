# =============================================================================
# ROS 2 Humble + Gazebo Harmonic Docker Image
# Ubuntu 22.04 LTS Base
# =============================================================================

FROM ubuntu:22.04

ENV DEBIAN_FRONTEND=noninteractive
ENV TZ=Asia/Tokyo

# System Dependencies
RUN apt-get update && apt-get install -y \
    curl gnupg2 lsb-release wget git python3 python3-pip \
    python3-setuptools python3-yaml locales software-properties-common \
    && rm -rf /var/lib/apt/lists/*

RUN locale-gen en_US en_US.UTF-8 && \
    update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8
ENV LANG=en_US.UTF-8

# ROS 2 Humble Installation
RUN curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg
RUN echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(. /etc/os-release && echo $UBUNTU_CODENAME) main" | tee /etc/apt/sources.list.d/ros2.list > /dev/null
RUN apt-get update && apt-get install -y \
    ros-humble-desktop ros-humble-ros-base ros-dev-tools \
    python3-colcon-common-extensions python3-rosdep \
    && rm -rf /var/lib/apt/lists/*
RUN rosdep init && rosdep update

# Gazebo Harmonic Installation
RUN wget https://packages.osrfoundation.org/gazebo.gpg -O /usr/share/keyrings/pkgs-osrf-archive-keyring.gpg
RUN echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/pkgs-osrf-archive-keyring.gpg] http://packages.osrfoundation.org/gazebo/ubuntu-stable $(lsb_release -cs) main" | tee /etc/apt/sources.list.d/gazebo-stable.list > /dev/null
RUN apt-get update && apt-get install -y gz-harmonic && rm -rf /var/lib/apt/lists/*

# ROS-Gazebo Bridge (ros_gz) & Python Dependencies
RUN apt-get update && apt-get install -y ros-humble-ros-gzharmonic \
    python3-numpy python3-scipy python3-pandas && rm -rf /var/lib/apt/lists/*

# VirtualGL Installation for Remote GPU Rendering
RUN wget https://github.com/VirtualGL/virtualgl/releases/download/3.1.1/virtualgl_3.1.1_amd64.deb -O /tmp/virtualgl.deb && \
    apt-get update && apt-get install -y /tmp/virtualgl.deb libglvnd-dev libegl1-mesa-dev && \
    rm /tmp/virtualgl.deb && rm -rf /var/lib/apt/lists/*

# Environment Setup
ENV ROS_DISTRO=humble
ENV NVIDIA_VISIBLE_DEVICES=all
ENV NVIDIA_DRIVER_CAPABILITIES=graphics,compute,utility,display

RUN echo "source /opt/ros/humble/setup.bash" >> ~/.bashrc
ENV GZ_SIM_RESOURCE_PATH=/workspace/models
WORKDIR /workspace

COPY ./entrypoint.sh /entrypoint.sh
RUN chmod +x /entrypoint.sh
ENTRYPOINT ["/entrypoint.sh"]
CMD ["bash"]
