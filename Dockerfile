# pull base image
FROM f1tenth/focal-l4t-foxy:f1tenth-stack

# Set up the shell
SHELL ["/bin/bash", "-o", "pipefail", "-c"]
ENV TZ=America/New_York

# Install packages
RUN apt-get update -y && DEBIAN_FRONTEND="noninteractive" apt-get install -y --no-install-recommends \
    sudo \
    git \
    curl \
    wget \
    build-essential \
    lsb-release \
    gnupg2 \
    cmake \
    pkg-config \
    swig \
    g++ \
    libpython3-dev \
    python3-dev \
    python \
    python3 \
    python3-pip \
    python3-setuptools \
    python3-numpy \
    python3-matplotlib \
    python3-opencv \
    python3-pil \
    python3-yaml \
    python3-tk \
    python3-pyqt5 \
    libopencv-dev \
    gedit \
    nautilus && \
    rm -rf /var/lib/apt/lists/*

# Initialize ROS workspace
ENV BUILD_HOME=/f1tenth_ws
ARG BUILD_HOME=$BUILD_HOME

RUN mkdir -p "$BUILD_HOME/src"

#################################################### (Optional) Setup ROS2

#################################################### Setup YDLidar
RUN mkdir -p "/SDKs/YDLIDAR" && cd "/SDKs/YDLIDAR" && git clone https://github.com/YDLIDAR/YDLidar-SDK.git && \
    cd YDLidar-SDK && mkdir build && cd build && cmake .. && make && sudo make install

RUN cd "$BUILD_HOME/src" && git clone https://github.com/YDLIDAR/ydlidar_ros2_driver.git
#RUN cd $BUILD_HOME && \
#    chmod 0777 src/ydlidar_ros2_driver/startup/* && sudo sh src/ydlidar_ros2_driver/startup/initenv.sh

#################################################### Setup Realsense ROS
RUN apt-get update && DEBIAN_FRONTEND="noninteractive" apt-get install -y --no-install-recommends \
    ros-${ROS_DISTRO}-librealsense2* \
    ros-${ROS_DISTRO}-realsense2-* && \
    rm -rf /var/lib/apt/lists/*

#################################################### (Optional) Setup F1tenth
RUN cd "$BUILD_HOME/src" && rm -rf f1tenth_system && git clone https://github.com/privvyledge/f1tenth_system.git && \
    cd f1tenth_system && git submodule update --init --force --remote
#################################################### (Optional) Setup VESC
#################################################### (Optional) Setup microROS
#################################################### (Optional) Setup Autoware
#################################################### (Optional) Setup ROS Nav
RUN apt-get update && DEBIAN_FRONTEND="noninteractive" apt-get install -y --no-install-recommends \
    ros-${ROS_DISTRO}-navigation2 \
    ros-${ROS_DISTRO}-nav2-bringup && \
    rm -rf /var/lib/apt/lists/*
#################################################### (Optional) Setup SLAM toolbox. Use galactic and above (or noetic) to get pose
#RUN apt-get update && DEBIAN_FRONTEND="noninteractive" apt-get install -y --no-install-recommends \
#    ros-${ROS_DISTRO}-slam-toolbox && \
#    rm -rf /var/lib/apt/lists/*
RUN cd "$BUILD_HOME/src" && git clone https://github.com/SteveMacenski/slam_toolbox.git -b ${ROS_DISTRO}-devel && \
    cd slam_toolbox && rosdep install -q -y -r --from-paths src --ignore-src
#################################################### (Optional) Setup Robot localization
RUN apt-get update && DEBIAN_FRONTEND="noninteractive" apt-get install -y --no-install-recommends \
    ros-${ROS_DISTRO}-robot-localization && \
    rm -rf /var/lib/apt/lists/*
#################################################### (Optional) Setup IMU Filters
RUN apt-get update && DEBIAN_FRONTEND="noninteractive" apt-get install -y --no-install-recommends \
    ros-${ROS_DISTRO}-imu-tools && \
    rm -rf /var/lib/apt/lists/*
#################################################### Setup Autonomous bringup package.
RUN cd "$BUILD_HOME/src" && git clone https://github.com/privvyledge/f1tenth_launch.git -b localization_dev && \
    rosdep install -q -y -r --from-paths src --ignore-src

# Setup permanent variables
RUN echo "source /f1tenth_ws/install/setup.bash" >> ~/.bashrc

# RUN ros2 doctor # run this if the LIDAR doesn't run (https://github.com/YDLIDAR/ydlidar_ros2_driver/issues/10)
RUN cd "$BUILD_HOME" && \
    source ${ROS_ROOT}/setup.bash && \
    rosdep update && rosdep install --from-paths src -i -y && \
    colcon build --symlink-install
