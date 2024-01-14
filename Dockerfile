# todo: fix xforwarding issue
# todo: fix Realsense issues: color image, gyro, accel, point clouds, GPU
# todo: switch to a dusty_nv container https://github.com/dusty-nv/jetson-containers/blob/master/packages/ros/Dockerfile.ros2
# todo: setup NVIDIA ISAAC NVBLOX (mapping) and map localizer
# Todo: setup Nvidia ISAAC ROS vSLAM
# Todo: setup RTABMAP based 2D LaserScan + Realsense global localization
# todo: setup particle filter
# todo: setup micro ros
# pull base image (Autoware or OSRF ROS2 or Dusty-NV)
FROM ghcr.io/autowarefoundation/autoware-universe:humble-latest-cuda-arm64

# Set up the shell
SHELL ["/bin/bash", "-o", "pipefail", "-c"]
ENV TZ=America/New_York

# Setup user
ARG USER=autoware
ARG USERNAME=${USER}
ENV USERNAME ${USERNAME}

ARG USER_UID=1000
ARG USER_GID=$USER_UID

RUN groupadd --gid $USER_GID $USERNAME && \
        useradd --uid $USER_UID --gid $USER_GID -m $USERNAME && \
        echo "$USERNAME:$USERNAME" | chpasswd && \
        usermod --shell /bin/bash $USERNAME && \
        usermod -aG sudo,video $USERNAME && \
        usermod  --uid $USER_UID $USERNAME && \
        echo "$USERNAME ALL=(ALL) NOPASSWD:ALL" >> /etc/sudoers.d/$USERNAME && \
        chmod 0440 /etc/sudoers.d/$USERNAME

# Setup env and shell
ENV LOGNAME root
ENV DEBIAN_FRONTEND noninteractive

SHELL ["/bin/bash", "-o", "pipefail", "-c"]
ENV TZ=America/New_York

ARG ROS_VERSION="ROS2"
ARG ROS_DISTRO="humble"
ENV ROS_DISTRO=${ROS_DISTRO}
ENV ROS_ROOT=/opt/ros/${ROS_DISTRO}
ENV RMW_IMPLEMENTATION=rmw_cyclonedds_cpp

# Install packages
RUN sudo apt-get update -y && DEBIAN_FRONTEND="noninteractive" sudo apt-get install -y --no-install-recommends \
    sudo \
    git \
    curl \
    wget \
    less \
    zstd \
    udev \
    unzip \
    build-essential \
    apt-transport-https \
    openssh-server libv4l-0 libv4l-dev v4l-utils binutils xz-utils bzip2 lbzip2 \
    ca-certificates libegl1 \
    lsb-release \
    gnupg2 \
    cmake \
    pkg-config \
    swig \
    g++ \
    libpython3-dev \
    python3-dev \
    python3 \
    python3-pip \
    python3-setuptools \
    python3-numpy \
    python3-rosdep \
    python3-matplotlib \
    python3-opencv \
    python3-pil \
    python3-yaml \
    python3-tk \
    python3-pyqt5 \
    libopencv-dev \
    libssl-dev \
    libusb-1.0-0-dev \
    libgtk-3-dev \
    libglfw3-dev \
    libgl1-mesa-dev \
    libglu1-mesa-dev \
    qtcreator && \
    sudo rm -rf /var/lib/apt/lists/*

# Install Python Packages
RUN python3 -m pip install do-mpc casadi

# Initialize ROS workspace
ENV BUILD_HOME=/f1tenth_ws
ARG BUILD_HOME=$BUILD_HOME

RUN mkdir -p "$BUILD_HOME/src"

#################################################### (Optional) Setup ROS2
WORKDIR $BUILD_HOME/src

#################################################### Setup TF2 and Geometry2
RUN sudo apt-get update && DEBIAN_FRONTEND="noninteractive" sudo apt-get install -y --no-install-recommends \
    ros-${ROS_DISTRO}-tf2-tools \
    ros-${ROS_DISTRO}-tf-transformations \
    ros-${ROS_DISTRO}-rqt-tf-tree \
    ros-${ROS_DISTRO}-tf2-geometry-msgs && \
    python3 -m pip install transforms3d numpy && \
    sudo rm -rf /var/lib/apt/lists/*

#################################################### (Optional) Setup F1tenth. Todo: setup IMU fix on humble branch
RUN cd "$BUILD_HOME/src" && rm -rf f1tenth_system && git clone https://github.com/privvyledge/f1tenth_system.git -b foxy-devel && \
    cd f1tenth_system && git submodule update --init --force --remote && \
    cd vesc && git checkout ros2_motor_direction_fix
#################################################### (Optional) Setup VESC
#################################################### (Optional) Setup Autoware

#################################################### (Optional) Setup ROS Nav
RUN sudo apt-get update && DEBIAN_FRONTEND="noninteractive" sudo apt-get install -y --no-install-recommends \
    ros-${ROS_DISTRO}-navigation2 \
    ros-${ROS_DISTRO}-nav2-bringup && \
    sudo rm -rf /var/lib/apt/lists/*
# RUN sudo apt-get update && DEBIAN_FRONTEND="noninteractive" sudo apt-get install -y --no-install-recommends \
#    ros-${ROS_DISTRO}-gazebo-ros-pkgs && \
#    cd "$BUILD_HOME/src" && git clone https://github.com/ros-planning/navigation2.git -b ${ROS_DISTRO}-devel && \
#    sudo rm -rf /var/lib/apt/lists/*

#################################################### (Optional) Setup SLAM toolbox. Use galactic and above (or noetic) to get pose
RUN sudo apt-get update && DEBIAN_FRONTEND="noninteractive" sudo apt-get install -y --no-install-recommends \
    ros-${ROS_DISTRO}-slam-toolbox && \
    sudo rm -rf /var/lib/apt/lists/*
# RUN cd "$BUILD_HOME/src" && git clone https://github.com/SteveMacenski/slam_toolbox.git -b ${ROS_DISTRO}-devel && \
#    cd slam_toolbox && rosdep install -q -y -r --from-paths src --ignore-src

#################################################### (Optional) Setup Robot localization
RUN sudo apt-get update && DEBIAN_FRONTEND="noninteractive" sudo apt-get install -y --no-install-recommends \
    ros-${ROS_DISTRO}-robot-localization && \
    sudo rm -rf /var/lib/apt/lists/*
# RUN cd "$BUILD_HOME/src" && git clone https://github.com/cra-ros-pkg/robot_localization.git -b ${ROS_DISTRO}-devel && \
#    cd robot_localization && rosdep install -q -y -r --from-paths src --ignore-src

#################################################### (Optional) Setup IMU Filters
RUN sudo apt-get update && DEBIAN_FRONTEND="noninteractive" sudo apt-get install -y --no-install-recommends \
    ros-${ROS_DISTRO}-imu-tools && \
    sudo rm -rf /var/lib/apt/lists/*

#################################################### Setup Autonomous bringup packages.
RUN cd "$BUILD_HOME/src" && git clone https://github.com/privvyledge/f1tenth_launch.git -b ${ROS_DISTRO}-dev && \
    git clone https://github.com/privvyledge/trajectory_following_ros2.git

#################################################### Setup Laser filters/pipeline.
#RUN apt-get update && DEBIAN_FRONTEND="noninteractive" apt-get install -y --no-install-recommends \
#    ros-${ROS_DISTRO}-laser-pipeline ros-${ROS_DISTRO}-laser-filters  && \
#    rm -rf /var/lib/apt/lists/*
RUN cd "$BUILD_HOME/src" && DEBIAN_FRONTEND="noninteractive" sudo apt-get install -y --no-install-recommends ros-${ROS_DISTRO}-filters && \
    git clone https://github.com/ros-perception/laser_filters.git -b ros2

#################################################### Setup depth image to laser scan. No need as its already in the container
#RUN sudo apt-get update && DEBIAN_FRONTEND="noninteractive" sudo apt-get install -y --no-install-recommends \
#    ros-${ROS_DISTRO}-depth-image-to-laserscan  && \
#    sudo rm -rf /var/lib/apt/lists/*
# RUN #cd "$BUILD_HOME/src" && git clone https://github.com/ros-perception/depthimage_to_laserscan.git -b ros2 && \
#    cd laser_filters && rosdep install -q -y -r --from-paths src --ignore-src

#################################################### Setup laser odometry packages. Todo: setup for humble
#RUN cd "$BUILD_HOME/src" && git clone https://github.com/Adlink-ROS/rf2o_laser_odometry.git && \
#    git clone https://github.com/AlexKaravaev/csm && git clone https://github.com/AlexKaravaev/ros2_laser_scan_matcher.git

#################################################### Setup RTAB-Map (which also publishes odometry from laser_scan)
#RUN cd "$BUILD_HOME/src" && git clone https://github.com/introlab/rtabmap.git && git clone https://github.com/introlab/rtabmap_ros.git -b ${ROS_DISTRO}-devel && \
#    cd rtabmap_ros && rosdep install -q -y -r --from-paths src --ignore-src
RUN sudo apt-get update && DEBIAN_FRONTEND="noninteractive" sudo apt-get install -y --no-install-recommends \
    ros-${ROS_DISTRO}-rtabmap* && \
    sudo rm -rf /var/lib/apt/lists/*

#-------------------------------------------------
# Setup MicroROS (https://github.com/micro-ROS/micro_ros_setup.git | https://micro.ros.org/docs/tutorials/core/first_application_linux/)
#-------------------------------------------------
RUN git clone -b ${ROS_DISTRO} https://github.com/micro-ROS/micro_ros_setup.git src/micro_ros_setup && \
    python3 -m pip install pyserial

#-------------------------------------------------
# Setup Autoware. Todo: use main branch when done
#-------------------------------------------------
#ARG AUTOWARE_DIR='src'
#ARG AUTOWARE_FOLDER_NAME='autoware_gokart'
#RUN git clone -b gokart_devel https://github.com/privvyledge/autoware.gokart.git ${AUTOWARE_DIR}/${AUTOWARE_FOLDER_NAME} && \
#    mkdir -p ${AUTOWARE_DIR}/${AUTOWARE_FOLDER_NAME}/src && \
#    vcs import src/${AUTOWARE_FOLDER_NAME}/src < ${AUTOWARE_DIR}/${AUTOWARE_FOLDER_NAME}/autoware.repos

WORKDIR /sdks

# Install Acados.
ARG TX2_ARCHITECTURE=ARMV8A_ARM_CORTEX_A57
ARG ORIN_ARCHITECTURE=ARMV8A_ARM_CORTEX_A76
ARG ACADO_BLASFEO_TARGET_CPU_ARCHITECHTURE=$TX2_ARCHITECTURE
ARG ACADOS_OPENMP_PARALLELIZATION_ENABLED=OFF
RUN mkdir -p "/sdks/" && cd "/sdks/" && \
    export ACADOS_ROOT='/sdks/acados' && export ACADOS_PATH=${ACADOS_ROOT} && export ACADOS_SOURCE_DIR=${ACADOS_ROOT} && \
    git clone https://github.com/acados/acados.git && cd acados && \
    git submodule update --recursive --init && \
    mkdir build && cd build && \
    cmake -DACADOS_WITH_QPOASES=ON -DACADOS_WITH_OSQP=ON -DACADOS_INSTALL_DIR=${ACADOS_ROOT} -DBLASFEO_TARGET=${ACADO_BLASFEO_TARGET_CPU_ARCHITECHTURE} -DACADOS_WITH_OPENMP=${ACADOS_OPENMP_PARALLELIZATION_ENABLED} .. && \
    make install -j$(nproc) && \
    python3 -m pip install -e ${ACADOS_ROOT}/interfaces/acados_template && \
    curl https://sh.rustup.rs -sSf | sh -s -- -y && \
    source $HOME/.cargo/env && cd ../bin && \
    git clone https://github.com/acados/tera_renderer.git && cd tera_renderer && $HOME/.cargo/bin/cargo build --verbose --release && \
    cp target/release/t_renderer ${ACADOS_ROOT}/bin

#################################################### Setup YDLidar
RUN mkdir -p "/sdks/YDLIDAR" && cd "/sdks/YDLIDAR" && git clone https://github.com/YDLIDAR/YDLidar-SDK.git && \
    cd YDLidar-SDK && mkdir build && cd build && cmake .. && make && sudo make install

RUN cd "$BUILD_HOME/src" && git clone https://github.com/YDLIDAR/ydlidar_ros2_driver.git -b ${ROS_DISTRO}
#RUN cd $BUILD_HOME && \
#    chmod 0777 src/ydlidar_ros2_driver/startup/* && sudo sh src/ydlidar_ros2_driver/startup/initenv.sh

#################################################### Setup Realsense ROS
#RUN sudo apt-get update && DEBIAN_FRONTEND="noninteractive" sudo apt-get install -y --no-install-recommends \
#    ros-${ROS_DISTRO}-librealsense2* \
#    ros-${ROS_DISTRO}-realsense2-* && \
#    sudo rm -rf /var/lib/apt/lists/*

# todo: set envs above instead of exporting
# branches R/2542, master, development, etc
#ARG LIBREALSENSE_BRANCH="R/2542"
#RUN export LD_LIBRARY_PATH=$LD_LIBRARY_PATH:/usr/local/cuda/targets/aarch64-linux/lib/stubs:/opt/ros/${ROS_DISTRO}/install/lib && \
#    export CUDACXX=/usr/local/cuda/bin/nvcc && export PATH=${PATH}:/usr/local/cuda/bin && \
#    cd /sdks && git clone --branch ${LIBREALSENSE_BRANCH} --depth=1 https://github.com/IntelRealSense/librealsense && \
#    cd librealsense && \
#    mkdir build && \
#    cd build && \
#    cmake \
#      -DBUILD_EXAMPLES=true \
#	   -DFORCE_RSUSB_BACKEND=true \
#	   -DBUILD_WITH_CUDA=true \
#	   -DCMAKE_BUILD_TYPE=release \
#	   -DBUILD_PYTHON_BINDINGS=bool:true \
#	   -DPYTHON_EXECUTABLE=/usr/bin/python3 \
#      -DBUILD_GRAPHICAL_EXAMPLES=true \
#	   -DPYTHON_INSTALL_DIR=$(python3 -c 'import sys; print(f"/usr/lib/python{sys.version_info.major}.{sys.version_info.minor}/dist-packages")') \
#	   ../ && \
#    make -j$(($(nproc)-1)) && \
#    sudo make install && \
#    cd ../ && \
#    sudo cp ./config/99-realsense-libusb.rules /etc/udev/rules.d/

ARG LIBREALSENSE_VERSION="2.54.2"
RUN export LD_LIBRARY_PATH=$LD_LIBRARY_PATH:/usr/local/cuda/targets/aarch64-linux/lib/stubs:/opt/ros/${ROS_DISTRO}/install/lib && \
    export CUDACXX=/usr/local/cuda/bin/nvcc && export PATH=${PATH}:/usr/local/cuda/bin && \
    cd /sdks && wget https://github.com/IntelRealSense/librealsense/archive/refs/tags/v${LIBREALSENSE_VERSION}.zip && \
    unzip "v${LIBREALSENSE_VERSION}.zip" && rm "v${LIBREALSENSE_VERSION}.zip" && \
    mv librealsense-${LIBREALSENSE_VERSION} librealsense && cd librealsense && mkdir build && cd build && \
    cmake \
       -DBUILD_EXAMPLES=true \
	   -DFORCE_RSUSB_BACKEND=true \
	   -DBUILD_WITH_CUDA=true \
	   -DCMAKE_BUILD_TYPE=release \
	   -DBUILD_PYTHON_BINDINGS=bool:true \
	   -DPYTHON_EXECUTABLE=/usr/bin/python3 \
       -DBUILD_GRAPHICAL_EXAMPLES=true \
	   -DPYTHON_INSTALL_DIR=$(python3 -c 'import sys; print(f"/usr/lib/python{sys.version_info.major}.{sys.version_info.minor}/dist-packages")') \
	   ../ && \
    make -j$(($(nproc)-1)) && \
    sudo make install &&  \
    cd ../ && \
    sudo cp ./config/99-realsense-libusb.rules /etc/udev/rules.d/

# Setup UDEV Rules. Disconnect all cameras. Todo: Might need to setup udev rules on host instead of docker.
# Todo: test setting up udev rules as root (https://forums.docker.com/t/udevadm-control-reload-rules/135564)

# Install realsense ros
# RUN sudo apt install -y --no-install-recommends ros-${ROS_DISTRO}-realsense2-*
ARG REALSENSE_ROS_VERSION=4.54.1
RUN cd ${BUILD_HOME}/src && wget https://github.com/IntelRealSense/realsense-ros/archive/refs/tags/${REALSENSE_ROS_VERSION}.zip && \
    unzip ${REALSENSE_ROS_VERSION}.zip && \
    mv realsense-ros-${REALSENSE_ROS_VERSION}/ realsense-ros && \
    rm ${REALSENSE_ROS_VERSION}.zip

#--------------------------------
# Build ROS workspace
# The '--event-handlers console_direct+ --base-paths',  ' -DCMAKE_LIBRARY_PATH' and ' -DCMAKE_CXX_FLAGS="-Wl,--allow-shlib-undefined"' flags are needed by ZED
# The ' -DCMAKE_BUILD_TYPE=Release' flag is for all of them, especially Autoware
#--------------------------------
WORKDIR $BUILD_HOME
RUN sudo apt update && \
    source /opt/ros/${ROS_DISTRO}/setup.bash && \
    rosdep update && \
#    rosdep install --from-paths src/autoware/src --ignore-src -r -y && \
    rosdep install --from-paths src --ignore-src -r -y && \
    colcon build --symlink-install --event-handlers console_direct+ --base-paths src --cmake-args ' -DCMAKE_BUILD_TYPE=Release' ' -DCMAKE_LIBRARY_PATH=/usr/local/cuda/lib64/stubs' ' -DCMAKE_CXX_FLAGS="-Wl,--allow-shlib-undefined"'

#-----------------------------
# Setup microros agent, i.e create_agent_ws, then build_agent. Modified to skip rosdep keys and specify colcon build arguments
#-----------------------------
RUN bash -c 'source install/setup.bash; \
             EXTERNAL_SKIP="lio_sam fast_lio"; \
             ros2 run micro_ros_setup create_agent_ws.sh; \
             ros2 run micro_ros_setup build_agent.sh'

#-----------------------------
# Setup environment variables
#-----------------------------
# Todo: use ENV to modify PATHs, e.g PATH, PYTHONPATH, LD_LIBRARY_PATH
# Todo: add autoware environment variables like vehicle_id
RUN echo 'alias build="colcon build --symlink-install  --event-handlers console_direct+"' >> ~/.bashrc && \
    echo 'source /opt/ros/${ROS_DISTRO}/setup.bash' >> ~/.bashrc && \
    echo "source ${BUILD_HOME}/install/setup.bash" >> ~/.bashrc && \
    echo "export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp" >> ~/.bashrc && \
    echo 'export LD_LIBRARY_PATH=${LD_LIBRARY_PATH}:/usr/local/cuda/targets/aarch64-linux/lib/stubs:/opt/ros/${ROS_DISTRO}/install/lib' >> ~/.bashrc && \
    echo 'export LD_LIBRARY_PATH=${LD_LIBRARY_PATH}:/usr/local/lib' >> ~/.bashrc && \
    echo 'export LD_LIBRARY_PATH=${LD_LIBRARY_PATH}:/usr/local/cuda/lib64:/usr/local/cuda/extras/CUPTI/lib64' >> ~/.bashrc && \
    echo 'export LD_LIBRARY_PATH=${LD_LIBRARY_PATH}:/usr/lib/aarch64-linux-gnu/tegra' >> ~/.bashrc && \
    echo 'export PATH=${PATH}:/usr/local/cuda/bin' >> ~/.bashrc && \
    echo 'CUDACXX=/usr/local/cuda/bin/nvcc' >> ~/.bashrc && \
    echo 'export ACADOS_ROOT=/sdks/acados' >> ~/.bashrc && \
    echo 'export ACADOS_PATH=${ACADOS_ROOT}' >> ~/.bashrc && \
    echo 'export ACADOS_SOURCE_DIR=${ACADOS_ROOT}' >> ~/.bashrc && \
    echo 'export LD_LIBRARY_PATH=${LD_LIBRARY_PATH}:${ACADOS_ROOT}/lib' >> ~/.bashrc && \
    echo "source /usr/share/colcon_cd/function/colcon_cd.sh" >> ~/.bashrc && \
    echo "export _colcon_cd_root=${ROS_ROOT}" >> ~/.bashrc && \
    echo "source /usr/share/colcon_argcomplete/hook/colcon-argcomplete.bash" >> ~/.bashrc

## RUN ros2 doctor # run this if the LIDAR doesn't run (https://github.com/YDLIDAR/ydlidar_ros2_driver/issues/10)

ENV NVIDIA_DRIVER_CAPABILITIES all
#ENV NVIDIA_VISIBLE_DEVICES all  # causes graphical failures

## Todo: remove the lines below
RUN sudo apt update && sudo apt install gedit cheese nautilus net-tools iputils-ping -y