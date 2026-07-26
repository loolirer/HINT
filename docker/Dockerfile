# Use the base ROS2 jazzy image
FROM osrf/ros:jazzy-desktop

# Set environment variables
ENV DEBIAN_FRONTEND=noninteractive
ENV COLCON_WS=/root/hint_ws

# Install the required packages in a single layer and clean up afterwards
RUN apt-get update && \
    apt-get install -y --no-install-recommends \
    ros-${ROS_DISTRO}-desktop \
    python3-argcomplete \
    python3-colcon-common-extensions \
    libboost-system-dev \
    build-essential \
    libudev-dev \
    udev \
    git \
    nano \
    ros-${ROS_DISTRO}-cartographer \
    ros-${ROS_DISTRO}-cartographer-ros \
    ros-${ROS_DISTRO}-navigation2 \
    ros-${ROS_DISTRO}-nav2-bringup \
    ros-${ROS_DISTRO}-nav2-route \
    ros-${ROS_DISTRO}-urdf \
    ros-${ROS_DISTRO}-rmw-cyclonedds-cpp \
    ros-${ROS_DISTRO}-compressed-image-transport \
    ros-${ROS_DISTRO}-topic-tools \
    ros-${ROS_DISTRO}-rosbag2-storage-mcap \
    && rm -rf /var/lib/apt/lists/*

RUN apt-get update && \
    apt-get install -y --no-install-recommends \
    python3-pip \
    python3-jinja2 \
    ninja-build \
    libgnutls28-dev \
    openssl \
    libtiff-dev \
    pybind11-dev \
    qtbase5-dev \
    libqt5core5a \
    libqt5widgets5 \
    cmake \
    python3-yaml \
    python3-ply \
    libglib2.0-dev \
    libgstreamer-plugins-base1.0-dev \
    fzf \
    tree \
    jq \
    intel-opencl-icd \
    ocl-icd-libopencl1 \
    clinfo \
    && rm -rf /var/lib/apt/lists/*

RUN mkdir -p /opt/btcpp_ros2/src && \
    git clone https://github.com/BehaviorTree/BehaviorTree.ROS2.git \
        /opt/btcpp_ros2/src/BehaviorTree.ROS2 && \
    git -C /opt/btcpp_ros2/src/BehaviorTree.ROS2 checkout ${BTCPP_ROS2_REF} && \
    bash -c "source /opt/ros/${ROS_DISTRO}/setup.bash && cd /opt/btcpp_ros2 && \
        colcon build --merge-install --packages-up-to behaviortree_ros2 \
        --cmake-args -DCMAKE_BUILD_TYPE=Release" && \
    rm -rf /opt/btcpp_ros2/build /opt/btcpp_ros2/log /opt/btcpp_ros2/src

WORKDIR ${COLCON_WS}

RUN bash -c "source /opt/ros/${ROS_DISTRO}/setup.bash && \
    cd ${COLCON_WS} && \
    colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release"

COPY docker/requirements.txt ./requirements.txt
RUN python3 -m pip config set global.break-system-packages true && \
    pip3 install --no-cache-dir -r requirements.txt

RUN echo "source /opt/ros/${ROS_DISTRO}/setup.bash" >> ~/.bashrc && \
    echo "source /opt/btcpp_ros2/install/setup.bash" >> ~/.bashrc && \
    echo "source ${COLCON_WS}/install/setup.bash" >> ~/.bashrc && \
    echo "alias cbss='colcon build --symlink-install && source install/setup.sh'" >> ~/.bashrc

CMD ["bash"]
