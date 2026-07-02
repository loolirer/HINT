# Use the base ROS2 jazzy image
FROM osrf/ros:jazzy-desktop

# Set environment variables
ENV DEBIAN_FRONTEND=noninteractive
ENV COLCON_WS=/root/turtlebot3_ws

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
    ros-${ROS_DISTRO}-turtlebot3-msgs \
    ros-${ROS_DISTRO}-dynamixel-sdk \
    ros-${ROS_DISTRO}-xacro \
    ros-${ROS_DISTRO}-hls-lfcd-lds-driver \
    ros-${ROS_DISTRO}-ld08-driver \
    ros-${ROS_DISTRO}-coin-d4-driver \
    ros-${ROS_DISTRO}-camera-ros \
    ros-${ROS_DISTRO}-urdf \
    ros-${ROS_DISTRO}-rmw-cyclonedds-cpp \
    ros-${ROS_DISTRO}-compressed-image-transport \
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
    && rm -rf /var/lib/apt/lists/*

WORKDIR ${COLCON_WS}

RUN mkdir -p ${COLCON_WS}/src && \
    cd ${COLCON_WS}/src && \
    git clone -b jazzy https://github.com/ROBOTIS-GIT/turtlebot3.git

RUN bash -c "source /opt/ros/${ROS_DISTRO}/setup.bash && \
    cd ${COLCON_WS} && \
    colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release"

RUN python3 -m pip config set global.break-system-packages true
RUN pip3 install meson

WORKDIR /root/turtlebot3_ws
RUN git clone -b v0.5.2 --depth 1 https://github.com/raspberrypi/libcamera.git && \
    cd libcamera && \
    meson setup build --buildtype=release -Dpipelines=rpi/vc4,rpi/pisp -Dipas=rpi/vc4,rpi/pisp -Dv4l2=true -Dgstreamer=enabled -Dtest=false -Dlc-compliance=disabled -Dcam=disabled -Dqcam=disabled -Ddocumentation=disabled -Dpycamera=enabled && \
    ninja -C build install && \
    ldconfig

COPY requirements.txt ./requirements.txt
RUN pip3 install --no-cache-dir -r requirements.txt

RUN echo "source /opt/ros/${ROS_DISTRO}/setup.bash" >> ~/.bashrc && \
    echo "source ${COLCON_WS}/install/setup.bash" >> ~/.bashrc && \
    echo "alias cb='colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release'" >> ~/.bashrc && \
    echo "alias kh='pkill -TERM -f \"ros2 launch\" 2>/dev/null; pkill -TERM -f \"ros2 run\" 2>/dev/null; sleep 1; pkill -9 -f \"ros2\" 2>/dev/null; pkill -9 -f lk_tracker 2>/dev/null; pkill -9 -f visual_servo 2>/dev/null; pkill -9 -f cartographer 2>/dev/null; pkill -9 rviz2 2>/dev/null; ros2 daemon stop 2>/dev/null; ros2 daemon start 2>/dev/null; echo done'" >> ~/.bashrc && \
    echo "export ROS_DOMAIN_ID=30" >> ~/.bashrc && \
    echo 'export LD_LIBRARY_PATH=/usr/local/lib/aarch64-linux-gnu:$LD_LIBRARY_PATH' >> ~/.bashrc && \
    echo "export TURTLEBOT3_MODEL=waffle_pi" >> ~/.bashrc && \
    echo "export LDS_MODEL=LDS-02" >> ~/.bashrc

CMD ["bash"]
