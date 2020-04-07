# Dependencies Dockerfile - Can be run on CI/CD when a new package is added.
# Installs small packages we need to build acrobat

ARG FROM_IMAGE
FROM $FROM_IMAGE

# ============ Install additional packages ===============
RUN apt-get update && apt-get install -y --no-install-recommends \
    clang \
    clang-tidy \
    libasio-dev \
    libboost-all-dev \
    libusb-1.0-0-dev \
    && rm -rf /var/lib/apt/lists/*

# ============ Install additional ROS2 packages ===============
# ros1 repo required for rosbag2* dependencies
RUN sh -c 'echo "deb http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros-latest.list' \
    && apt-key adv --keyserver 'hkp://keyserver.ubuntu.com:80' --recv-key C1CF6E31E6BADE8868B172B4F42ED6FBAB17C654
RUN apt-get update && apt-get install -y --no-install-recommends \
    ros-eloquent-sophus \
    ros-eloquent-ament-* \
    ros-eloquent-ros2bag \
    ros-eloquent-rosbag2* \
    && rm -rf /var/lib/apt/lists/*

# ============ Install additional packages from source ===============
RUN git clone --branch acrobat_mod https://github.com/martindeegan/msp.git "${ROS2_WS}/src/msp"
RUN git clone --branch acrobat_mod https://github.com/martindeegan/vision_opencv.git "${ROS2_WS}/src/vision_opencv"
RUN source "${ROS2_WS}/install/setup.bash" && \
    colcon build \
    --symlink-install \
    --cmake-args \
    -DSECURITY=ON \
    --no-warn-unused-cli \
    -DBUILD_TESTING=OFF \
    -DENABLE_CPPCHECK=OFF \
    -DENABLE_CLANG_TIDY=OFF \
    -DCMAKE_BUILD_TYPE=Release \
    -DBUILD_EXAMPLES=OFF \
    -DBUILD_TESTS=OFF \
    -DG2O_BUILD_EXAMPLES=OFF \
    -DG2O_BUILD_APPS=OFF \
    -DG2O_USE_OPENGL=OFF \
    -DG2O_USE_CSPARSE=OFF \
    -DBUILD_CSPARSE=OFF
