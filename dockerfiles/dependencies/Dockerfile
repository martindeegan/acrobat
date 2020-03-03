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
RUN apt-get update && apt-get install -y --no-install-recommends \
    ros-eloquent-sophus \
    ros-eloquent-ament-* \
    ros-eloquent-vision-opencv \
    && rm -rf /var/lib/apt/lists/*

# ============ Install additional packages from source ===============
RUN git clone https://github.com/martindeegan/msp.git "${ROS2_WS}/src/msp"
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
    -DBUILD_TESTS=OFF
