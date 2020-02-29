# For use with nvidia docker on the Jetson

ARG FROM_IMAGE=nvcr.io/nvidia/l4t-base:r32.3.1
FROM $FROM_IMAGE

SHELL ["/bin/bash", "-c"]
ARG DEBIAN_FRONTEND=noninteractive

# setup timezone
RUN echo 'Etc/UTC' > /etc/timezone && \
    ln -s /usr/share/zoneinfo/Etc/UTC /etc/localtime && \
    apt-get update && apt-get install -q -y tzdata && rm -rf /var/lib/apt/lists/*

# ===================================
# Install Dependencies
# ===================================

RUN apt-get update && apt-get install -q -y locales \
    curl \
    gnupg2 \
    lsb-release \
    software-properties-common \
    bash-completion \
    dirmngr \
    python3-pip \
    wget \
    && rm -rf /var/lib/apt/lists/*

# ============ Install ROS Eloquent ===============

# setup environment
RUN locale-gen en_US en_US.UTF-8
RUN update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8
RUN export LANG=en_US.UTF-8

# setup keys
RUN apt-key adv --keyserver hkp://keyserver.ubuntu.com:80 --recv-keys C1CF6E31E6BADE8868B172B4F42ED6FBAB17C654

# setup sources.list
RUN sh -c 'echo "deb [arch=amd64,arm64] http://packages.ros.org/ros2/ubuntu `lsb_release -cs` main" > /etc/apt/sources.list.d/ros2-latest.list'

# install bootstrap tools
RUN apt-get update && apt-get install --no-install-recommends -y \
    git \
    python3-colcon-common-extensions \
    python3-colcon-mixin \
    python3-rosdep \
    python3-vcstool \
    && rm -rf /var/lib/apt/lists/*

ENV ROS_DISTRO eloquent
ENV ROS_VERSION=2 \
    ROS_PYTHON_VERSION=3

# bootstrap rosdep
RUN rosdep init && \
    rosdep update --rosdistro $ROS_DISTRO

# setup colcon mixin and metadata
RUN colcon mixin add default \
    https://raw.githubusercontent.com/colcon/colcon-mixin-repository/master/index.yaml && \
    colcon mixin update && \
    colcon metadata add default \
    https://raw.githubusercontent.com/colcon/colcon-metadata-repository/master/index.yaml && \
    colcon metadata update

# install python packages
RUN pip3 install -U \
    argcomplete

# install ros2 base
RUN apt-get update && apt-get install -y \
    ros-eloquent-ros-base \
    && rm -rf /var/lib/apt/lists/*

# ============ Install CMake 3.17 ==================
RUN wget https://github.com/Kitware/CMake/releases/download/v3.17.0-rc1/cmake-3.17.0-rc1.tar.gz
RUN mkdir -p /cmake_source
RUN tar -xvzf cmake-3.17.0-rc1.tar.gz
WORKDIR /cmake-3.17.0-rc1
RUN ./bootstrap
RUN make -j4 && make install
# Clean build files to reduce size
RUN rm -rf /cmake-3.17.0-rc1

# ============ Install additional packages ===============
RUN apt-get update
RUN apt-get install -y clang clang-tidy

# ============ Install additional ROS2 packages ===============
RUN apt-get install -y ros-eloquent-sophus
RUN apt-get install -y ros-eloquent-ament-*

# ===================================
# Create ROS Workspace
# ===================================

ENV ROS2_WS /acrobat
WORKDIR ${ROS2_WS}
RUN mkdir -p ${ROS2_WS}/src/acrobat_packages

# Copy and Build Flight Software
COPY ./ ${ROS2_WS}/src/acrobat_packages

RUN source /opt/ros/${ROS_DISTRO}/setup.bash && \
    colcon build \
    --symlink-install \
    --cmake-args \
    -DSECURITY=ON \
    --no-warn-unused-cli \
    -DBUILD_TESTING=ON \
    -DENABLE_CPPCHECK=ON \
    -DENABLE_CLANG_TIDY=ON

RUN source ${ROS2_WS}/install/setup.bash && \
    colcon test --return-code-on-test-failure

# ===================================
# Set image entry
# ===================================

COPY ./ros_entrypoint.sh /
ENTRYPOINT ["/ros_entrypoint.sh"]
CMD ["ros2", "launch", "demo", "demo.launch.py"]
