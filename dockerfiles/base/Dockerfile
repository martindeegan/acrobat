# Base Dockerfile - Builds expensive dockerfile. This should not be run on CI/CD
#   Installs: 
#       ROS2 Eloquent
#       CMake
#   Sets up ROS2 workspace

ARG FROM_IMAGE
FROM $FROM_IMAGE

SHELL ["/bin/bash", "-c"]
ARG DEBIAN_FRONTEND=noninteractive

# setup timezone
RUN echo 'Etc/UTC' > /etc/timezone && \
    ln -s /usr/share/zoneinfo/Etc/UTC /etc/localtime
RUN apt-get update && apt-get install -q --no-install-recommends -y tzdata \
    &&  rm -rf /var/lib/apt/lists/*

# ===================================
# Install Dependencies
# ===================================

RUN apt-get update && apt-get install -q --no-install-recommends -y \
    build-essential \
    bash-completion \
    curl \
    dirmngr \
    gnupg2 \
    locales \
    lsb-release \
    python3-pip \
    software-properties-common \
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
RUN apt-get update && apt-get install --no-install-recommends -y \
    ros-eloquent-ros-base \
    && rm -rf /var/lib/apt/lists/*

RUN mkdir -p /cuda_toolkit
COPY ./cuda-repo-ubuntu1804-10-2-local-10.2.107-435.17.01_1.0-1_arm64.deb /cuda_toolkit
WORKDIR /cuda_toolkit
RUN wget https://developer.download.nvidia.com/compute/cuda/repos/ubuntu1804/x86_64/cuda-ubuntu1804.pin
RUN mv cuda-ubuntu1804.pin /etc/apt/preferences.d/cuda-repository-pin-600
RUN dpkg -i cuda-repo-ubuntu1804-10-2-local-10.2.107-435.17.01_1.0-1_arm64.deb
RUN apt-key add /var/cuda-repo-10-2-local-10.2.107-435.17.01/7fa2af80.pub
RUN apt-get update
RUN apt-get -y install cuda
WORKDIR /
RUN rm -rf /cuda_toolkit
RUN rm -rf /usr/local/cuda-10.0 /usr/local/cuda
RUN ln -s /usr/local/cuda-10.2 /usr/local/cuda
ENV LD_LIBRARY_PATH /usr/local/cuda/targets/aarch64-linux/lib
ENV PATH /usr/local/cuda/bin:${PATH}

# ============ Install OpenCV 4.3.0 ===============
RUN mkdir -p /opencv_install
WORKDIR /opencv_install
RUN git clone --branch 4.3.0 https://github.com/opencv/opencv.git
RUN git clone --branch 4.3.0 https://github.com/opencv/opencv_contrib.git
WORKDIR /opencv_install/opencv
RUN mkdir build
WORKDIR /opencv_install/opencv/build
RUN cmake -DBUILD_TESTS=OFF -DWITH_CUDA=ON -DOPENCV_EXTRA_MODULES_PATH=/opencv_install/opencv_contrib/modules ..
RUN make
RUN make install

WORKDIR /
RUN rm -rf /opencv_install

# ============ Install CMake 3.17 ==================
RUN wget https://github.com/Kitware/CMake/releases/download/v3.17.0-rc1/cmake-3.17.0-rc1.tar.gz
RUN mkdir -p /cmake_source
RUN tar -xvzf cmake-3.17.0-rc1.tar.gz
RUN rm cmake-3.17.0-rc1.tar.gz
WORKDIR /cmake-3.17.0-rc1
RUN ./bootstrap
RUN make -j4 && make install
# Clean build files to reduce size
RUN rm -rf /cmake-3.17.0-rc1


# ===================================
# Create ROS2 Workspace
# ===================================

ENV ROS2_WS /acrobat
RUN mkdir -p ${ROS2_WS}/src/acrobat_packages
WORKDIR ${ROS2_WS}

RUN source "/opt/ros/${ROS_DISTRO}/setup.bash" && colcon build --symlink-install


