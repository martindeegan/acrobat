# ===================================
# Set parent images
# ===================================

# For base image
ARG FROM_IMAGE=martindeegan/acrobat:base_arm64
FROM $FROM_IMAGE AS testing

SHELL ["/bin/bash", "-c"]
ARG DEBIAN_FRONTEND=noninteractive

# ===================================
# Create Testing Image
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
# Create Runtime Image
# ===================================

FROM $FROM_IMAGE AS runtime

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
    -DBUILD_TESTING=OFF \
    -DENABLE_CPPCHECK=OFF \
    -DENABLE_CLANG_TIDY=OFF

# ===================================
# Set image entry
# ===================================

COPY ./ros_entrypoint.sh /
ENTRYPOINT ["/ros_entrypoint.sh"]
CMD ["ros2", "launch", "demo", "demo.launch.py"]
