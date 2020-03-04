ARG FROM_IMAGE
FROM $FROM_IMAGE

# Copy and Build Flight Software
COPY ./ ${ROS2_WS}/src/acrobat

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
    --packages-select project_settings

RUN source "${ROS2_WS}/install/setup.bash" && \
    colcon build \
    --symlink-install \
    --cmake-args \
    -DSECURITY=ON \
    --no-warn-unused-cli \
    -DBUILD_TESTING=OFF \
    -DENABLE_CPPCHECK=OFF \
    -DENABLE_CLANG_TIDY=OFF \
    -DCMAKE_BUILD_TYPE=Release

# ===================================
# Set image entry
# ===================================

COPY ./ros_entrypoint.sh /
ENTRYPOINT ["/ros_entrypoint.sh"]
CMD ["ros2", "launch", "acrobat", "acrobat.launch.py"]
