ARG FROM_IMAGE
FROM $FROM_IMAGE

# Copy and Build Flight Software
COPY ./ ${ROS2_WS}/src/acrobat

RUN source "${ROS2_WS}/install/setup.bash" && \
    colcon build \
    --symlink-install \
    --packages-skip vision_opencv msp cv_bridge image_geometry opencv_tests \
    --cmake-args \
    -DSECURITY=ON \
    --no-warn-unused-cli \
    -DBUILD_TESTING=OFF \
    -DENABLE_CPPCHECK=OFF \
    -DENABLE_CLANG_TIDY=OFF \
    -DCMAKE_BUILD_TYPE=Release \
    -DG2O_BUILD_EXAMPLES=OFF \
    -DG2O_BUILD_APPS=OFF \
    -DG2O_USE_OPENGL=OFF \
    -DG2O_USE_CSPARSE=OFF \
    -DBUILD_CSPARSE=OFF


# ===================================
# Set image entry
# ===================================

COPY ./ros_entrypoint.sh /
ENTRYPOINT ["/ros_entrypoint.sh"]
CMD ["ros2", "launch", "acrobat", "acrobat.launch.py"]
