ARG FROM_IMAGE
FROM $FROM_IMAGE

# Copy and Build Flight Software
COPY ./ ${ROS2_WS}/src/acrobat_packages

RUN source "${ROS2_WS}/install/setup.bash" && \
    colcon build \
    --symlink-install \
    --cmake-args \
    -DSECURITY=ON \
    --no-warn-unused-cli \
    -DBUILD_TESTING=ON \
    -DENABLE_CPPCHECK=ON \
    -DENABLE_CLANG_TIDY=ON

# Run unit tests
RUN source "${ROS2_WS}/install/setup.bash" && \
    colcon test --return-code-on-test-failure

COPY ./ros_entrypoint.sh /
ENTRYPOINT ["/ros_entrypoint.sh"]
CMD ["/bin/bash"]
