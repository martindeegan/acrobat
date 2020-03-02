# Dependencies Dockerfile - Can be run on CI/CD when a new package is added.
# Installs small packages we need to build acrobat

ARG FROM_IMAGE
FROM $FROM_IMAGE

# ============ Install additional packages ===============
RUN apt-get update && apt-get install -y --no-install-recommends \
    clang clang-tidy \
    && rm -rf /var/lib/apt/lists/*

# ============ Install additional ROS2 packages ===============
RUN apt-get update && apt-get install -y --no-install-recommends \
    ros-eloquent-sophus \
    ros-eloquent-ament-* \
    && rm -rf /var/lib/apt/lists/*
