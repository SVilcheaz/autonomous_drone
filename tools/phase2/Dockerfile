# Official 4.6 JetPack image, verified from its registry build history.
ARG BASE_IMAGE=nvcr.io/nvidia/isaac/ros@sha256:7d8ce7640912976f604d9f1d28c986f6abdc44571498b5c351cfcbb88b51af4c
FROM ${BASE_IMAGE}
USER root
SHELL ["/bin/bash", "-c"]
ARG DEBIAN_FRONTEND=noninteractive
ARG ISAAC_VERSION=4.6.0-0noble.20260818193828000
RUN --mount=type=cache,target=/var/cache/apt,sharing=locked \
    rm -f /etc/apt/apt.conf.d/docker-clean && \
    rm -rf /var/lib/apt/lists/* && \
    apt-get -o Acquire::Retries=5 update && apt-get -o Acquire::Retries=5 install -y --no-install-recommends \
    ros-jazzy-isaac-ros-stereo-image-proc=${ISAAC_VERSION} \
    ros-jazzy-isaac-ros-nitros-image-type=${ISAAC_VERSION} \
    ros-jazzy-isaac-ros-nitros-disparity-image-type=${ISAAC_VERSION} \
    ros-jazzy-ament-cmake-gtest \
    ros-jazzy-tf2-msgs \
    python3-numpy \
    && rm -rf /var/lib/apt/lists/*
WORKDIR /workspaces/isaac_ros-dev
