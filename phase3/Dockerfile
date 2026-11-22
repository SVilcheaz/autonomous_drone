ARG BASE_IMAGE=uav-phase2:isaac-ros4.6
FROM ${BASE_IMAGE}

USER root
ARG DEBIAN_FRONTEND=noninteractive
ARG ISAAC_VERSION=4.6.0-0noble.20260818193828000
RUN --mount=type=cache,target=/var/cache/apt,sharing=locked \
    apt-get -o Acquire::Retries=3 update && \
    apt-get -o Acquire::Retries=3 install -y --no-install-recommends \
        ros-jazzy-isaac-ros-visual-slam=${ISAAC_VERSION} && \
    rm -rf /var/lib/apt/lists/*
