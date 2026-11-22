#!/usr/bin/env bash
set -eo pipefail
source "$(dirname -- "${BASH_SOURCE[0]}")/environment.sh"
exec ros2 run ros_gz_bridge parameter_bridge --ros-args \
    -p config_file:="$PHASE2_DIR/bridge.yaml" \
    -p use_sim_time:=true
