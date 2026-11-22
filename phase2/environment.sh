#!/usr/bin/env bash
# Source from Bash; retain the established Phase 0 domain/discovery settings.
PHASE2_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
PX4_DIR="${PX4_DIR:-$HOME/PX4-Autopilot}"
export ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-10}"
export ROS_AUTOMATIC_DISCOVERY_RANGE="${ROS_AUTOMATIC_DISCOVERY_RANGE:-SUBNET}"
export ROS_STATIC_PEERS="${ROS_STATIC_PEERS:-192.168.50.2}"
export RMW_IMPLEMENTATION="${RMW_IMPLEMENTATION:-rmw_fastrtps_cpp}"
export GZ_SIM_RESOURCE_PATH="$PHASE2_DIR/models:$PX4_DIR/Tools/simulation/gz/models:$PHASE2_DIR/worlds${GZ_SIM_RESOURCE_PATH:+:$GZ_SIM_RESOURCE_PATH}"
export SDF_PATH="$PHASE2_DIR/models:$PX4_DIR/Tools/simulation/gz/models${SDF_PATH:+:$SDF_PATH}"
# A separate transport partition prevents accidental attachment to another test.
export GZ_PARTITION="${GZ_PARTITION:-uav_phase2}"
while IFS= read -r phase2_plugin_dir; do
    export GZ_SIM_SYSTEM_PLUGIN_PATH="$phase2_plugin_dir${GZ_SIM_SYSTEM_PLUGIN_PATH:+:$GZ_SIM_SYSTEM_PLUGIN_PATH}"
done < <(find "$PX4_DIR/build/px4_sitl_default" -name 'libMotorFailurePlugin.so' -printf '%h\n')
source /opt/ros/jazzy/setup.bash
