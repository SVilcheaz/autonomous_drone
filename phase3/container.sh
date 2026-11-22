#!/usr/bin/env bash
set -euo pipefail

phase3_dir="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
image="${PHASE3_IMAGE:-uav-phase3:isaac-ros4.6}"
name=uav_phase3_vio
dds_profile="$phase3_dir/../tools/phase2/fastdds_jetson.xml"
mode="${1:-help}"

case "$mode" in
    build)
        docker build --progress plain -t "$image" "$phase3_dir"
        ;;
    run|start)
        shift
        flags=()
        if [ "$mode" = start ]; then flags+=(--detach); fi
        docker run "${flags[@]}" --rm --init --stop-signal SIGINT --name "$name" \
            --runtime nvidia --gpus all --network host --ipc host \
            --user "$(id -u):$(id -g)" \
            --group-add "$(getent group video | cut -d: -f3)" \
            --group-add "$(getent group render | cut -d: -f3)" \
            --mount "type=bind,src=$phase3_dir,dst=/phase3,readonly" \
            --mount "type=bind,src=$dds_profile,dst=/fastdds.xml,readonly" \
            -e "ROS_DOMAIN_ID=${ROS_DOMAIN_ID:-10}" \
            -e "ROS_AUTOMATIC_DISCOVERY_RANGE=${ROS_AUTOMATIC_DISCOVERY_RANGE:-SUBNET}" \
            -e "ROS_STATIC_PEERS=${ROS_STATIC_PEERS:-192.168.50.1}" \
            -e "RMW_IMPLEMENTATION=${RMW_IMPLEMENTATION:-rmw_fastrtps_cpp}" \
            -e FASTDDS_DEFAULT_PROFILES_FILE=/fastdds.xml \
            -e FASTRTPS_DEFAULT_PROFILES_FILE=/fastdds.xml \
            -e ROS_LOG_DIR=/tmp/ros -e NVIDIA_DRIVER_CAPABILITIES=all \
            --entrypoint /bin/bash "$image" -ec \
            'source /opt/ros/jazzy/setup.bash; exec ros2 launch /phase3/stereo_vio.launch.py "$@"' bash "$@"
        ;;
    exec)
        shift
        docker exec "$name" /bin/bash -ec \
            'source /opt/ros/jazzy/setup.bash; exec "$@"' bash "$@"
        ;;
    logs)
        docker logs --tail 100 "$name"
        ;;
    stop)
        docker stop --timeout 20 "$name"
        ;;
    *)
        echo 'Usage: bash phase3/container.sh {build|run|start|exec COMMAND...|logs|stop}'
        ;;
esac
