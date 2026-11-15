#!/usr/bin/env bash
# Jetson-only helper. Do not source phase2/environment.sh here: that file is for WSL.
set -eo pipefail
phase2_repo="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/../.." && pwd)"
phase2_image="${PHASE2_IMAGE:-uav-phase2:isaac-ros4.6}"
phase2_workspace="$phase2_repo/phase2/jetson_workspace"
phase2_results="$phase2_repo/phase2/results/jetson"
mkdir -p "$phase2_workspace" "$phase2_results"
touch "$phase2_workspace/COLCON_IGNORE"

docker_cmd() {
    if docker info >/dev/null 2>&1; then
        docker "$@"
    else
        # Membership may have been enabled after the current login was created.
        sg docker -c "$(printf '%q ' docker "$@")"
    fi
}

run_container() {
    local phase2_args=(run --rm --runtime nvidia --gpus all --network host --ipc host
        --user "$(id -u):$(id -g)"
        --group-add "$(getent group video | cut -d: -f3)"
        --group-add "$(getent group render | cut -d: -f3)"
        --mount "type=bind,src=$phase2_repo,dst=/project,readonly"
        --mount "type=bind,src=$phase2_workspace,dst=/workspaces/isaac_ros-dev"
        --mount "type=bind,src=$phase2_results,dst=/results"
        -e "ROS_DOMAIN_ID=${ROS_DOMAIN_ID:-10}"
        -e "ROS_AUTOMATIC_DISCOVERY_RANGE=${ROS_AUTOMATIC_DISCOVERY_RANGE:-SUBNET}"
        -e "ROS_STATIC_PEERS=${ROS_STATIC_PEERS:-192.168.50.1}"
        -e "RMW_IMPLEMENTATION=${RMW_IMPLEMENTATION:-rmw_fastrtps_cpp}"
        -e FASTDDS_DEFAULT_PROFILES_FILE=/project/tools/phase2/fastdds_jetson.xml
        -e FASTRTPS_DEFAULT_PROFILES_FILE=/project/tools/phase2/fastdds_jetson.xml
        -e ROS_LOG_DIR=/workspaces/isaac_ros-dev/log/ros
        -e NVIDIA_VISIBLE_DEVICES=all -e NVIDIA_DRIVER_CAPABILITIES=all
        --entrypoint /bin/bash)
    if [ -t 0 ]; then phase2_args+=(-it); fi
    docker_cmd "${phase2_args[@]}" "$phase2_image" "$@"
}

case "${1:-help}" in
    build)
        docker_cmd build --progress plain -t "$phase2_image" "$phase2_repo/tools/phase2"
        ;;
    check)
        run_container -c 'source /opt/ros/jazzy/setup.bash
            python3 /project/tools/phase2/check_sensors.py --seconds 20 \
                --scope "Jetson Isaac ROS container sensor contract; no flight acceptance" \
                --output /results/container_streams.json'
        ;;
    compile)
        run_container -c 'source /opt/ros/jazzy/setup.bash
            MAKEFLAGS=-j2 colcon build --executor sequential \
                --base-paths /project/src/autonomous_drone_perception \
                --packages-select autonomous_drone_perception --cmake-args -DBUILD_TESTING=ON'
        ;;
    test)
        run_container -c 'source /opt/ros/jazzy/setup.bash
            export ROS_DOMAIN_ID=72 ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST ROS_STATIC_PEERS=""
            colcon test --packages-select autonomous_drone_perception --event-handlers console_direct+
            colcon test-result --verbose'
        ;;
    run)
        run_container -c 'source /opt/ros/jazzy/setup.bash
            source install/local_setup.bash
            exec ros2 launch autonomous_drone_perception phase2.launch.py'
        ;;
    depth-check)
        run_container -c 'source /opt/ros/jazzy/setup.bash
            python3 /project/tools/phase2/check_depth.py --seconds 20 --output /results/depth_contract.json'
        ;;
    shell)
        run_container -c 'source /opt/ros/jazzy/setup.bash
            if [ -f install/local_setup.bash ]; then source install/local_setup.bash; fi
            exec bash'
        ;;
    inspect)
        docker_cmd image inspect "$phase2_image" --format '{{json .}}' > "$phase2_results/image.json"
        run_container -c 'dpkg-query -W > /results/packages.tsv
            cat /etc/nv_tegra_release 2>/dev/null || true
            ls -l /dev/nvhost-gpu /dev/nvmap 2>/dev/null || true
            source /opt/ros/jazzy/setup.bash
            ros2 pkg prefix isaac_ros_stereo_image_proc'
        ;;
    *)
        echo 'Usage: bash tools/phase2/container.sh {build|check|compile|test|run|depth-check|inspect|shell}'
        ;;
esac
