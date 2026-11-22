#!/usr/bin/env bash
set -eo pipefail
source "$(dirname -- "${BASH_SOURCE[0]}")/environment.sh"
distance="${1:-2}"
case "$distance" in 1|2|4) ;; *) echo 'Use: bash run_sim.sh [1|2|4]' >&2; exit 2;; esac
world_file="$PHASE2_DIR/worlds/phase2_validation_${distance}m.sdf"
if [ ! -f "$world_file" ]; then
    echo 'Generate assets with python3 setup_assets.py first.' >&2
    exit 1
fi
mkdir -p "$PHASE2_DIR/results"
printf '%s\n' "$$" > "$PHASE2_DIR/results/sim.pid"
exec gz sim -s -r -v 2 "$world_file"
