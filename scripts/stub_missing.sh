#!/bin/bash
# Auto-stub missing extensions until SimulationApp gets past dependency check

EXTS_DIR=/home/wukong/miniconda3/envs/isaacenv/lib/python3.12/site-packages/isaacsim/exts
PYTHON=/home/wukong/miniconda3/envs/isaacenv/bin/python

stub_ext() {
    local ext=$1
    local dir="$EXTS_DIR/$ext"
    if [ ! -d "$dir" ]; then
        mkdir -p "$dir/config"
        cat > "$dir/config/extension.toml" << EOF
[package]
version = "1.0.0"
title = "$ext (stub)"
description = "Stub to satisfy dependency"
writeTarget.kit = true

[dependencies]
EOF
        echo "[STUB] $ext"
    fi
}

for i in $(seq 1 30); do
    echo "=== Attempt $i ==="
    output=$($PYTHON /home/wukong/WALL_E/scripts/wall_e_perception.py 2>&1 | head -15)

    missing=$(echo "$output" | grep -oP 'No versions of \K[^\s]+' | head -1)

    if [ -z "$missing" ]; then
        echo "$output"
        echo "=== No dependency error — Isaac Sim got further ==="
        break
    fi

    stub_ext "$missing"
done
