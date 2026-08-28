#!/usr/bin/env bash
set -eo pipefail

source /opt/ros/jazzy/setup.bash

if ! ament_uncrustify "$@"; then
    ament_uncrustify --reformat "$@"
    echo "Reformatted in place — review with git diff and re-stage."
    exit 1
fi
