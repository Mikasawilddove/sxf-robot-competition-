#!/usr/bin/env bash
set -e
cd "$(dirname "$0")/.."
# Catkin setup scripts read the current shell's positional arguments.
match_args=("$@")
set --
source /opt/ros/noetic/setup.bash
source /home/spark/spark_noetic/devel/setup.bash
source devel/setup.bash
exec python3 tools/run_competition.py "${match_args[@]}"
