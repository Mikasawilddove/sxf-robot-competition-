#!/usr/bin/env bash
set -e
cd "$(dirname "$0")/.."
pickup_args=("$@")
set --
source /opt/ros/noetic/setup.bash
source /home/spark/spark_noetic/devel/setup.bash
source devel/setup.bash
exec python3 tools/west_static_pickup.py "${pickup_args[@]}"
