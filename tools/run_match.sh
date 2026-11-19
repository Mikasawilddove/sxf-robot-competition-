#!/usr/bin/env bash
set -e
cd "$(dirname "$0")/.."
source /opt/ros/noetic/setup.bash
source /home/spark/spark_noetic/devel/setup.bash
source devel/setup.bash
exec python3 tools/run_competition.py "$@"
