#!/usr/bin/env bash
# 单帧分拣识别测试：默认四类，也可指定 banana、clock、teddybear、cake。
set -eo pipefail

workspace=/home/spark/SUAT/spark
target=${1:-all}
if [[ $# -gt 0 ]]; then shift; fi

case "$target" in
  all|banana|clock|teddybear|cake) ;;
  *) printf '用法：%s [all|banana|clock|teddybear|cake] [--no-window]\n' "$0" >&2; exit 2 ;;
esac

if [[ ! -f "$workspace/devel/setup.bash" ]]; then
  printf '找不到 ROS 工作区环境：%s/devel/setup.bash\n' "$workspace" >&2
  exit 1
fi
source "$workspace/devel/setup.bash"
export DISPLAY=:0
export XAUTHORITY=/home/spark/.Xauthority
exec rosrun suat_mission field_perception_node.py --target "$target" "$@"
