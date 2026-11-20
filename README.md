# Spark-I 自动任务赛

本轮实机进度、失败记录与待验收项见[开发进度与实机缺口](docs/integration-log.md)。

本仓库维护 ROS 1 Noetic 下的导航、视觉、机械臂与任务编排核心代码。各模块可独立开发，比赛入口负责接起完整流程。

机器人从 `/home/spark/SUAT/spark` 开发和运行。六模块的快捷命令见[模块开发入口](docs/模块开发入口.md)，机器人桌面可打开 `/home/spark/SUAT/cmd.txt`。

## 代码目录

| 模块 | 路径 | 职责 |
| --- | --- | --- |
| 导航 | [navigation/](src/suat_mission/src/suat_mission/navigation/) | A 点定位、路径规划与执行、底盘安全门控 |
| 视觉 | [vision/](src/suat_mission/src/suat_mission/vision/) | RGB-D 场景识别、分类板新帧投票 |
| 机械臂 | [manipulation/](src/suat_mission/src/suat_mission/manipulation/) | 抓取几何、机械臂控制和抓放反馈 |
| 自动化 | [automation/](src/suat_mission/src/suat_mission/automation/) | 任务状态机、读板站选择、单 Enter 控制和运行记录 |
| 运行配置 | [config/](src/suat_mission/config/) · [launch/](src/suat_mission/launch/) | ROS 参数、节点连接、RViz 显示 |
| 启动入口 | [tools/run_match.sh](tools/run_match.sh) · [tools/run_competition.py](tools/run_competition.py) | 准备并启动完整比赛流程 |

## 正式启动

在具备 ROS 1 Noetic、Spark-I 驱动、realsense2_camera、RViz 及已构建 catkin 工作区的机器人上，从项目根目录运行：

```bash
bash tools/run_match.sh cake-only --at-start-a --speed slow --planner rollout
```

将机器人预先摆在已标定的 A 区起点，朝 D。脚本提示后按一次 Enter。脚本自行启动或复用 ROS、定位、RGB-D 相机和导航，在有桌面显示时打开相机预览和 RViz，定位与任务准备通过后自动开始。蛋糕流程依次执行导航、停车读板、取物、运送和放置，并保留结果。启动准备计入本次时间预算。

RealVNC 中的 `Spark-I Mission Camera` 显示正式流程实际使用的图像。读板时可看板框、槽位、区域码、类别及投票结果。抓放时可看候选框、分数、机械臂坐标、顶面净空、持物状态，以及候选是否被发布或拒绝。画面底部标出帧年龄，阶段间恢复实时 RGB。读板最终标注图保存在该任务的 `board/<站位>/mission_overlay.png`。

两种局部规划模式共用当前配置中的 `global_planner/GlobalPlanner`、代价地图和碰撞范围。启动器与任务执行器按配置核对实际加载的插件。默认使用 Trajectory Rollout，也可以使用 DWA。

```bash
bash tools/run_match.sh cake-only --at-start-a --speed slow --planner dwa
```

`--planner dwa` 按控制周期内可达的速度窗口采样。`--planner rollout` 在预测时域内模拟加速，靠近目标时缩短预测时域。上面的现场验收命令使用 `slow`。当选择更高巡航档时，任务站的末段会切回低速。

只查看两种模式的参数时，可用 `python3 tools/start_field_navigation.py --describe --owner mission --speed fast --planner rollout`，将末尾换成 `dwa` 查看另一组。正式日志会记录每段实际生效的采样参数、里程计速度峰值及最后停稳误差。

## 整场模式与证据

`cake-only` 完成本局读板和蛋糕送 A 后结束。`full` 继续按分类板结果运送普通块。它使用独立的 39 cm 候选配置。

```bash
bash tools/run_match.sh full --at-start-a --speed slow --planner rollout --mission-config src/suat_mission/config/mission_full_pickup_39_candidate_20261119.yaml
```

分类板投票通过后，系统按西、南、东、北的候选顺序找同一物块。确认空手后可有限重试或换侧。抓取需要新的顶面观测，运输需要持物证据，放置需要同一物块脱离吸盘并稳定在正确区域。只有已确认放置的物块才计数。结果保存在 `reports/runs/<run_id>/result.json`。

## 本地现场资源

部署时保留现场已核验的选图配置、地图、起点和路线、读板与抓取标定，以及视觉所需模板。选图配置位于 `src/suat_mission/config/selected_map.yaml`，现场数据位于 `data/`，模板位于 `src/suat_mission/assets/templates/`。队友区域码资源位于 `src/suat_mission/zone/zone_markers/`，独立读板工具的模板位于同级 `_tpl/`。运行记录写入 `reports/`。这些资源由团队随实机工作区管理，GitHub 仓库保存代码版本。

正式导航使用与选图绑定的 `data/navigation/central_pickup_keepout.json` 定义中央方块区。场地版本和边界须与实物核对。全局与局部规划绕行该区，底盘输出同时按实时位姿与足迹检查。四方位取物候选须分别确认净空后才能切入正式路线。

正式串联读板支持在 `mission.yaml` 的 `board_view_candidate` 指定当前相机视角的候选四角及图像尺寸，原始板面几何文件保持不变；记录参考站仍使用原几何。候选仅适用于记录时的观察视角，机器人、相机或板面位置发生变化后必须重新核对。图案与 B/C/D 区域的方向仍使用原记录，应与现场区域确认；多帧投票通过不代表放置区物理方向已重新验证。


## 目录维护

- `src/suat_mission/src/suat_mission/` 下的四个模块目录是源码入口。`src/suat_mission/scripts/` 负责 ROS 启动和兼容调用。
- `tools/` 放当前可用的正式、专项验收与调试工具，`cmd.txt` 汇总真机命令。旧的蛋糕试跑入口已归档，蛋糕任务使用 `tools/run_match.sh cake-only`。
- `data/` 保存现场地图、标定和点位。`reports/` 保存实际运行证据。`build/`、`devel/` 供 catkin 构建和启动。
- 真机的 `.trash/` 保存淘汰源码和回退版本。归档路径和原路径见真机的 `.trash/moves.json`。
