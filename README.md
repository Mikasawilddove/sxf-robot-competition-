# Spark-I 自动任务赛

本仓库维护 ROS 1 Noetic 下的导航、视觉、机械臂与任务编排核心代码。各模块可独立开发，比赛入口负责接起完整流程。

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
bash tools/run_match.sh cake-only --at-start-a --speed fast --planner rollout
```

将机器人预先摆在已标定的 A 区起点，朝 D。脚本提示后按一次 Enter。脚本自行启动或复用 ROS、定位、RGB-D 相机和导航，在有桌面显示时打开相机预览和 RViz，定位与任务准备通过后自动开始。蛋糕流程依次执行导航、停车读板、取物、运送和放置，并保留结果。启动准备计入本次时间预算。

正式入口的两种局部规划模式共用全局规划器、代价地图、碰撞范围和速度档。现场配置使用 `global_planner/GlobalPlanner`。启动器与任务执行器都按导航配置核对实际加载的插件。默认使用 Trajectory Rollout，DWA 可用下面的命令启动同一任务。

```bash
bash tools/run_match.sh cake-only --at-start-a --speed fast --planner dwa
```

`--planner dwa` 按控制周期内可达的速度窗口采样。`--planner rollout` 在预测时域内模拟加速。两种模式均以 `fast` 巡航，接近任务站时停车切到 `slow`。Rollout 在靠近目标时缩短预测时域，以保留直行微调候选。速度和精度仍待实机验收。

只查看两种模式的参数时，可用 `python3 tools/start_field_navigation.py --describe --owner mission --speed fast --planner rollout`，将末尾换成 `dwa` 查看另一组。正式日志会记录每段实际生效的采样参数、里程计速度峰值及最后停稳误差。

## 整场任务怎样推进

`cake-only` 完成读板和蛋糕送 A 后结束。`full` 沿用同一个 Enter 入口，接着按本局读板结果配送普通块。现场启用 `full` 前要核对选用的取物站、B/C/D 放置站和各区放置资源。

| 阶段 | 脚本进入下一步的条件 | 条件未满足时 |
| --- | --- | --- |
| 停车读板 | 本局的新图像投票通过，时钟、熊、香蕉分别对应 B/C/D 中的一个区 | 单站票数不足或冲突时尝试下一候选站。结果属于其他局或所有站都失败时结束 |
| 选物与抓取 | 当前取物站看到同一物块 ID，顶面可抓且无遮挡。串口后端在接近前重拍，抬升后用连续帧检查物块随吸盘移动 | 确认空手才能重试。若机械臂从该侧够不到，优先尝试配置中的下一侧。持物不明时结束 |
| 运输与放置 | 运输携带抓取时的物块 ID，到站停车后再次确认持物。释放后连续看到同一物块处于目标区、与吸盘分离且位置稳定，才累计该物块 | 途中失去持物证据或落区未确认时停止，本次物块不计数 |
| 整场收尾 | 蛋糕优先送 A。普通块在当前站的可抓候选中按配送代价选择，出发前和实际抓取前各判断一次剩余时间 | 时间不足时保留已确认的配送结果。蛋糕专项抓取失败即结束；整场模式按配置在确认空手后继续普通块 |

取物站顺序来自 `mission.yaml` 的 `sorting_points`。四侧配置优先西侧，然后南、东、北。程序在找到可抓目标后进入抓取。现场文件 `data/waypoints/pickup_approaches_20261119.json` 保存四侧实录位姿、沿车头反方向退开的距离和入口使用的候选位姿。重新建图时应重新核对该文件的地图绑定、禁行区和站位净空。读板站、放置站继续取自各自原有的现场记录。

蛋糕专项从 A 朝 D 起跑。现场验收时，核对同一运行目录的 `result.json` 中 `scope=cake-only`、`execution=completed`、`delivered_confirmed.cake=1`，其余类别均为零。用 `report.actions` 和 `report.objects` 核对同一蛋糕 ID 的已确认抓取、运送和一次已确认放置，终态为 `placed_confirmed`、区域为 A；同时核实蛋糕实际落在 A 红区内。

运行后查看 `reports/runs/<run_id>/result.json`。先看 `report.phase` 和 `report.error` 定位失败阶段，再用 `report.board_evidence`、`report.actions` 和 `report.objects` 核对本局映射、动作和每个物块的最终状态。离线可用以下命令演练指定的分类板排列。

```bash
PYTHONPATH=src/suat_mission/src python3 -m suat_mission.automation.simulator --board clock,teddybear,banana
```

## 本地现场资源

部署时保留现场已核验的选图配置、地图、起点和路线、读板与抓取标定，以及视觉所需模板。选图配置位于 `src/suat_mission/config/selected_map.yaml`，现场数据位于 `data/`，模板位于 `src/suat_mission/assets/templates/`。运行记录写入 `reports/`。这些资源和凭据由团队另行管理，公开仓库提供可审查、可回退的代码版本。

正式导航使用与选图绑定的 `data/navigation/central_pickup_keepout.json` 定义中央方块区。场地版本和边界须与实物核对。全局与局部规划绕行该区，底盘输出同时按实时位姿与足迹检查。四方位取物候选须分别确认净空后才能切入正式路线。
