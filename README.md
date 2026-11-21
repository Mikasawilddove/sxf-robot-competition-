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
| 启动入口 | [tools/run_field_match.sh](tools/run_field_match.sh) · [tools/run_match.sh](tools/run_match.sh) | 从当前工作区启动比赛或单蛋糕验收 |

## 正式启动

本分支以真机 `spark-direct-board-e6e4276.FZEiBY` 的实际源码为基线。仓库收录执行代码，现场资源按原路径配置。使用 Ubuntu 20.04 / ROS 1 Noetic 和现有 Spark-I、SwiftPro、RealSense 驱动。在这份 checkout 内加载 Noetic 与底层驱动环境，再运行 `catkin_make`。部署到现有真机工作区时保留该工作区的现场资源。

```bash
bash tools/run_field_match.sh
```

将机器人摆在已标定的 A 区起点，朝 D。等待启动提示后按一次 Enter。正式入口固定选用这个工作区的 `path` 导航、`fast` 档和西侧抓取标定。启动程序会核对 `suat_mission` 解析到当前工作区。脚本启动相机、定位和任务节点，在有桌面时显示相机与 RViz。

正式流程先停车读分类板，将蛋糕送入 A 区并确认落区，再按本局 B/C/D 映射搬运熊、时钟和香蕉。单蛋糕试验仍可使用同一个入口：

```bash
bash tools/run_field_match.sh cake-only
```

蛋糕按实抓配套四系数求 XY，并使用 20 cm 标定面的实测接触 Z。三种普通块按自己的 RGB-D 顶面求抓取点，由控制器验证可达性。携带时使用吸盘附近的 RGB-D 证据，观测断流或确认空手时沿用停车逻辑。到放置站后继续使用原有的落区观测流程。

## 本地现场资源

部署时保留现场已核验的选图配置、地图、起点和路线、读板与抓取标定，以及视觉所需模板。选图配置位于 `src/suat_mission/config/selected_map.yaml`，现场数据位于 `data/`，模板位于 `src/suat_mission/assets/templates/`。运行记录写入 `reports/`。这些资源和凭据由团队另行管理，公开仓库提供可审查、可回退的代码版本。

正式入口读取本工作区 `data/calibration/cake_west_20cm_e6e4276/` 下配套的 `geometry.yaml`、`profile.yaml` 和 `thefile.txt`。几何配置应为同次部署版本，包含 `rgbd_pickup_labels: [clock, teddybear, banana]`。三个文件需要一起保留。

正式导航使用与选图绑定的 `data/navigation/central_pickup_keepout.json` 定义中央方块区。场地版本和边界须与实物核对。全局与局部规划绕行该区，底盘输出同时按实时位姿与足迹检查。四方位取物候选须分别确认净空后才能切入正式路线。

## 实机验证记录

真机运行 `04d34e3d92a8435c860f88c1a922be17` 完成读板、西侧抓起及持物确认，在前往 Collection_A 时因持物观测超时中止。对应的修复已在真机工作区编译，并用模拟输入验证了运输时的观测分支和普通块接触点计算。修复后的 A 区交付及三类普通块全程仍待真机验收。
