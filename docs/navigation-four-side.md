# 四侧导航开发与验收

基线为 `dev/branch-2`，现场资源来自已同步的 `corner_20261115_height_adjusted_A_03` 地图快照。运行以下离线检查时未连接 ROS，也未发送底盘指令。

## 调用关系

- 正式任务经 `run_match.sh`、`run_competition.py`、`Mission._nav()` 和 `RosRobot._navigate()` 到 `Navigator.execute_leg()`。
- 读板站对照经 `run_board_comparison.sh`、`run_competition.py --to-board` 到同一个 `Navigator.execute_leg()`。两个入口共享站位解析、阶段预算和到站判断。
- 独立现场单点由 `manual_navigation_test.py` 调用 `field_navigation.py`。明确选择执行时使用同一阶段预算。独立现场入口仍使用自己的导航 owner。用于比赛规划器公平对照时应走读板站对照入口。
- `field_navigation.py --offline-start-a` 和 `prepare_field_route.py --coverage` 直接读取保存地图。它们使用独立的静态圆形足迹规划器，无需 ROS。

原分支执行 `from suat_mission.navigation.navigation_gate import GOAL_PREFILTER_RADIUS_M` 重现了 `ImportError`。现行 live 代码已有 0.25 m 普通点预检下限，已集中到目标预检函数，继续与配置的底盘半径加 padding 取较大值。精确拾取仍只在匹配的 keepout 配置下使用原有的额外 0.01 m 目标净空。Windows 上已验证模块导入、独立入口 `--help`，以及从保存的 A 起点到西侧的离线路线。

## 导航预算

| 阶段 | 上限 | 专项与正式共用规则 |
| --- | ---: | --- |
| 初始离墙 | 25 s | 仅在足迹起点重叠且有合格离墙路径时执行 |
| 巡航或读板站 XY 到位 | 60 s | 取配置的 `navigation_timeout`、路程估计与剩余可分配时间中的最小值 |
| 末段对准或读板站转向 | 20 s | 从巡航额度中预留独立窗口，容差继续沿用现有值 |
| 每阶段停止和释放 | 5 s | 阶段运动截止早于阶段总截止，停止确认共用原执行器 |
| 单腿总上界 | 120 s | 含三个阶段各自停止。可提前结束。候选换站及有限重规划共用原截止时刻 |

正式任务从原有比赛时钟取得硬截止，`--to-board` 用本次 Enter 时间加比赛总时限、扣除已消耗时间和停车预留。两者的导航额度取阶段总上界和比赛硬截止中的较早值。普通动作继续使用原 `action_timeout`。超时报告带阶段、经过时间和最后可读的 map 位姿。未取得有效位姿时记录为空。

## 录点格式与使用

新录点使用 schema 2，每个点独立保存为 `data/waypoint_candidates/<session>/<station>.json`。字段包括 `name`、`pose=[x,y,yaw]`、`frame_id=map`、`child_frame_id=base_footprint`、`recorded_at_utc`、`source=live_tf`、`source_stamp_ns`、`map_identity`（包括几何、布局和对齐版本）、`map_image_sha256`、`checked_radius_m` 和 `validation_status`。保存时核对实时地图与保存地图的匹配、TF 新鲜度、足迹和中央限行区。同一会话的同名点不可覆盖。新录的 `Start_A` 是候选，贴墙时标记待验证离墙，正式 `start_pose_A.yaml` 仍供初始化。

以下为下次现场操作。停车后在 ROS Noetic 已加载的终端启动录点节点，再按现场实际位置逐点发布名称。

```bash
rosrun suat_mission waypoint_recorder.py _output_directory:=/home/spark/SUAT/spark/data/waypoint_candidates/session_01
rostopic pub -1 /suat/record_waypoint std_msgs/String "data: 'Sorting_S'"
rostopic pub -1 /suat/record_waypoint std_msgs/String "data: 'Sorting_E'"
```

支持 `Start_A`、`Classification_area`、`Sorting_W/S/E/N` 和 `Collection_A/B/C/D`。现有 schema 1 资源继续经地图身份检查。正式顺序为已录路线、补充站位、配置的四个接近点。新 schema 2 由覆盖工具显式读取用于对照，正式配置继续使用已选资源。

```bash
python3 tools/prepare_field_route.py --workspace-root /home/spark/SUAT/spark --coverage --output reports/station_coverage.json
python3 tools/prepare_field_route.py --workspace-root /home/spark/SUAT/spark --coverage \
  --mission-config src/suat_mission/config/mission_full_pickup_39_candidate_20261119.yaml \
  --candidate-directory data/waypoint_candidates/session_01 --output reports/station_candidates.json
```

## Rollout 和 DWA 对照

相同地图、限行区、A 起点、目标候选和 AMCL 选项；两个模式都使用 `global_planner/GlobalPlanner`、`DWAPlannerROS` 插件、slow 输出限制和同一 XY/朝向及停稳容差。比较模式的采样密度、仿真时长与评分参数均一致，运行开关 `use_dwa` 区分速度搜索。每次试验重新摆回同一 A 标记。保留所有失败及超时。

```bash
bash tools/run_board_comparison.sh rollout latest_recording baseline
bash tools/run_board_comparison.sh dwa latest_recording baseline
```

报告字段包括规划长度、里程计轨迹长度、经过时间、终止时误差、独立停稳读回误差、超时数、里程计转向反转次数和观察到的 `/cmd_vel` 最大消息间隔。反转次数是振荡指标，结合位置与停稳误差判断；消息间隔是接收端观测。未取得现场成对样本，因此未比较实际性能。新的阶段预算、录点 ROS 回调、真机局部规划轨迹、四侧载物通行和停稳精度均待验收。

## 离线证明

本轮新增 5 项集成回归，连同执行器、门控和导航几何的 46 项现有测试共 51 项通过。用两套任务配置分别生成 20 条路线。A 起点有静态贴墙，先验证有限离墙前缀；表中 A 路线的净空值从离墙交接点起算。末段对准只检查圆形足迹原地转向的静态空间。

## 四侧覆盖矩阵

### 默认 cake-only 配置

| Start | Goal | Static Plan | Footprint | Keepout | Status | 长度 m | 净空下界 m | 起点 → 终点 yaw rad | 末段 |
| --- | --- | --- | --- | --- | --- | ---: | ---: | --- | --- |
| Start_A | Sorting_W | 有 | START_CONTACT | CLEAR | STATIC_WITH_DEPARTURE | 1.621 | 0.046 | 0.577 → 2.174 | STATIC_CIRCULAR_TURN_CLEAR |
| Start_A | Sorting_S | 有 | START_CONTACT | CLEAR | STATIC_WITH_DEPARTURE | 2.682 | 0.035 | 0.577 → -2.581 | STATIC_CIRCULAR_TURN_CLEAR |
| Start_A | Sorting_E | 有 | START_CONTACT | CLEAR | STATIC_WITH_DEPARTURE | 2.584 | 0.034 | 0.577 → -0.965 | STATIC_CIRCULAR_TURN_CLEAR |
| Start_A | Sorting_N | 有 | START_CONTACT | CLEAR | STATIC_WITH_DEPARTURE | 1.578 | 0.064 | 0.577 → 0.613 | STATIC_CIRCULAR_TURN_CLEAR |
| Sorting_W | Collection_A | 有 | CLEAR | CLEAR | STATIC_PASS | 1.141 | 0.053 | 2.174 → -1.540 | STATIC_CIRCULAR_TURN_CLEAR |
| Sorting_W | Collection_B | 有 | CLEAR | CLEAR | STATIC_PASS | 2.079 | 0.041 | 2.174 → 2.913 | STATIC_CIRCULAR_TURN_CLEAR |
| Sorting_W | Collection_C | 有 | CLEAR | CLEAR | STATIC_PASS | 2.233 | 0.053 | 2.174 → 1.420 | STATIC_CIRCULAR_TURN_CLEAR |
| Sorting_W | Collection_D | 有 | CLEAR | CLEAR | STATIC_PASS | 1.060 | 0.053 | 2.174 → -0.155 | STATIC_CIRCULAR_TURN_CLEAR |
| Sorting_S | Collection_A | 有 | CLEAR | CLEAR | STATIC_PASS | 2.189 | 0.041 | -2.581 → -1.540 | STATIC_CIRCULAR_TURN_CLEAR |
| Sorting_S | Collection_B | 有 | CLEAR | CLEAR | STATIC_PASS | 2.297 | 0.041 | -2.581 → 2.913 | STATIC_CIRCULAR_TURN_CLEAR |
| Sorting_S | Collection_C | 有 | CLEAR | CLEAR | STATIC_PASS | 1.142 | 0.041 | -2.581 → 1.420 | STATIC_CIRCULAR_TURN_CLEAR |
| Sorting_S | Collection_D | 有 | CLEAR | CLEAR | STATIC_PASS | 1.082 | 0.041 | -2.581 → -0.155 | STATIC_CIRCULAR_TURN_CLEAR |
| Sorting_E | Collection_A | 有 | CLEAR | CLEAR | STATIC_PASS | 2.163 | 0.034 | -0.965 → -1.540 | STATIC_CIRCULAR_TURN_CLEAR |
| Sorting_E | Collection_B | 有 | CLEAR | CLEAR | STATIC_PASS | 1.140 | 0.059 | -0.965 → 2.913 | STATIC_CIRCULAR_TURN_CLEAR |
| Sorting_E | Collection_C | 有 | CLEAR | CLEAR | STATIC_PASS | 1.093 | 0.059 | -0.965 → 1.420 | STATIC_CIRCULAR_TURN_CLEAR |
| Sorting_E | Collection_D | 有 | CLEAR | CLEAR | STATIC_PASS | 2.146 | 0.055 | -0.965 → -0.155 | STATIC_CIRCULAR_TURN_CLEAR |
| Sorting_N | Collection_A | 有 | CLEAR | CLEAR | STATIC_PASS | 1.140 | 0.064 | 0.613 → -1.540 | STATIC_CIRCULAR_TURN_CLEAR |
| Sorting_N | Collection_B | 有 | CLEAR | CLEAR | STATIC_PASS | 1.091 | 0.064 | 0.613 → 2.913 | STATIC_CIRCULAR_TURN_CLEAR |
| Sorting_N | Collection_C | 有 | CLEAR | CLEAR | STATIC_PASS | 2.102 | 0.034 | 0.613 → 1.420 | STATIC_CIRCULAR_TURN_CLEAR |
| Sorting_N | Collection_D | 有 | CLEAR | CLEAR | STATIC_PASS | 2.066 | 0.041 | 0.613 → -0.155 | STATIC_CIRCULAR_TURN_CLEAR |

旧 `Classification_area` 主点足迹检查未通过。对应候选中，以下通过静态足迹与限行区检查。 `latest_recording`、`latest_backoff_5cm`。

### 39 cm 整场候选配置

| Start | Goal | Static Plan | Footprint | Keepout | Status | 长度 m | 净空下界 m | 起点 → 终点 yaw rad | 末段 |
| --- | --- | --- | --- | --- | --- | ---: | ---: | --- | --- |
| Start_A | Sorting_W | 有 | START_CONTACT | CLEAR | STATIC_WITH_DEPARTURE | 1.629 | 0.007 | 0.577 → 2.174 | STATIC_CIRCULAR_TURN_CLEAR |
| Start_A | Sorting_S | 有 | START_CONTACT | CLEAR | STATIC_WITH_DEPARTURE | 2.673 | 0.006 | 0.577 → -2.581 | STATIC_CIRCULAR_TURN_CLEAR |
| Start_A | Sorting_E | 有 | START_CONTACT | CLEAR | STATIC_WITH_DEPARTURE | 2.588 | 0.004 | 0.577 → -0.965 | STATIC_CIRCULAR_TURN_CLEAR |
| Start_A | Sorting_N | 有 | START_CONTACT | CLEAR | STATIC_WITH_DEPARTURE | 1.607 | 0.007 | 0.577 → 0.613 | STATIC_CIRCULAR_TURN_CLEAR |
| Sorting_W | Collection_A | 有 | CLEAR | CLEAR | STATIC_PASS | 1.148 | 0.007 | 2.174 → -1.540 | STATIC_CIRCULAR_TURN_CLEAR |
| Sorting_W | Collection_B | 有 | CLEAR | CLEAR | STATIC_PASS | 2.129 | 0.007 | 2.174 → 2.913 | STATIC_CIRCULAR_TURN_CLEAR |
| Sorting_W | Collection_C | 有 | CLEAR | CLEAR | STATIC_PASS | 2.245 | 0.007 | 2.174 → 1.420 | STATIC_CIRCULAR_TURN_CLEAR |
| Sorting_W | Collection_D | 有 | CLEAR | CLEAR | STATIC_PASS | 1.067 | 0.007 | 2.174 → -0.155 | STATIC_CIRCULAR_TURN_CLEAR |
| Sorting_S | Collection_A | 有 | CLEAR | CLEAR | STATIC_PASS | 2.181 | 0.006 | -2.581 → -1.540 | STATIC_CIRCULAR_TURN_CLEAR |
| Sorting_S | Collection_B | 有 | CLEAR | CLEAR | STATIC_PASS | 2.250 | 0.006 | -2.581 → 2.913 | STATIC_CIRCULAR_TURN_CLEAR |
| Sorting_S | Collection_C | 有 | CLEAR | CLEAR | STATIC_PASS | 1.156 | 0.006 | -2.581 → 1.420 | STATIC_CIRCULAR_TURN_CLEAR |
| Sorting_S | Collection_D | 有 | CLEAR | CLEAR | STATIC_PASS | 1.099 | 0.006 | -2.581 → -0.155 | STATIC_CIRCULAR_TURN_CLEAR |
| Sorting_E | Collection_A | 有 | CLEAR | CLEAR | STATIC_PASS | 2.163 | 0.004 | -0.965 → -1.540 | STATIC_CIRCULAR_TURN_CLEAR |
| Sorting_E | Collection_B | 有 | CLEAR | CLEAR | STATIC_PASS | 1.171 | 0.004 | -0.965 → 2.913 | STATIC_CIRCULAR_TURN_CLEAR |
| Sorting_E | Collection_C | 有 | CLEAR | CLEAR | STATIC_PASS | 1.136 | 0.004 | -0.965 → 1.420 | STATIC_CIRCULAR_TURN_CLEAR |
| Sorting_E | Collection_D | 有 | CLEAR | CLEAR | STATIC_PASS | 2.093 | 0.004 | -0.965 → -0.155 | STATIC_CIRCULAR_TURN_CLEAR |
| Sorting_N | Collection_A | 有 | CLEAR | CLEAR | STATIC_PASS | 1.163 | 0.007 | 0.613 → -1.540 | STATIC_CIRCULAR_TURN_CLEAR |
| Sorting_N | Collection_B | 有 | CLEAR | CLEAR | STATIC_PASS | 1.118 | 0.007 | 0.613 → 2.913 | STATIC_CIRCULAR_TURN_CLEAR |
| Sorting_N | Collection_C | 有 | CLEAR | CLEAR | STATIC_PASS | 2.104 | 0.007 | 0.613 → 1.420 | STATIC_CIRCULAR_TURN_CLEAR |
| Sorting_N | Collection_D | 有 | CLEAR | CLEAR | STATIC_PASS | 2.081 | 0.007 | 0.613 → -0.155 | STATIC_CIRCULAR_TURN_CLEAR |

旧 `Classification_area` 主点足迹检查未通过。对应候选中，以下通过静态足迹与限行区检查。 `latest_recording`、`latest_backoff_5cm`。
