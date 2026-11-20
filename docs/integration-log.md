# Spark-I 开发进度与实机缺口

更新于 2026-11-20。此页记录开发分支二同步时的状态。实机的运行目录沿用机器人生成的名称，目录中的日期字段可能与现场日期不同。本文使用运行 ID 定位原始证据。

## 本轮同步

- 以真机 `/home/spark/SUAT/spark` 的现行源码为准，更新导航、视觉、机械臂、自动化与入口脚本。地图、四侧录点、现场标定、视觉模板和运行日志由团队随实机工作区维护。
- `run_match.sh` 保留一次 Enter。正式流程使用停车后的分类板多帧投票，接着从四侧候选取物，持物状态通过后运输，确认释放并稳定落区后计数。
- `Spark-I Mission Camera` 已接入正式入口，显示同一实拍帧上的板框、区域码、投票、物块候选、机械臂坐标及失败原因。真机绘图和 ROS 图像转换已检查。一次 Enter 后窗口实显待现场复核。
- 西侧专项与正式入口共用机械臂串口执行器。现行版本有上电初始化、指令回执与真实到位分别判断、受限速度回退和单段垂直抬升。正式 XY 抓取默认使用 DLT，官方平面标定保留对照入口。

## 证据与待完成项

| 模块 | 当前状态 | 真机证据与下一步 |
| --- | --- | --- |
| A 起点定位和 Rollout 导航 | VERIFIED（到读板站专项） | 两次通过 `a_to_board_acceptance_20261103_210242`、`a_to_board_acceptance_20261103_212021`。后一轮起点报告为 `scan_refined`，离墙段约 0.282 m，已确认到站、停车和释放控制权。正式串联历史运行 `competition_start_20261103_195038` 在读板站前超时。用同地图、同起点、同 slow 档补 DWA 实测，再比较巡航速度和精确到站。 |
| 分类板和区域码 | PARTIAL | 正式读板已调用队友的区域码解码。在三个有效区域码可确认槽位时采用其方向，缺失时按已核实的槽位顺序处理。正式图案判定走模板匹配和跨帧投票。下一轮保存最终叠加图，核对本局 B/C/D 映射实际进入状态机并触发取物导航。 |
| 多物块与抓取坐标 | PARTIAL | 队友的独立 `test_sorting_vision.sh all` 支持同帧四类、多实例和深度层数候选。正式抓取使用现有 BRISK、HSV 与 RGB-D 顶面和跟踪链。需在同一组西、南、东、北原图对比漏检、重复框、标签和顶面误差，再决定接入多实例候选。YOLO 属于候选路线。 |
| 西侧静止抓取与带载 | FAILED（最近已读实测） | `west_static_pickup_20261103_191035_093213` 进入接近阶段后视觉对齐未通过。 `west_static_pickup_20261103_192831_370145` 的工作位误差为 26.6 mm。后一次 `west_static_pickup_20261103_215038_295501` 在 `observe_objects` 未按时停稳，`grasp_confirmed=false`。优先确认观察位实测收敛，再确认接触、持物和单段抬升。 |
| 蛋糕闭环与整场 | NOT TESTED（当前版本完整实物链） | 合法 A 起跑后，一次 Enter 串联“读板 → 取物导航 → 真正吸起 → 运输 → 落入 A → 同一蛋糕计数一次”。西侧实物抓取通过后，即做这条验收。再测四侧抓取覆盖和 B/C/D 各收取区的实物释放、必要的区域兜底与计时。 |

## 复现入口

```bash
cd /home/spark/SUAT/spark
bash tools/run_match.sh cake-only --at-start-a --speed slow --planner rollout
bash tools/run_board_comparison.sh rollout latest_recording baseline
bash tools/run_board_comparison.sh dwa latest_recording baseline
bash tools/run_west_static_pickup.sh --help
bash tools/test_sorting_vision.sh all
```

运行后记录配置、运行 ID、首个失败阶段、真实到位或持物证据、最终实物落区及计数。导航专项看 `reports/runs/a_to_board_acceptance_*/result.json`。正式任务看 `reports/runs/<run_id>/result.json` 与启动目录的 `resource_selection.json`、`start_localization.json` 和 `camera_view.log`。读板叠加图位于任务目录的 `board/<站位>/mission_overlay.png`，区域码图为同目录的 `zone_markers_overlay.png`。
