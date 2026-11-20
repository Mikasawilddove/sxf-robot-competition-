# 完整比赛导航覆盖与现场验收

基线为 `codex2/navigation-four-side` 的 `e251c73`。上一阶段的[20 条路线报告](navigation-four-side.md)保留。第二阶段使用同一份已保存的赛场地图和同一静态规划器。完整结果见[覆盖矩阵与净空风险](navigation-coverage-matrix.md)。

## 覆盖范围

| 分组 | 条数 | 用途 |
| --- | ---: | --- |
| 主流程 | 37 | A 到读板站 1 条、读板站到四个取物站 4 条、取物到四个放置站 16 条、放置后返回取物 16 条 |
| 换站 | 12 | 四个取物站间的有向备用路线，去程与回程分别检查 |
| A 直达取物 | 4 | 复核上一阶段的专项路线 |
| 读板候选 | 每个 5 | 用原始候选姿态分别检查 A 进站和到四个取物站的出站路线 |

覆盖工具保留已选资源的原始语义名称。缺少记录的站点和路径显示 `MISSING_RECORD`，输出列出缺的是哪一端。位姿保留实录 yaw，末段静态检查使用原圆形足迹。A 贴墙的路线只有在现有离墙检查找到交接点时才有静态路径，净空下界从该交接点起算。

### 净空判定

`FIELD_REVIEW_REQUIRED` 表示静态规划器有候选路径，但保守净空下界低于 **10 mm**。这一离线标记沿用已有精确取物端点的 1 cm 预检余量作为筛查基准。实机安全余量要从现场定位误差、地图偏差、跟踪误差、障碍外形及带载轮廓中确定。即使超过筛查基准，仍需实机验收。报告保留原 `static_status`、最小净空下界、朝向和路径长度，标记不会改变 footprint、障碍、keepout、速度或任务门控。

优先复核 39 cm 配置的取物站和连接路线。核对物块外沿、吸取后的突出量及实际停稳偏差。若现场余量不足，从开阔一侧重录候选，再重跑同一矩阵。旧读板主点和两个净空通过的替代站各自有记录，最终仍需验证停稳后板框和映射投票。

## 录点候选如何进入正式资源

1. 将本次实录放在新的 `data/waypoint_candidates/<session>/<station>.json`。Schema 2 保存 map → base_footprint 的位姿和原始时间、yaw、来源、地图身份和图片 SHA256。同一会话同名文件只能新建。
2. 用 `--candidate-directory` 加载指定会话作离线对照。读取时核对完整地图几何、布局及对齐版本，再对照当前实际地图图片的 SHA256。格式或身份不符时退出。覆盖报告中的候选保留 `formal_resource=false`。
3. 每个拟采用站位须有对应实机验收证据。读板站需同时满足停稳、板框可见和有效投票。取物站核对末段驶入、停稳、目标可见和作业空间。放置站核对带载到站、停稳、真实落区和出站。A 继续使用专用初始化记录及现场朝向确认。
4. 人工批准后，以新文件名构建与现有消费端兼容的版本资源。沿用原有配置入口，读板使用已联动核验的 `recorded_route_file`、`board_station_records_file` 与板框几何。四个取物接近点用包含全部 W/S/E/N 的 `pickup_stations_file`。放置补充站走 `additional_waypoints_file`。先用独立任务配置试跑，再由现场负责人决定是否切换正式配置。原文件与回退配置保留。

当前正式入口继续读取此前选定的版本。单点 Schema 2 文件直接写进正式资源配置会被拒绝。现有记录的现场状态以原始证据为准。A 有静止初始化记录，读板替代站有旧版到站专项证据。四侧取物站的完整通过、39 cm 通行、带载到 B/C/D 落区及所有换站路线均缺本版本实机验收。覆盖报告中的 `FIELD_VALIDATION_NOT_INFERRED` 表示没有从静态结果推断现场成功。

## 专项、正式入口和 Codex1 接口

| 项目 | 对 e251c73 的审查 |
| --- | --- |
| 阶段预算 | 两条入口同用 `NavigationBudget` 与 `Navigator.execute_leg`。离墙 25 s、巡航默认 60 s、末段 20 s，每阶段最多 5 s 停稳和释放。 |
| 比赛总截止 | 正式任务使用原始比赛 deadline。读板对照从 Enter 起按同一总时限、启动消耗及停止预留计算。两个入口的局部预算均受该总截止约束。 |
| 读板换站与重规划 | 读板候选共享一次计算的导航 deadline。导航内部的有限重规划复用原 `ActionContext`。 |
| 取物换侧 | `Mission._find_any()` 每侧调用独立的 `_nav()`，因此单腿额度可以刷新。比赛 deadline 持续生效。若要求同一轮取物搜索复用单一阶段预算窗口，交 Codex1 在其 Mission 分支复用现有 `navigation_deadline` 参数。 |
| 取消、停止、释放 | 执行器请求停止并取消所属目标，按原阶段截止等待停稳。控制释放失败时保持失败状态，`can_try_next_station()` 拒绝换站。现场应核对取消确认、停稳读回、输出释放和下一腿才重新取权。 |
| 末段 | 共用阶段预算预留末段与停车。静态圆形足迹支持原地转向只证明平面空间，最终以新鲜的停稳位姿读回为准。 |

Codex1 可继续基于 `e251c73` 开发。本分支改动限于站点加载边界、覆盖报告、离线入口和对应测试。RunGate、ArmSession、机械臂初始化、Mission 启动权限、正式环境参数及 Mission 源文件均沿用基线。

## 下次现场验收顺序

1. 确认实际场地与保存地图一致，按 A 朝 D 恢复摆位。用同一读板候选、slow 档和 AMCL 配置分别做 Rollout、DWA 专项，记录到站、停稳、输出释放及取消行为。
2. 读板站通过后，逐侧验证读板站到 W/S/E/N。对存在 `FIELD_REVIEW_REQUIRED` 的路线先核对净空，再进入场地执行。以同一台机器人和同一配置衡量位置、朝向及转入误差。
3. 验证首个蛋糕往返。再从 A/B/C/D 各放置站回到取物站，按高频路线优先补齐载物与返程。随后单独验证发生阻塞或取物失败时的备用换站和剩余时间。
4. 只有取得证据后才更新候选状态或切换正式配置。回填运行 ID、地图版本、起终点、耗时、实际位姿、停稳和输出释放。

离线复现可从项目根目录运行下列命令，输出用全新的文件名保存。

```bash
python3 tools/prepare_field_route.py --coverage --output reports/coverage_stage2_cake.json
python3 tools/prepare_field_route.py --coverage \
  --mission-config src/suat_mission/config/mission_full_pickup_39_candidate_20261119.yaml \
  --output reports/coverage_stage2_39cm.json
```

## 测试结果

Windows 上完成 Python 3.8 语法兼容检查，并运行 **58 项导航测试，全部通过**。保留上一轮 51 项用例，其中执行器、门控和导航几何的 46 项来自现有本地测试目录。新增 7 项覆盖路线及方向、点位缺失、地图身份冲突、候选隔离、毫米级净空、读板替代站和共享截止时间。当前分支随仓库的用例可用以下命令复现。

```bash
python3 -m unittest discover -s tests -p 'test_navigation*.py' -v
```

现场范围为 NOT TESTED。实机局部规划、停稳误差、输出释放、带载净空及 Rollout/DWA 对照留待现场记录。

## 修改文件

| 文件 | 变化 |
| --- | --- |
| `src/suat_mission/src/suat_mission/navigation/stations.py` | 路线分组、缺失资源清单、候选与正式资源隔离 |
| `src/suat_mission/src/suat_mission/navigation/station_coverage.py` | 多组路线、读板候选和净空复核结果 |
| `tools/prepare_field_route.py` | 同一离线入口输出完整报告 |
| `tests/test_navigation_integration.py` | 原覆盖断言跟随扩展后的接口 |
| `tests/test_navigation_coverage_safety.py` | 路线拓扑、缺失、身份、候选隔离、净空和截止时间回归 |
| `docs/navigation-coverage-safety.md`、`docs/navigation-coverage-matrix.md` | 验收说明、矩阵与净空风险清单 |
