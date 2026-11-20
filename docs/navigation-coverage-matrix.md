# 完整导航覆盖矩阵与净空风险

基于已同步的赛场保存地图和现行 keepout，使用原 0.22 m 圆形足迹规划。未运行机器人。
每套配置均保留旧读板主点的结果。已录读板候选单独核对，供现场决定。

状态为 `STATIC_PASS`、`STATIC_WITH_DEPARTURE`、`FIELD_REVIEW_REQUIRED`、`BLOCKED` 或 `MISSING_RECORD`。
末段为圆形足迹在目标位姿原地转向的空间检查。A 贴墙离场的净空从交接点起算。
`FIELD_REVIEW_REQUIRED` 保留静态可规划证据，标记的净空下界低于 10 mm。

## 净空与阻挡风险

| 配置 | 主流程 37 条 | 换站 12 条 | A 直达 4 条 | 读板候选 |
| --- | --- | --- | --- | --- |
| 默认 cake-only | 32 静态通过，5 阻挡 | 12 静态通过 | 4 静态通过 | 10 静态通过。含最新实录、后退 5 cm |
| 39 cm 整场候选 | 5 阻挡，32 待复核 | 12 待复核 | 4 待复核 | 2 静态通过，8 待复核。含最新实录、后退 5 cm |

- 旧 `Classification_area` 主点的目标足迹或出站足迹与障碍重叠，牵涉 5 条主流程路线。旧 `active_route` 候选同样受阻。
- 默认配置两套读板候选各可接通 A 进站及四侧出站。纳入已可用的一个替代姿态后，37 条主流程均有静态路径。
- 39 cm 配置中 56 个结果低于复核线。包括 32 条主流程、12 条换站、4 条 A 直达、两个可用读板候选各自的 4 条出站路径。
- 39 cm 配置最小净空下界约 **3.7 mm**，取物端点和连接段以横向偏差、停稳误差及带载突出量为现场复核重点。

## 默认 cake-only

### 37 条主流程

| 起点 | 终点 | 静态路径 | 足迹 | Keepout | 状态 | 距离 m | 净空下界 mm | yaw 起→终 rad | 末段 |
| --- | --- | --- | --- | --- | --- | ---: | ---: | --- | --- |
| Start_A | Classification_area | 无 | BLOCKED | CLEAR | BLOCKED | — | — | 0.577 → -1.018 | 待核查 |
| Classification_area | Sorting_W | 无 | BLOCKED | CLEAR | BLOCKED | — | — | -1.018 → 2.174 | 待核查 |
| Classification_area | Sorting_S | 无 | BLOCKED | CLEAR | BLOCKED | — | — | -1.018 → -2.581 | 待核查 |
| Classification_area | Sorting_E | 无 | BLOCKED | CLEAR | BLOCKED | — | — | -1.018 → -0.965 | 待核查 |
| Classification_area | Sorting_N | 无 | BLOCKED | CLEAR | BLOCKED | — | — | -1.018 → 0.613 | 待核查 |
| Sorting_W | Collection_A | 有 | CLEAR | CLEAR | STATIC_PASS | 1.141 | 53.2 | 2.174 → -1.540 | 空间通过 |
| Sorting_W | Collection_B | 有 | CLEAR | CLEAR | STATIC_PASS | 2.079 | 40.8 | 2.174 → 2.913 | 空间通过 |
| Sorting_W | Collection_C | 有 | CLEAR | CLEAR | STATIC_PASS | 2.233 | 53.2 | 2.174 → 1.420 | 空间通过 |
| Sorting_W | Collection_D | 有 | CLEAR | CLEAR | STATIC_PASS | 1.060 | 53.2 | 2.174 → -0.155 | 空间通过 |
| Sorting_S | Collection_A | 有 | CLEAR | CLEAR | STATIC_PASS | 2.189 | 40.7 | -2.581 → -1.540 | 空间通过 |
| Sorting_S | Collection_B | 有 | CLEAR | CLEAR | STATIC_PASS | 2.297 | 40.7 | -2.581 → 2.913 | 空间通过 |
| Sorting_S | Collection_C | 有 | CLEAR | CLEAR | STATIC_PASS | 1.142 | 40.7 | -2.581 → 1.420 | 空间通过 |
| Sorting_S | Collection_D | 有 | CLEAR | CLEAR | STATIC_PASS | 1.082 | 40.7 | -2.581 → -0.155 | 空间通过 |
| Sorting_E | Collection_A | 有 | CLEAR | CLEAR | STATIC_PASS | 2.163 | 33.6 | -0.965 → -1.540 | 空间通过 |
| Sorting_E | Collection_B | 有 | CLEAR | CLEAR | STATIC_PASS | 1.140 | 58.9 | -0.965 → 2.913 | 空间通过 |
| Sorting_E | Collection_C | 有 | CLEAR | CLEAR | STATIC_PASS | 1.093 | 58.9 | -0.965 → 1.420 | 空间通过 |
| Sorting_E | Collection_D | 有 | CLEAR | CLEAR | STATIC_PASS | 2.146 | 55.0 | -0.965 → -0.155 | 空间通过 |
| Sorting_N | Collection_A | 有 | CLEAR | CLEAR | STATIC_PASS | 1.140 | 63.9 | 0.613 → -1.540 | 空间通过 |
| Sorting_N | Collection_B | 有 | CLEAR | CLEAR | STATIC_PASS | 1.091 | 63.9 | 0.613 → 2.913 | 空间通过 |
| Sorting_N | Collection_C | 有 | CLEAR | CLEAR | STATIC_PASS | 2.102 | 33.6 | 0.613 → 1.420 | 空间通过 |
| Sorting_N | Collection_D | 有 | CLEAR | CLEAR | STATIC_PASS | 2.066 | 40.8 | 0.613 → -0.155 | 空间通过 |
| Collection_A | Sorting_W | 有 | CLEAR | CLEAR | STATIC_PASS | 1.141 | 53.2 | -1.540 → 2.174 | 空间通过 |
| Collection_A | Sorting_S | 有 | CLEAR | CLEAR | STATIC_PASS | 2.189 | 40.7 | -1.540 → -2.581 | 空间通过 |
| Collection_A | Sorting_E | 有 | CLEAR | CLEAR | STATIC_PASS | 2.163 | 33.6 | -1.540 → -0.965 | 空间通过 |
| Collection_A | Sorting_N | 有 | CLEAR | CLEAR | STATIC_PASS | 1.140 | 63.9 | -1.540 → 0.613 | 空间通过 |
| Collection_B | Sorting_W | 有 | CLEAR | CLEAR | STATIC_PASS | 2.079 | 40.8 | 2.913 → 2.174 | 空间通过 |
| Collection_B | Sorting_S | 有 | CLEAR | CLEAR | STATIC_PASS | 2.297 | 40.7 | 2.913 → -2.581 | 空间通过 |
| Collection_B | Sorting_E | 有 | CLEAR | CLEAR | STATIC_PASS | 1.140 | 58.9 | 2.913 → -0.965 | 空间通过 |
| Collection_B | Sorting_N | 有 | CLEAR | CLEAR | STATIC_PASS | 1.091 | 63.9 | 2.913 → 0.613 | 空间通过 |
| Collection_C | Sorting_W | 有 | CLEAR | CLEAR | STATIC_PASS | 2.233 | 53.2 | 1.420 → 2.174 | 空间通过 |
| Collection_C | Sorting_S | 有 | CLEAR | CLEAR | STATIC_PASS | 1.142 | 40.7 | 1.420 → -2.581 | 空间通过 |
| Collection_C | Sorting_E | 有 | CLEAR | CLEAR | STATIC_PASS | 1.093 | 58.9 | 1.420 → -0.965 | 空间通过 |
| Collection_C | Sorting_N | 有 | CLEAR | CLEAR | STATIC_PASS | 2.102 | 33.6 | 1.420 → 0.613 | 空间通过 |
| Collection_D | Sorting_W | 有 | CLEAR | CLEAR | STATIC_PASS | 1.060 | 53.2 | -0.155 → 2.174 | 空间通过 |
| Collection_D | Sorting_S | 有 | CLEAR | CLEAR | STATIC_PASS | 1.082 | 40.7 | -0.155 → -2.581 | 空间通过 |
| Collection_D | Sorting_E | 有 | CLEAR | CLEAR | STATIC_PASS | 2.146 | 55.0 | -0.155 → -0.965 | 空间通过 |
| Collection_D | Sorting_N | 有 | CLEAR | CLEAR | STATIC_PASS | 2.066 | 40.8 | -0.155 → 0.613 | 空间通过 |

### 12 条有向备用换站

| 起点 | 终点 | 静态路径 | 足迹 | Keepout | 状态 | 距离 m | 净空下界 mm | yaw 起→终 rad | 末段 |
| --- | --- | --- | --- | --- | --- | ---: | ---: | --- | --- |
| Sorting_W | Sorting_S | 有 | CLEAR | CLEAR | STATIC_PASS | 1.054 | 40.7 | 2.174 → -2.581 | 空间通过 |
| Sorting_W | Sorting_E | 有 | CLEAR | CLEAR | STATIC_PASS | 1.952 | 33.6 | 2.174 → -0.965 | 空间通过 |
| Sorting_W | Sorting_N | 有 | CLEAR | CLEAR | STATIC_PASS | 0.994 | 40.8 | 2.174 → 0.613 | 空间通过 |
| Sorting_S | Sorting_W | 有 | CLEAR | CLEAR | STATIC_PASS | 1.054 | 40.7 | -2.581 → 2.174 | 空间通过 |
| Sorting_S | Sorting_E | 有 | CLEAR | CLEAR | STATIC_PASS | 1.103 | 40.7 | -2.581 → -0.965 | 空间通过 |
| Sorting_S | Sorting_N | 有 | CLEAR | CLEAR | STATIC_PASS | 2.020 | 40.7 | -2.581 → 0.613 | 空间通过 |
| Sorting_E | Sorting_W | 有 | CLEAR | CLEAR | STATIC_PASS | 1.952 | 33.6 | -0.965 → 2.174 | 空间通过 |
| Sorting_E | Sorting_S | 有 | CLEAR | CLEAR | STATIC_PASS | 1.103 | 40.7 | -0.965 → -2.581 | 空间通过 |
| Sorting_E | Sorting_N | 有 | CLEAR | CLEAR | STATIC_PASS | 0.988 | 33.6 | -0.965 → 0.613 | 空间通过 |
| Sorting_N | Sorting_W | 有 | CLEAR | CLEAR | STATIC_PASS | 0.994 | 40.8 | 0.613 → 2.174 | 空间通过 |
| Sorting_N | Sorting_S | 有 | CLEAR | CLEAR | STATIC_PASS | 2.020 | 40.7 | 0.613 → -2.581 | 空间通过 |
| Sorting_N | Sorting_E | 有 | CLEAR | CLEAR | STATIC_PASS | 0.988 | 33.6 | 0.613 → -0.965 | 空间通过 |

### 4 条 A 直达专项

| 起点 | 终点 | 静态路径 | 足迹 | Keepout | 状态 | 距离 m | 净空下界 mm | yaw 起→终 rad | 末段 |
| --- | --- | --- | --- | --- | --- | ---: | ---: | --- | --- |
| Start_A | Sorting_W | 有 | START_CONTACT | CLEAR | STATIC_WITH_DEPARTURE | 1.621 | 46.5 | 0.577 → 2.174 | 空间通过 |
| Start_A | Sorting_S | 有 | START_CONTACT | CLEAR | STATIC_WITH_DEPARTURE | 2.682 | 34.8 | 0.577 → -2.581 | 空间通过 |
| Start_A | Sorting_E | 有 | START_CONTACT | CLEAR | STATIC_WITH_DEPARTURE | 2.584 | 33.6 | 0.577 → -0.965 | 空间通过 |
| Start_A | Sorting_N | 有 | START_CONTACT | CLEAR | STATIC_WITH_DEPARTURE | 1.578 | 63.9 | 0.577 → 0.613 | 空间通过 |

### 读板候选 `latest_recording`

| 起点 | 终点 | 静态路径 | 足迹 | Keepout | 状态 | 距离 m | 净空下界 mm | yaw 起→终 rad | 末段 |
| --- | --- | --- | --- | --- | --- | ---: | ---: | --- | --- |
| Start_A | Classification_area__latest_recording | 有 | START_CONTACT | CLEAR | STATIC_WITH_DEPARTURE | 1.406 | 27.0 | 0.577 → -0.976 | 空间通过 |
| Classification_area__latest_recording | Sorting_W | 有 | CLEAR | CLEAR | STATIC_PASS | 0.361 | 53.2 | -0.976 → 2.174 | 空间通过 |
| Classification_area__latest_recording | Sorting_S | 有 | CLEAR | CLEAR | STATIC_PASS | 1.218 | 40.7 | -0.976 → -2.581 | 空间通过 |
| Classification_area__latest_recording | Sorting_E | 有 | CLEAR | CLEAR | STATIC_PASS | 2.083 | 33.6 | -0.976 → -0.965 | 空间通过 |
| Classification_area__latest_recording | Sorting_N | 有 | CLEAR | CLEAR | STATIC_PASS | 1.125 | 40.8 | -0.976 → 0.613 | 空间通过 |

### 读板候选 `latest_backoff_5cm`

| 起点 | 终点 | 静态路径 | 足迹 | Keepout | 状态 | 距离 m | 净空下界 mm | yaw 起→终 rad | 末段 |
| --- | --- | --- | --- | --- | --- | ---: | ---: | --- | --- |
| Start_A | Classification_area__latest_backoff_5cm | 有 | START_CONTACT | CLEAR | STATIC_WITH_DEPARTURE | 1.423 | 36.7 | 0.577 → -0.976 | 空间通过 |
| Classification_area__latest_backoff_5cm | Sorting_W | 有 | CLEAR | CLEAR | STATIC_PASS | 0.312 | 53.2 | -0.976 → 2.174 | 空间通过 |
| Classification_area__latest_backoff_5cm | Sorting_S | 有 | CLEAR | CLEAR | STATIC_PASS | 1.190 | 40.7 | -0.976 → -2.581 | 空间通过 |
| Classification_area__latest_backoff_5cm | Sorting_E | 有 | CLEAR | CLEAR | STATIC_PASS | 2.039 | 33.6 | -0.976 → -0.965 | 空间通过 |
| Classification_area__latest_backoff_5cm | Sorting_N | 有 | CLEAR | CLEAR | STATIC_PASS | 1.082 | 40.8 | -0.976 → 0.613 | 空间通过 |

### 读板候选 `active_route`

| 起点 | 终点 | 静态路径 | 足迹 | Keepout | 状态 | 距离 m | 净空下界 mm | yaw 起→终 rad | 末段 |
| --- | --- | --- | --- | --- | --- | ---: | ---: | --- | --- |
| Start_A | Classification_area__active_route | 无 | BLOCKED | CLEAR | BLOCKED | — | — | 0.577 → -1.018 | 待核查 |
| Classification_area__active_route | Sorting_W | 无 | BLOCKED | CLEAR | BLOCKED | — | — | -1.018 → 2.174 | 待核查 |
| Classification_area__active_route | Sorting_S | 无 | BLOCKED | CLEAR | BLOCKED | — | — | -1.018 → -2.581 | 待核查 |
| Classification_area__active_route | Sorting_E | 无 | BLOCKED | CLEAR | BLOCKED | — | — | -1.018 → -0.965 | 待核查 |
| Classification_area__active_route | Sorting_N | 无 | BLOCKED | CLEAR | BLOCKED | — | — | -1.018 → 0.613 | 待核查 |

## 39 cm 整场候选

### 37 条主流程

| 起点 | 终点 | 静态路径 | 足迹 | Keepout | 状态 | 距离 m | 净空下界 mm | yaw 起→终 rad | 末段 |
| --- | --- | --- | --- | --- | --- | ---: | ---: | --- | --- |
| Start_A | Classification_area | 无 | BLOCKED | CLEAR | BLOCKED | — | — | 0.577 → -1.018 | 待核查 |
| Classification_area | Sorting_W | 无 | BLOCKED | CLEAR | BLOCKED | — | — | -1.018 → 2.174 | 待核查 |
| Classification_area | Sorting_S | 无 | BLOCKED | CLEAR | BLOCKED | — | — | -1.018 → -2.581 | 待核查 |
| Classification_area | Sorting_E | 无 | BLOCKED | CLEAR | BLOCKED | — | — | -1.018 → -0.965 | 待核查 |
| Classification_area | Sorting_N | 无 | BLOCKED | CLEAR | BLOCKED | — | — | -1.018 → 0.613 | 待核查 |
| Sorting_W | Collection_A | 有 | CLEAR | CLEAR | FIELD_REVIEW_REQUIRED | 1.148 | 7.0 | 2.174 → -1.540 | 空间通过 |
| Sorting_W | Collection_B | 有 | CLEAR | CLEAR | FIELD_REVIEW_REQUIRED | 2.129 | 7.0 | 2.174 → 2.913 | 空间通过 |
| Sorting_W | Collection_C | 有 | CLEAR | CLEAR | FIELD_REVIEW_REQUIRED | 2.245 | 7.0 | 2.174 → 1.420 | 空间通过 |
| Sorting_W | Collection_D | 有 | CLEAR | CLEAR | FIELD_REVIEW_REQUIRED | 1.067 | 7.0 | 2.174 → -0.155 | 空间通过 |
| Sorting_S | Collection_A | 有 | CLEAR | CLEAR | FIELD_REVIEW_REQUIRED | 2.181 | 6.3 | -2.581 → -1.540 | 空间通过 |
| Sorting_S | Collection_B | 有 | CLEAR | CLEAR | FIELD_REVIEW_REQUIRED | 2.250 | 6.3 | -2.581 → 2.913 | 空间通过 |
| Sorting_S | Collection_C | 有 | CLEAR | CLEAR | FIELD_REVIEW_REQUIRED | 1.156 | 6.3 | -2.581 → 1.420 | 空间通过 |
| Sorting_S | Collection_D | 有 | CLEAR | CLEAR | FIELD_REVIEW_REQUIRED | 1.099 | 6.3 | -2.581 → -0.155 | 空间通过 |
| Sorting_E | Collection_A | 有 | CLEAR | CLEAR | FIELD_REVIEW_REQUIRED | 2.163 | 3.7 | -0.965 → -1.540 | 空间通过 |
| Sorting_E | Collection_B | 有 | CLEAR | CLEAR | FIELD_REVIEW_REQUIRED | 1.171 | 3.7 | -0.965 → 2.913 | 空间通过 |
| Sorting_E | Collection_C | 有 | CLEAR | CLEAR | FIELD_REVIEW_REQUIRED | 1.136 | 3.7 | -0.965 → 1.420 | 空间通过 |
| Sorting_E | Collection_D | 有 | CLEAR | CLEAR | FIELD_REVIEW_REQUIRED | 2.093 | 3.7 | -0.965 → -0.155 | 空间通过 |
| Sorting_N | Collection_A | 有 | CLEAR | CLEAR | FIELD_REVIEW_REQUIRED | 1.163 | 6.8 | 0.613 → -1.540 | 空间通过 |
| Sorting_N | Collection_B | 有 | CLEAR | CLEAR | FIELD_REVIEW_REQUIRED | 1.118 | 6.8 | 0.613 → 2.913 | 空间通过 |
| Sorting_N | Collection_C | 有 | CLEAR | CLEAR | FIELD_REVIEW_REQUIRED | 2.104 | 6.8 | 0.613 → 1.420 | 空间通过 |
| Sorting_N | Collection_D | 有 | CLEAR | CLEAR | FIELD_REVIEW_REQUIRED | 2.081 | 6.8 | 0.613 → -0.155 | 空间通过 |
| Collection_A | Sorting_W | 有 | CLEAR | CLEAR | FIELD_REVIEW_REQUIRED | 1.148 | 7.0 | -1.540 → 2.174 | 空间通过 |
| Collection_A | Sorting_S | 有 | CLEAR | CLEAR | FIELD_REVIEW_REQUIRED | 2.181 | 6.3 | -1.540 → -2.581 | 空间通过 |
| Collection_A | Sorting_E | 有 | CLEAR | CLEAR | FIELD_REVIEW_REQUIRED | 2.163 | 3.7 | -1.540 → -0.965 | 空间通过 |
| Collection_A | Sorting_N | 有 | CLEAR | CLEAR | FIELD_REVIEW_REQUIRED | 1.163 | 6.8 | -1.540 → 0.613 | 空间通过 |
| Collection_B | Sorting_W | 有 | CLEAR | CLEAR | FIELD_REVIEW_REQUIRED | 2.129 | 7.0 | 2.913 → 2.174 | 空间通过 |
| Collection_B | Sorting_S | 有 | CLEAR | CLEAR | FIELD_REVIEW_REQUIRED | 2.250 | 6.3 | 2.913 → -2.581 | 空间通过 |
| Collection_B | Sorting_E | 有 | CLEAR | CLEAR | FIELD_REVIEW_REQUIRED | 1.171 | 3.7 | 2.913 → -0.965 | 空间通过 |
| Collection_B | Sorting_N | 有 | CLEAR | CLEAR | FIELD_REVIEW_REQUIRED | 1.118 | 6.8 | 2.913 → 0.613 | 空间通过 |
| Collection_C | Sorting_W | 有 | CLEAR | CLEAR | FIELD_REVIEW_REQUIRED | 2.245 | 7.0 | 1.420 → 2.174 | 空间通过 |
| Collection_C | Sorting_S | 有 | CLEAR | CLEAR | FIELD_REVIEW_REQUIRED | 1.156 | 6.3 | 1.420 → -2.581 | 空间通过 |
| Collection_C | Sorting_E | 有 | CLEAR | CLEAR | FIELD_REVIEW_REQUIRED | 1.136 | 3.7 | 1.420 → -0.965 | 空间通过 |
| Collection_C | Sorting_N | 有 | CLEAR | CLEAR | FIELD_REVIEW_REQUIRED | 2.104 | 6.8 | 1.420 → 0.613 | 空间通过 |
| Collection_D | Sorting_W | 有 | CLEAR | CLEAR | FIELD_REVIEW_REQUIRED | 1.067 | 7.0 | -0.155 → 2.174 | 空间通过 |
| Collection_D | Sorting_S | 有 | CLEAR | CLEAR | FIELD_REVIEW_REQUIRED | 1.099 | 6.3 | -0.155 → -2.581 | 空间通过 |
| Collection_D | Sorting_E | 有 | CLEAR | CLEAR | FIELD_REVIEW_REQUIRED | 2.093 | 3.7 | -0.155 → -0.965 | 空间通过 |
| Collection_D | Sorting_N | 有 | CLEAR | CLEAR | FIELD_REVIEW_REQUIRED | 2.081 | 6.8 | -0.155 → 0.613 | 空间通过 |

### 12 条有向备用换站

| 起点 | 终点 | 静态路径 | 足迹 | Keepout | 状态 | 距离 m | 净空下界 mm | yaw 起→终 rad | 末段 |
| --- | --- | --- | --- | --- | --- | ---: | ---: | --- | --- |
| Sorting_W | Sorting_S | 有 | CLEAR | CLEAR | FIELD_REVIEW_REQUIRED | 1.059 | 6.3 | 2.174 → -2.581 | 空间通过 |
| Sorting_W | Sorting_E | 有 | CLEAR | CLEAR | FIELD_REVIEW_REQUIRED | 1.973 | 3.7 | 2.174 → -0.965 | 空间通过 |
| Sorting_W | Sorting_N | 有 | CLEAR | CLEAR | FIELD_REVIEW_REQUIRED | 1.018 | 6.8 | 2.174 → 0.613 | 空间通过 |
| Sorting_S | Sorting_W | 有 | CLEAR | CLEAR | FIELD_REVIEW_REQUIRED | 1.059 | 6.3 | -2.581 → 2.174 | 空间通过 |
| Sorting_S | Sorting_E | 有 | CLEAR | CLEAR | FIELD_REVIEW_REQUIRED | 1.046 | 3.7 | -2.581 → -0.965 | 空间通过 |
| Sorting_S | Sorting_N | 有 | CLEAR | CLEAR | FIELD_REVIEW_REQUIRED | 1.987 | 6.3 | -2.581 → 0.613 | 空间通过 |
| Sorting_E | Sorting_W | 有 | CLEAR | CLEAR | FIELD_REVIEW_REQUIRED | 1.973 | 3.7 | -0.965 → 2.174 | 空间通过 |
| Sorting_E | Sorting_S | 有 | CLEAR | CLEAR | FIELD_REVIEW_REQUIRED | 1.046 | 3.7 | -0.965 → -2.581 | 空间通过 |
| Sorting_E | Sorting_N | 有 | CLEAR | CLEAR | FIELD_REVIEW_REQUIRED | 0.991 | 3.7 | -0.965 → 0.613 | 空间通过 |
| Sorting_N | Sorting_W | 有 | CLEAR | CLEAR | FIELD_REVIEW_REQUIRED | 1.018 | 6.8 | 0.613 → 2.174 | 空间通过 |
| Sorting_N | Sorting_S | 有 | CLEAR | CLEAR | FIELD_REVIEW_REQUIRED | 1.987 | 6.3 | 0.613 → -2.581 | 空间通过 |
| Sorting_N | Sorting_E | 有 | CLEAR | CLEAR | FIELD_REVIEW_REQUIRED | 0.991 | 3.7 | 0.613 → -0.965 | 空间通过 |

### 4 条 A 直达专项

| 起点 | 终点 | 静态路径 | 足迹 | Keepout | 状态 | 距离 m | 净空下界 mm | yaw 起→终 rad | 末段 |
| --- | --- | --- | --- | --- | --- | ---: | ---: | --- | --- |
| Start_A | Sorting_W | 有 | START_CONTACT | CLEAR | FIELD_REVIEW_REQUIRED | 1.629 | 7.0 | 0.577 → 2.174 | 空间通过 |
| Start_A | Sorting_S | 有 | START_CONTACT | CLEAR | FIELD_REVIEW_REQUIRED | 2.673 | 6.3 | 0.577 → -2.581 | 空间通过 |
| Start_A | Sorting_E | 有 | START_CONTACT | CLEAR | FIELD_REVIEW_REQUIRED | 2.588 | 3.7 | 0.577 → -0.965 | 空间通过 |
| Start_A | Sorting_N | 有 | START_CONTACT | CLEAR | FIELD_REVIEW_REQUIRED | 1.607 | 6.8 | 0.577 → 0.613 | 空间通过 |

### 读板候选 `latest_recording`

| 起点 | 终点 | 静态路径 | 足迹 | Keepout | 状态 | 距离 m | 净空下界 mm | yaw 起→终 rad | 末段 |
| --- | --- | --- | --- | --- | --- | ---: | ---: | --- | --- |
| Start_A | Classification_area__latest_recording | 有 | START_CONTACT | CLEAR | STATIC_WITH_DEPARTURE | 1.406 | 27.0 | 0.577 → -0.976 | 空间通过 |
| Classification_area__latest_recording | Sorting_W | 有 | CLEAR | CLEAR | FIELD_REVIEW_REQUIRED | 0.376 | 7.0 | -0.976 → 2.174 | 空间通过 |
| Classification_area__latest_recording | Sorting_S | 有 | CLEAR | CLEAR | FIELD_REVIEW_REQUIRED | 1.211 | 6.3 | -0.976 → -2.581 | 空间通过 |
| Classification_area__latest_recording | Sorting_E | 有 | CLEAR | CLEAR | FIELD_REVIEW_REQUIRED | 2.054 | 3.7 | -0.976 → -0.965 | 空间通过 |
| Classification_area__latest_recording | Sorting_N | 有 | CLEAR | CLEAR | FIELD_REVIEW_REQUIRED | 1.100 | 6.8 | -0.976 → 0.613 | 空间通过 |

### 读板候选 `latest_backoff_5cm`

| 起点 | 终点 | 静态路径 | 足迹 | Keepout | 状态 | 距离 m | 净空下界 mm | yaw 起→终 rad | 末段 |
| --- | --- | --- | --- | --- | --- | ---: | ---: | --- | --- |
| Start_A | Classification_area__latest_backoff_5cm | 有 | START_CONTACT | CLEAR | STATIC_WITH_DEPARTURE | 1.423 | 36.7 | 0.577 → -0.976 | 空间通过 |
| Classification_area__latest_backoff_5cm | Sorting_W | 有 | CLEAR | CLEAR | FIELD_REVIEW_REQUIRED | 0.327 | 7.0 | -0.976 → 2.174 | 空间通过 |
| Classification_area__latest_backoff_5cm | Sorting_S | 有 | CLEAR | CLEAR | FIELD_REVIEW_REQUIRED | 1.183 | 6.3 | -0.976 → -2.581 | 空间通过 |
| Classification_area__latest_backoff_5cm | Sorting_E | 有 | CLEAR | CLEAR | FIELD_REVIEW_REQUIRED | 2.011 | 3.7 | -0.976 → -0.965 | 空间通过 |
| Classification_area__latest_backoff_5cm | Sorting_N | 有 | CLEAR | CLEAR | FIELD_REVIEW_REQUIRED | 1.056 | 6.8 | -0.976 → 0.613 | 空间通过 |

### 读板候选 `active_route`

| 起点 | 终点 | 静态路径 | 足迹 | Keepout | 状态 | 距离 m | 净空下界 mm | yaw 起→终 rad | 末段 |
| --- | --- | --- | --- | --- | --- | ---: | ---: | --- | --- |
| Start_A | Classification_area__active_route | 无 | BLOCKED | CLEAR | BLOCKED | — | — | 0.577 → -1.018 | 待核查 |
| Classification_area__active_route | Sorting_W | 无 | BLOCKED | CLEAR | BLOCKED | — | — | -1.018 → 2.174 | 待核查 |
| Classification_area__active_route | Sorting_S | 无 | BLOCKED | CLEAR | BLOCKED | — | — | -1.018 → -2.581 | 待核查 |
| Classification_area__active_route | Sorting_E | 无 | BLOCKED | CLEAR | BLOCKED | — | — | -1.018 → -0.965 | 待核查 |
| Classification_area__active_route | Sorting_N | 无 | BLOCKED | CLEAR | BLOCKED | — | — | -1.018 → 0.613 | 待核查 |
