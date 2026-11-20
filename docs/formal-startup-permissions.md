# 正式入口启动与机械臂权限

基线为 `dev/branch-2` 的 `bea7452`。验收对象是正式任务的 Prepare、Start、停止与串口交接。

## 原问题与调用链

`Node.command("prepare")` 派发 Prepare 线程，`Node.prepare()` 在 RunGate 仍处于 `idle` 时调用 `SerialRosRobot.preflight(preparing=True)`。原实现沿 `SerialMissionRuntime.open() → ArmSession.open() → _ask("M17")` 上力。此时 `robot.run_token` 为空，运行权限检查返回 false。Prepare 失败，`Node.command("start")` 又要求 Prepare 已成功，因此拿不到运行令牌。

在原基线上使用同一个 Mock 入口复现，`test_prepare_reaches_ready_through_serial_handshake` 的失败信息为 `preparation failed: competition action was stopped before arm command`。

修复后，Start 执行 `gate.start()`、启动记录并绑定令牌。唯一任务线程先调用 `SerialRosRobot.preflight()`，此时才执行 `ArmSession.enable_motion()`、核对空闲状态、重新读取臂位，以及等待新相机帧。通过后才调用 `Mission.run()`。因此 `_before_navigation()` 即使比 `bootstrap()` 更早收到移臂请求，也已经完成初始化。

## 权限状态

正式控制台仍使用 `state=idle` 和 `preparing/prepared` 标志识别准备状态。

| 阶段 | 修复前 | 修复后 |
| --- | --- | --- |
| Prepare | 串口打开即请求 `M17`，被运行门控拒绝 | 独占打开串口、查询电源、吸泵和臂位，设置 `M2120 V0` 禁止周期串口上报。拒绝 `M17`、`G0` 和开泵 |
| Ready | Start 等待准备成功 | 保留已准备的串口。只有一次有效 Start 可创建任务线程 |
| Running | 执行动作需要运行权限，串口初始化在 Prepare 提前触发 | 已绑定且未过期的令牌授权 `M17`。完成初始化并确认空闲后才授权任务动作。检查令牌与写入串口由同一个 RunGate 锁保护，等待回执时可响应 Stop |
| Stop 或启动失败 | 无令牌禁止主要动作，停止和清理可能与 Prepare 并发 | 先撤销权限。准备中由 Prepare 线程收尾，执行中仍可发送 `M2000`。已确认吸泵关闭时释放串口 |
| 需要恢复 | 可能持物时保留会话 | 同时保留未清理的所有权。Reset、Prepare 和 Start 都无法接管。操作员确认后可通过原恢复服务关泵或重试关闭 |

正式任务关闭时释放串口并延后原厂驱动重启。原厂驱动启动会自行发送 `M17`，在 Stop 清理中重启会绕开 RunGate。后续独立维护可由现场操作员受控启动原厂驱动。

## 改动文件

| 文件 | 改动与原因 |
| --- | --- |
| `automation/mission_node.py` | 区分准备预检与执行预检。重复请求保持同一控制者，失败撤销令牌，只有生命周期所属的线程关闭会话。清理失败时保留恢复入口 |
| `automation/run_gate.py` | 校验非空匹配令牌和截止时间。串口写入与停止撤销互斥 |
| `automation/serial_mission.py` | Prepare 保留只读打开的会话。Start 完成上力后刷新臂位，避免沿用准备时的位姿和读取时限 |
| `manipulation/arm_session.py` | 在串口打开前竞争互斥锁。分开设备握手和上力，限制动作写入并保留停止、显式关泵和串口清理 |
| `tests/test_formal_startup.py` | 使用 unittest、Mock 和模拟 SwiftPro 回执，驱动实际 ROS Adapter、RunGate 和 ArmSession 代码 |

代码文件位于 `src/suat_mission/src/suat_mission/`。

## 回归用例与结果

Windows 的 Python 3.12.14 执行以下命令，全部 15 项通过。四个修改的 Python 模块与新增测试通过 Python 3.8 语法解析。

```sh
python -m unittest discover -s tests -v
```

| 场景 | 结果 |
| --- | --- |
| Prepare 串口握手后进入 Ready | 通过 |
| Prepare 拒绝运动、开泵和上力 | 通过 |
| 有效 Start 进入 Running，先上力、刷新臂位，再放行首个 G0 | 通过 |
| 运行令牌缺失、错误或已过期时拒绝动作 | 通过 |
| 旧 run ID、重复 Start 和运行期间 Prepare/Reset | 通过 |
| Prepare 重复请求及准备导航阶段 Stop | 通过 |
| 串口上电阶段 Stop 可回收已确认空手的串口 | 通过 |
| Stop 中断机械臂回执等待，拒绝后续运动、开泵、上力，并保留 M2000 | 通过 |
| Prepare 串口初始化失败后可 Reset 重试 | 通过 |
| Start 的 M17 失败时不进入任务并释放串口 | 通过 |
| Start 记录建立失败或线程无法启动时回滚 | 通过 |
| 第二个执行器在打开串口前被拒绝 | 通过 |
| 停止清理不会重启自行上力的原厂驱动 | 通过 |
| 关闭失败保留所有权，等待显式恢复 | 通过 |
| 停止后只有显式恢复路径允许关泵 | 通过 |

## 待现场验收

Windows 测试替换了 ROS、SwiftPro 串口与 Linux 的 `flock`。ROS Noetic 上的实际握手、厂商进程交接、Linux 设备独占以及机械臂的物理表现仍待确认。

1. 现场执行端在有正式地图和标定的工作区部署此分支，按现有流程准备车辆和场地后，使用 `bash tools/run_match.sh cake-only --at-start-a --speed slow --planner rollout`，只按一次 Enter。
2. 核对同一 run ID 的准备、运行状态和 `manipulation/arm_serial_trace.json`。预期顺序为准备期间设备状态查询，Start 后一次 `M17`，随后重新读取臂位，最后才有首个 `G0`。
3. 在单独的现场轮次验收准备中停止和执行中停止。核对停止后不再写入普通动作，执行中发出 `M2000`，并根据物理状态核对串口释放或显式恢复。保留一次 Enter 的时间线和运行报告。
