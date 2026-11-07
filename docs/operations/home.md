# 正常退出后的 Home：定义、实现和证据

当前长期入口已经接入自动 Home；它是**原生力控安全结束后的独立位置伺服阶段**。故障退出不会自动运动回 Home。本文核对当前源码与现存记录，不新增控制或安全逻辑。

## 真实定义来源

| 来源 | 定义/作用 |
|---|---|
| [6pus_geometry.yaml](../../bone_ws/src/pus6_kinematics/config/6pus_geometry.yaml) | `actuator.operating_home_raw_steps = [1000]*6`；模型滑块 Home 为 `[0.05]*6 m` |
| [geometry_config.py](../../bone_ws/src/pus6_kinematics/pus6_kinematics/geometry_config.py) | 加载并校验当前 Home 契约 |
| [safety.py](../../bone_ws/src/pus6_motor_driver/pus6_motor_driver/safety.py) | 导出共享 `OPERATING_HOME_RAW_STEPS` |
| [run_j7_servo_home_park.py](../../bone_ws/scripts/run_j7_servo_home_park.py) | 实际 Home runner 的 `HOME_TARGET=(1000,1000,1000,1000,1000,1000)`；值一致，但不是动态加载 YAML |
| [mode3_auto_home.py](../../bone_ws/src/pus6_force_control/pus6_force_control/mode3_auto_home.py) | 调用已有 J7 runner，不自行构造新机构坐标 |

`STORAGE_PARK=[0]*6` 是收拢存放位，不是主动控制原点。Home 是软件工作参考，不是撞限位或设备零点校准。模型映射为 `q_model=0.05+(raw-1000)×0.000025 m`；名义平台高度约 132.612268 mm。YAML 中最终装配、零位、行程和 IMU 外参仍带待确认标记，应保留这种不确定性。

## 正常退出的串行交接

1. 连续模式收到 Ctrl+C，或原结果目录的正常停止请求。
2. 三进程原生力控制结束；要求 `operator_stopped`、`all_mode0_exit_confirmed=true`、`parent_error=null`。故障、启动异常和未确认停止均直接跳过 Home。
3. 关闭原控制进程和 IMU，完成已有离线分析，再由 Home runner 独占打开电机 UART。
4. 重新读取六轴实际位置/状态，执行原启动分类；不能用保存的初始 Home 数字替代实际硬件读回。
5. 六轴切换 Mode 1 Servo，50 Hz 五次平滑 `0x29` 位置轨迹到 `HOME_TARGET`，默认峰值 50 steps/s、保持 2 s。
6. 轮询反馈监测原故障、电流和跟踪检查；结束发送六轴 Pause，最终全轴状态读取并关闭串口。

轨迹 `s(τ)=10τ³−15τ⁴+6τ⁵`；非零位移时 `T=max(2 s,1.875×最大轴位移/设定峰值速度)`。反馈为轮询单轴（标称每轴约 8.33 Hz），不是六轴同时 50 Hz 采样。当前 runner 电流阈值 1500 mA、轮询位置误差超过 35 steps 停止；这些是既有位置返回入口的检查，不是重新加回 Mode 3 主动阶段的误差退出。

成功条件包括完成全部规划周期、运行记录无故障/超时、Pause 成功及最终最大位置误差 ≤5 steps（0.125 mm）。没有在计划保持结束后无限等待收敛，也没有单独全局壁钟超时；有限周期计划、串口超时和原异常处理构成当前返回过程的时间边界。

## 不能混淆的实现边界

- **Mode 0 确认发生在回 Home 之前。** Home 进入 Mode 1，结束 Pause，没有再次 Mode 0 读回确认；不得将原力控报告里的 `MODE0: True` 当成 Home 后的模式测量。
- 返回中的原故障/超时/中断走 Pause；不能据 Mock 测试泛化为所有故障都已真机验证。当前最终读回异常可回退到缓存位置，结果判定没有独立的六轴最终样本新鲜度标志；应结合完整日志判断到位证据，不能写成任意反馈失效都已被严密覆盖。
- Home 只保存摘要与控制台，没有原生力阶段同等级完整逐帧 UART 日志；原力控 `hardware_write_count` 不包含 Home。
- 同一正常退出代码还接受 `stop_request.json`，并非仅键盘信号；故障退出的排除条件以源码实际判断为准。

这些是本次只读核对发现的现有边界，本轮只记录，没有改代码、阈值或退出逻辑。

## 现有证据，不能外推

| 记录 | 事实 |
|---|---|
| [Mock 返回摘要](../../results/mode3_dual_imu/dry_run_20261107_161045_154475/home_recovery.json) | `hardware_mode=mock`、`HOME_CONFIRMED`；初始即 Home，只证明软件流程，不证明带负载返回 |
| [真机返回摘要](../../results/mode3_dual_imu/run_20261107_161516_844033/home_recovery.json) | `hardware_mode=active`；实际起点 `[980,793,825,1017,1210,1178]`，终点 `[999,998,998,1000,1000,1001]`，最大误差 2 steps、Pause 成功、约 10.090 s |
| [对应控制台](../../results/mode3_dual_imu/console/native_20261107_161516_bPovDF.log) | 记录 Mode 1 进入、494/494 周期、保持、六轴最终读取及 `HOME PASS` |
| [针对性测试源码](../../bone_ws/src/pus6_force_control/test/test_mode3_auto_home.py) | 提供正常/故障退出、Mock Home、故障和中断用例；本轮没有重跑，不把用例存在直接称为全部通过 |

该真机记录支持一次正常退出后返回到工程 Home 容差内，不能称为已完成多次循环、所有异常退出、第二次 Ctrl+C 或长期稳定验证。用户后续现场运行方式见[长期入口](long_running.md)。
