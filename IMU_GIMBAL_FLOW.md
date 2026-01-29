# IMU 读取与云台控制流程说明

本文档用于汇报当前工程的 IMU 读取、姿态解算与云台控制流程。
对应代码主要在 `App/imu_task.c` 与 `App/gimbal_task.c`。

## 1. 总览

- IMU 传感器：BMI088（陀螺 + 加速度）
- 姿态融合：Mahony IMU（无磁力计），输出 `imuAngle[]`
- 云台控制：IMU 角度闭环 + 电机角度反馈 + 角速度前馈
- 任务频率：IMU 任务 1kHz（可配置 500Hz），云台任务 1kHz 基准，IMU 控制 500Hz

数据流简图（逻辑）：

```
BMI088_read() -> gyro/acc -> Mahony -> imuAngle[] -> gimbal_step_imu_execute()
                              ^
                              |  gyro 偏置校准
```

## 2. IMU 任务流程（`App/imu_task.c`）

### 2.1 初始化与零偏校准

1) BMI088 初始化
2) Mahony 四元数初始化
3) 陀螺静止零偏采样：
   - 采样 500 次（每次间隔 2ms）
   - 得到 `gyro_bias[3]`

### 2.2 运行循环（1kHz）

1) 读取 BMI088 原始数据
2) 进行零偏修正：`gyro -= gyro_bias`
3) Mahony 更新姿态：`MahonyAHRSupdateIMU(...)`
4) 解算角度：`imuAngle[yaw/pitch/roll]`（弧度）
5) 保持恒温控制（PWM）
6) `vTaskDelayUntil` 固定周期

关键可调参数：

- `IMU_TASK_PERIOD_TICKS`（1=1kHz, 2=500Hz）
- `GYRO_BIAS_SAMPLE_COUNT` / `GYRO_BIAS_SAMPLE_DELAY_MS`

注意事项：

- 当前为 **无磁力计**的 Mahony IMU 模式，Yaw 角仅靠陀螺积分，理论上会慢漂。
- 零偏校准能显著减小慢漂，但无法彻底消除长期漂移。

## 3. 云台任务调度（`App/gimbal_task.c`）

主循环基准频率为 1kHz（`vTaskDelayUntil(..., 1ms)`），在此基础上做分频：

- 500Hz：`gimbal_step_imu_execute()`（IMU 增稳控制）
- 200Hz：视觉逻辑 `gimbal_control_step_200hz()`
- 500Hz：CAN 角度查询（`gimbal_request_angle`）

## 4. Yaw 控制流程（IMU 闭环）

### 4.1 目标初始化

系统启动后，等待 IMU 稳定：

- `g_world_target_yaw = imu_yaw_current`
- `g_world_target_pitch = imu_pitch_current`

### 4.2 误差计算与环节

1) `yaw_world_error = target - imu_yaw`
2) 过零处理（-180~180）
3) 误差换向时清零积分
4) PID（P/I/D）计算速度
5) 角速度前馈（基于 gyro[2]）
6) 小误差时积分泄放
7) 输出限幅并下发电机速度

### 4.3 Yaw 可调参数

在 `App/gimbal_task.c` 可直接调：

- `g_yaw_kp, g_yaw_ki, g_yaw_kd`
- `g_yaw_integral_limit`
- `g_yaw_ff_gain, g_yaw_ff_sign`
- `g_yaw_deg_s_to_rpm`（直驱 1:1 时 = 1/6）
- `g_yaw_integral_decay` / `g_yaw_integral_decay_threshold`

### 4.4 Yaw 调试变量

- `dbg_yaw_error`：当前误差（度）
- `dbg_yaw_ff_rpm`：前馈输出（rpm）
- `dbg_yaw_speed_rpm`：最终输出（rpm）

## 5. Pitch 控制流程（IMU 闭环）

Pitch 采用“IMU 角度 -> 电机目标角”映射：

```
target_motor_pitch = base_motor - (imu_pitch - base_imu)
```

随后与电机实际角度做误差，进入 PID + 前馈控制。

Pitch 可调参数：

- `g_pitch_kp, g_pitch_ki, g_pitch_kd`
- `g_pitch_integral_limit`
- `g_pitch_ff_gain, g_pitch_ff_sign`
- `g_pitch_deg_s_to_rpm`

Pitch 调试变量：

- `dbg_pitch_error`
- `dbg_pitch_speed_rpm`
- `dbg_target_pitch`

## 6. 限幅与安全

- Pitch 限幅：`PITCH_LIMIT_DOWN / PITCH_LIMIT_UP`
- Yaw 角度限幅逻辑已移除（只保留速度限幅）
- 速度限幅：`GIMBAL_SAFE_MAX_SPEED_RPM`

## 7. 关键结论（用于汇报）

- IMU 融合使用 Mahony IMU，无磁力计，Yaw 依赖陀螺积分，存在慢漂。
- 通过静止零偏校准减少漂移；动态响应主要靠 PID + gyro 前馈。
- 采用 1kHz 基准任务 + 500Hz IMU 控制，实时性满足云台稳定需求。
- 通过积分泄放与换向清零积分解决停止时过冲问题。
