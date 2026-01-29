# Gimbal Speed Mode + Dual-Axis IMU Stabilization

## Context

### Original Request
用户需要重构 gimbal_task.c：
1. 将 JC4010 电机控制模式从位置梯形模式切换到速度模式
2. Yaw 和 Pitch 双轴都实现 IMU 世界坐标增稳（当前只有 Yaw 部分实现）
3. 禁用视觉跟踪逻辑（保留代码但不累加到世界目标）
4. 清理未使用的冗余代码
5. 保留 chassis_turning_mode 代码但不修改

### Interview Summary
**Key Discussions**:
- 确认使用速度模式 (0x0060=0x01)，PID 输出速度指令而非位置指令
- Pitch 轴 IMU 增稳采用与 Yaw 相同的架构：世界目标角 - IMU当前角 = 误差 → PID → 速度
- 视觉数据继续接收（维持 chassis_turning_mode 命令解析），但不累加到 g_world_target_*
- 代码清理是次要目标，确保功能正确后再清理

**Research Findings**:
- `gimbal_send_speed()` 函数存在 (Line 393-402) 但从未被调用
- Pitch 当前直接使用 `g_world_target_pitch` 而无 IMU 补偿 (Line 651)
- 位置梯形模式在 `gimbal_axis_enable()` 中设置 (Line 371)
- IMU 数据来自 `imu_task.c`，约 1000Hz 更新

### Metis Review
**Identified Gaps** (addressed):
- 速度模式是否需要持续发送命令 → 是，必须持续发送，否则电机可能停止
- Pitch IMU 轴对齐问题 → 使用与 Yaw 相同的模式，初次测试时验证符号
- `gimbal_send_speed()` 是否正确 → 代码符合 JC4010 协议，但需实际验证
- IMU 零点初始化 → Pitch 需要与 Yaw 相同的初始化逻辑

---

## Work Objectives

### Core Objective
将云台控制从位置梯形模式切换到速度模式，并为 Yaw 和 Pitch 双轴实现 IMU 世界坐标增稳，使云台在底盘移动/倾斜时保持指向固定点。

### Concrete Deliverables
- `gimbal_task.c` 修改：速度模式初始化 + 双轴 IMU 闭环 + 视觉禁用 + 代码清理
- `gimbal_task.h` 清理：移除未使用的宏定义

### Definition of Done
- [ ] 电机初始化后进入速度模式（CAN 抓包显示 0x0060=0x01）
- [ ] Yaw 轴 IMU 增稳有效：旋转底盘时云台自动反向补偿
- [ ] Pitch 轴 IMU 增稳有效：倾斜底盘时云台自动反向补偿
- [ ] 视觉数据不影响云台朝向：发送视觉坐标时云台不移动
- [ ] 代码编译无警告 (`-Wall`)
- [ ] 所有未使用函数/变量已删除

### Must Have
- 速度模式初始化 (0x0060=0x01)
- Yaw IMU 闭环控制（现有逻辑适配速度模式）
- Pitch IMU 闭环控制（新增）
- 视觉累加逻辑禁用
- 未使用代码清理

### Must NOT Have (Guardrails)
- ❌ 不修改 `imu_task.c` 或 `can_bsp.c`
- ❌ 不修改 `chassis_turning_mode` 相关逻辑 (Lines 827-853)
- ❌ 不添加陀螺仪前馈或其他新功能
- ❌ 不删除 `gimbal_send_position_trapezoid()` 直到速度模式验证完成
- ❌ 不调整 PID 参数（保持占位值，后续调参）
- ❌ 速度指令不超过 ±30 RPM (即 ±3000 raw)

---

## Verification Strategy (MANDATORY)

### Test Decision
- **Infrastructure exists**: NO (嵌入式项目，无自动化测试)
- **User wants tests**: Manual-only
- **Framework**: none

### Manual QA Procedures

**Each TODO 包含以下手动验证步骤：**

| 验证类型 | 工具 | 方法 |
|----------|------|------|
| CAN 通信 | 逻辑分析仪/CAN调试器 | 抓包验证帧格式 |
| IMU 响应 | UART串口 (huart7) | 打印角度变化 |
| 电机行为 | 物理观察 | 手动旋转底盘，观察云台补偿 |

---

## Task Flow

```
┌─────────────────────────────────────────────────────────────────────────┐
│                           EXECUTION FLOW                                │
├─────────────────────────────────────────────────────────────────────────┤
│                                                                         │
│  Task 1: Speed Mode Init                                                │
│      │                                                                  │
│      ▼                                                                  │
│  Task 2: Yaw Speed Control (convert existing IMU stabilization)         │
│      │                                                                  │
│      ▼                                                                  │
│  Task 3: Pitch IMU Stabilization (add new)                              │
│      │                                                                  │
│      ▼                                                                  │
│  Task 4: Disable Vision Tracking                                        │
│      │                                                                  │
│      ▼                                                                  │
│  Task 5: Code Cleanup                                                   │
│                                                                         │
└─────────────────────────────────────────────────────────────────────────┘
```

## Parallelization

| Task | Depends On | Reason |
|------|------------|--------|
| 1 | - | 基础设施 |
| 2 | 1 | 需要速度模式生效 |
| 3 | 2 | 复用 Yaw 的控制架构 |
| 4 | 3 | 确保控制正确后再禁用视觉 |
| 5 | 4 | 最后清理 |

**所有任务串行执行，每个任务完成后验证再进行下一个。**

---

## TODOs

### - [ ] 1. Speed Mode Initialization

**What to do**:
1. 在 `gimbal_axis_enable()` 中修改模式命令：
   - 将 `position_trap_mode_cmd` 从 `{0x2B, 0x00, 0x60, 0x00, 0x00, 0x02, ...}` 改为 `{0x2B, 0x00, 0x60, 0x00, 0x00, 0x01, ...}`
   - 命名改为 `speed_mode_cmd`
2. 移除初始化时的 `gimbal_send_position_trapezoid()` 调用
3. 上电后发送一次 `speed=0` 确保电机静止

**Must NOT do**:
- 不修改 `enable_cmd` (0x00A2=0x01)，这是使能命令
- 不修改 CAN ID 或句柄配置

**Parallelizable**: NO (基础任务)

**References**:
- `gimbal_task.c:368-382` - 当前 `gimbal_axis_enable()` 实现
- `gimbal_task.c:371` - 位置梯形模式命令定义
- JC4010 协议：速度模式 = 0x0060 寄存器写入 0x01

**Acceptance Criteria**:

**Manual Execution Verification:**
- [ ] 编译通过，无警告
- [ ] 上电后 CAN 抓包显示：`01 06 00 60 00 01 XX XX` (ID=0x601 for Yaw)
- [ ] 电机上电后静止不动（speed=0 生效）
- [ ] 手动转动电机轴，感觉阻力正常（速度模式下有阻尼）

**Commit**: YES
- Message: `feat(gimbal): switch to speed mode initialization`
- Files: `App/gimbal_task.c`

---

### - [ ] 2. Yaw Axis Speed Control Conversion

**What to do**:
1. 在 `gimbal_step_imu_execute()` 中修改 Yaw 控制：
   - 保留 IMU 误差计算逻辑 (Lines 621-629)
   - 将误差通过 PID 转换为速度输出 (rpm)
   - 调用 `gimbal_send_speed()` 替代 `gimbal_send_position_trapezoid()`
2. 创建或复用 Yaw 速度 PID 结构体：
   - 初始参数：Kp=5.0, Ki=0.0, Kd=0.0（占位，待调参）
   - 输出限幅：±30 RPM (即 raw ±3000)
3. 确保 500Hz 持续发送速度命令

**Must NOT do**:
- 不删除 `gimbal_send_position_trapezoid()` 函数（保留备用）
- 不修改 YAW 机械限位逻辑 (Lines 637-647)，但需要适配速度模式

**Parallelizable**: NO (依赖 Task 1)

**References**:
- `gimbal_task.c:603-657` - 当前 `gimbal_step_imu_execute()` 实现
- `gimbal_task.c:621-629` - Yaw 误差计算逻辑
- `gimbal_task.c:393-402` - `gimbal_send_speed()` 函数
- `gimbal_task.c:491-512` - `pid_calc()` 函数

**Acceptance Criteria**:

**Manual Execution Verification:**
- [ ] 编译通过，无警告
- [ ] CAN 抓包显示 Yaw 轴发送速度指令：`01 23 00 21 00 XX XX XX XX`
- [ ] 手动旋转底盘（改变 IMU yaw），云台 Yaw 电机反向转动补偿
- [ ] 停止旋转底盘，云台 Yaw 稳定在初始朝向
- [ ] 速度值不超过 ±3000 (±30 RPM)

**Commit**: YES
- Message: `feat(gimbal): convert Yaw to speed control with IMU stabilization`
- Files: `App/gimbal_task.c`

---

### - [ ] 3. Pitch Axis IMU Stabilization

**What to do**:
1. 在 `gimbal_step_imu_execute()` 中添加 Pitch IMU 闭环：
   - 读取 `imuAngle[INS_PITCH_ADDRESS_OFFSET]` 并转换为度数
   - 计算误差：`pitch_world_error = g_world_target_pitch - imu_pitch_current`
   - PID 计算速度输出
   - 调用 `gimbal_send_speed()` 发送到 Pitch 电机
2. 添加 Pitch 世界目标初始化：
   - 在 `g_target_initialized` 块中添加 `g_world_target_pitch = imu_pitch_current`
3. 创建 Pitch 速度 PID 结构体：
   - 初始参数：Kp=5.0, Ki=0.0, Kd=0.0
   - 输出限幅：±30 RPM

**Must NOT do**:
- 不修改 Pitch 软限位逻辑，但需要在速度输出前检查
- 不修改 IMU 数据读取方式

**Parallelizable**: NO (依赖 Task 2 架构)

**References**:
- `gimbal_task.c:603-657` - Yaw IMU 控制逻辑（复制模式）
- `gimbal_task.c:649-656` - 当前 Pitch 控制（需替换）
- `imu_task.h:7` - `INS_PITCH_ADDRESS_OFFSET = 1`
- `gimbal_task.c:610-618` - 目标初始化逻辑（需扩展）

**Acceptance Criteria**:

**Manual Execution Verification:**
- [ ] 编译通过，无警告
- [ ] CAN 抓包显示 Pitch 轴发送速度指令到 hfdcan2
- [ ] 手动倾斜底盘（改变 IMU pitch），云台 Pitch 电机反向转动补偿
- [ ] 停止倾斜，云台 Pitch 稳定
- [ ] 若符号错误（正反馈转圈），在误差计算处取反

**Commit**: YES
- Message: `feat(gimbal): add Pitch IMU stabilization with speed control`
- Files: `App/gimbal_task.c`

---

### - [ ] 4. Disable Vision Tracking

**What to do**:
1. 在 `gimbal_control_step_200hz()` 开头添加 `return;` 语句：
   ```c
   static void gimbal_control_step_200hz(void)
   {
       return;  // DISABLED: Vision tracking disabled, using IMU-only stabilization
       // ... rest of code unchanged
   }
   ```
2. 保留 `commUARTCallback()` 中的命令解析（T,T,T,T 和 N,N,N,N）
3. 保留 `vision_values[]` 数组更新（供调试用）

**Must NOT do**:
- 不删除 `gimbal_control_step_200hz()` 函数体
- 不修改 `commUARTCallback()` 的命令解析逻辑
- 不修改 UART 接收中断处理

**Parallelizable**: NO (需确认控制正确后再禁用)

**References**:
- `gimbal_task.c:553-599` - `gimbal_control_step_200hz()` 视觉处理逻辑
- `gimbal_task.c:808-884` - `commUARTCallback()` 命令解析

**Acceptance Criteria**:

**Manual Execution Verification:**
- [ ] 编译通过，无警告
- [ ] 通过 UART 发送视觉坐标数据（如 `50,30,0,0\n`）
- [ ] 云台不响应视觉数据（朝向不变）
- [ ] 发送 `T,T,T,T\n` 命令，蜂鸣器响（确认命令解析正常）
- [ ] IMU 增稳仍然有效

**Commit**: YES
- Message: `feat(gimbal): disable vision tracking, IMU-only stabilization`
- Files: `App/gimbal_task.c`

---

### - [ ] 5. Code Cleanup

**What to do**:
1. 删除未使用的函数：
   - `gimbal_send_pv_cmd()` (Lines 443-471)
   - `gimbal_control_step_100hz()` (Lines 753-758)
   - `gimbal_send_position_relative()` (Lines 426-441)
   - `distance_to_coef()` (Lines 740-750)
   - `enable_stabilizer()` (仅声明，Line 154)
   - `disable_stabilizer()` (Lines 798-806)

2. 删除未使用的 PID 结构体：
   - `pid_yaw` (Lines 184-186)
   - `pid_pitch` (Lines 188-190)
   - `pid_yaw_tracking` (Lines 175-177)
   - `pid_pitch_tracking` (Lines 179-181)
   - `pid_yaw_imu` (Lines 168-170) - 如果不使用

3. 删除未实现的前向声明：
   - `gimbal_tracking_handle` (Line 159)
   - `gimbal_searching_handle` (Line 160)

4. 删除未使用的变量：
   - `yaw_target_deg` (Line 198)
   - `pitch_target_deg` (Line 199)
   - `yaw_speed_vision_rpm` (Line 200)
   - `pitch_speed_vision_rpm` (Line 201)
   - `yaw_speed_imu_rpm` (Line 202)
   - `stabilizer_enabled` (Line 203)
   - `last_yaw_err`, `last_pitch_err` (Lines 227-228)
   - `last_yaw_err_pixels`, `last_pitch_err_pixels` (Lines 230-231)

5. 删除未使用的状态机代码：
   - `gimbal_state_machine_update()` (Lines 660-729)
   - `gimbal_tracking_handle()` 声明
   - `gimbal_searching_handle()` 声明
   - 相关变量：`gimbal_state`, `vision_lost_cnt`, `vision_found_cnt`, `search_*` 等

6. 删除未使用的宏定义：
   - `SEARCH_*` 系列宏 (Lines 28-33)
   - `TRACKING_*` 宏
   - `PIXEL_TO_DEG_BASE_*` 宏

**Must NOT do**:
- 不删除 `gimbal_send_position_trapezoid()`（保留作为回退方案）
- 不删除 `chassis_turning_mode` 相关代码
- 不删除 `g_rx_debug` 调试结构

**Parallelizable**: NO (最后执行)

**References**:
- Metis 分析中的 "Unused Code Identified" 表格
- 使用 `lsp_find_references` 验证每个函数确实无引用

**Acceptance Criteria**:

**Manual Execution Verification:**
- [ ] 编译通过，无警告 (`-Wall -Wextra`)
- [ ] 代码行数显著减少（预计减少 200+ 行）
- [ ] grep 搜索已删除函数名 → 无结果
- [ ] 功能回归测试：Yaw/Pitch IMU 增稳仍正常工作

**Commit**: YES
- Message: `refactor(gimbal): remove unused code and dead declarations`
- Files: `App/gimbal_task.c`

---

## Commit Strategy

| After Task | Message | Files | Verification |
|------------|---------|-------|--------------|
| 1 | `feat(gimbal): switch to speed mode initialization` | gimbal_task.c | CAN 抓包 |
| 2 | `feat(gimbal): convert Yaw to speed control` | gimbal_task.c | 手动测试 |
| 3 | `feat(gimbal): add Pitch IMU stabilization` | gimbal_task.c | 手动测试 |
| 4 | `feat(gimbal): disable vision tracking` | gimbal_task.c | 视觉无响应 |
| 5 | `refactor(gimbal): remove unused code` | gimbal_task.c | 编译 + 回归 |

---

## Success Criteria

### Verification Commands
```bash
# 编译验证
arm-none-eabi-gcc ... -Wall -Wextra  # Expected: 0 warnings

# 搜索已删除函数
grep -r "gimbal_send_pv_cmd" .       # Expected: no results
grep -r "gimbal_tracking_handle" .   # Expected: no results
```

### Final Checklist
- [ ] 速度模式初始化成功 (CAN 帧 0x0060=0x01)
- [ ] Yaw IMU 增稳：旋转底盘 → 云台补偿
- [ ] Pitch IMU 增稳：倾斜底盘 → 云台补偿
- [ ] 视觉数据不影响云台
- [ ] 代码无警告
- [ ] 未使用代码已删除
- [ ] `chassis_turning_mode` 代码保留且未修改

---

## Rollback Strategy

每个 Task 完成后提交 Git commit，如出现问题可回滚：

```bash
# 回滚到 Task N-1 状态
git revert HEAD

# 或完全回滚到开始前
git checkout <initial-commit-hash> -- App/gimbal_task.c
```

**保留 `gimbal_send_position_trapezoid()` 函数**，如需紧急回退到位置模式：
1. 将 `speed_mode_cmd` 改回 `position_trap_mode_cmd` (0x02)
2. 将控制循环中的 `gimbal_send_speed()` 改回 `gimbal_send_position_trapezoid()`
