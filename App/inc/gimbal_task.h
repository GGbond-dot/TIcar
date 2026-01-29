#ifndef __GIMBAL_TASK_H__
#define __GIMBAL_TASK_H__

#include "cmsis_os.h"
#include "fdcan.h"

#ifdef __cplusplus
extern "C" {
#endif

typedef enum
{
    GIMBAL_AXIS_YAW = 0,
    GIMBAL_AXIS_PITCH,
    GIMBAL_AXIS_COUNT
} gimbal_axis_t;

#ifndef GIMBAL_YAW_CAN_HANDLE
#define GIMBAL_YAW_CAN_HANDLE       (&hfdcan1)   /* 水平：CAN1 */
#endif

#ifndef GIMBAL_CAN_BASE_ID
#define GIMBAL_CAN_BASE_ID          (0x600U)
#endif

#ifndef GIMBAL_CAN_TX_ENABLE
#define GIMBAL_CAN_TX_ENABLE        1   /* 1: 允许任务发送CAN */
#endif

#ifndef GIMBAL_JC_DEMO_MODE
#define GIMBAL_JC_DEMO_MODE         0   /* 关闭示例周期读电压逻辑 */
#endif

#ifndef GIMBAL_DEMO_SPEED_RPM_X100
#define GIMBAL_DEMO_SPEED_RPM_X100  100   /* demo模式下发送的测试转速，100=1rpm */
#endif

#ifndef GIMBAL_STARTUP_SPEED_RPM_X100
#define GIMBAL_STARTUP_SPEED_RPM_X100 2000 /* 上电后一次性发送的速度指令，2000=20rpm */
#endif

#ifndef GIMBAL_YAW_CAN_ID
#define GIMBAL_YAW_CAN_ID           (0x601U)      /* 水平：CAN1, ID 0x602 */
#endif

#ifndef GIMBAL_YAW_CAN_RX_ID
#define GIMBAL_YAW_CAN_RX_ID        (0x580U + 0x1U) /* 对应 0x601 */
#endif

#ifndef GIMBAL_PITCH_CAN_HANDLE
#define GIMBAL_PITCH_CAN_HANDLE     (&hfdcan2)   /* 竖直：CAN2 */
#endif

#ifndef GIMBAL_PITCH_CAN_ID
#define GIMBAL_PITCH_CAN_ID         (0x601U)      /* 竖直：ID 0x602 */
#endif

#ifndef GIMBAL_PITCH_CAN_RX_ID
#define GIMBAL_PITCH_CAN_RX_ID      (0x580U + 0x1U)
#endif

#ifndef GIMBAL_SPEED_KP_DEFAULT
#define GIMBAL_SPEED_KP_DEFAULT     (4.0f)
#endif

#ifndef GIMBAL_PITCH_SPEED_KP_DEFAULT
#define GIMBAL_PITCH_SPEED_KP_DEFAULT   (GIMBAL_SPEED_KP_DEFAULT)
#endif

#ifndef GIMBAL_MAX_SPEED_DEG_S
#define GIMBAL_MAX_SPEED_DEG_S      (120.0f)
#endif

#ifndef GIMBAL_PITCH_MAX_SPEED_DEG_S
#define GIMBAL_PITCH_MAX_SPEED_DEG_S    (GIMBAL_MAX_SPEED_DEG_S)
#endif

#ifndef GIMBAL_SAFE_MAX_SPEED_DEG_S
#define GIMBAL_SAFE_MAX_SPEED_DEG_S     (180.0f) /* 安全最大角速度限制，单位 deg/s */
#endif

#ifndef GIMBAL_SAFE_MAX_SPEED_RPM
#define GIMBAL_SAFE_MAX_SPEED_RPM       (50.0f) /* 安全最大角速度限制，单位 rpm */
#endif

#ifndef GIMBAL_YAW_HOME_DEG
#define GIMBAL_YAW_HOME_DEG         (0.0f)
#endif

#ifndef GIMBAL_PITCH_HOME_DEG
#define GIMBAL_PITCH_HOME_DEG       (0.0f)
#endif

#ifndef GIMBAL_TEST_SPIN_ENABLE
#define GIMBAL_TEST_SPIN_ENABLE     0   /* 关闭 */
#endif

#ifndef GIMBAL_TEST_SPIN_RPM_X100
#define GIMBAL_TEST_SPIN_RPM_X100   160 /* 约1.6 rpm≈10 deg/s */
#endif

#ifndef GIMBAL_OPEN_LOOP_TEST_ENABLE
#define GIMBAL_OPEN_LOOP_TEST_ENABLE    0   /* 关闭开环测试 */
#endif

#ifndef GIMBAL_OPEN_LOOP_YAW_RPM_X100
#define GIMBAL_OPEN_LOOP_YAW_RPM_X100   300 /* 3 rpm≈18 deg/s */
#endif

#ifndef GIMBAL_OPEN_LOOP_PITCH_RPM_X100
#define GIMBAL_OPEN_LOOP_PITCH_RPM_X100 300 /* 3 rpm≈18 deg/s */
#endif

#ifndef GIMBAL_CONTROL_PERIOD_MS
#define GIMBAL_CONTROL_PERIOD_MS    (5U)
#endif

void GimbalTask_Entry(void const *argument);
void Gimbal_SetTargetAngle(gimbal_axis_t axis, float angle_deg);
void Gimbal_SendPositionTrapezoid(gimbal_axis_t axis, float angle_deg);
float Gimbal_GetCurrentAngle(gimbal_axis_t axis);
void Gimbal_GoHome(void);

#ifdef __cplusplus
}
#endif

#endif /* __GIMBAL_TASK_H__ */
