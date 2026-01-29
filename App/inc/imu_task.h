#ifndef __IMU_TASK_H__
#define __IMU_TASK_H__

#include "cmsis_os.h"

#define INS_YAW_ADDRESS_OFFSET    0
#define INS_PITCH_ADDRESS_OFFSET  1
#define INS_ROLL_ADDRESS_OFFSET   2

/* IMU 姿态、陀螺仪数据由 imu_task.c 中更新，云台控制直接读取 */
extern float imuAngle[3];
extern float gyro[3];

#endif
