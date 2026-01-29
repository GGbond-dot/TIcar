#include "gimbal_task.h"
#include "FreeRTOS.h"
#include "task.h"
#include "can_bsp.h"
#include "main.h"
#include "bsp_buzzer.h"
#include "imu_task.h"
#include "usart.h"
#include <math.h>
#include <stdlib.h>
#include <string.h>
#include <stdio.h>
#include <stdbool.h>

/* ===================== 外部声明：陀螺仪数据 ===================== */
extern float gyro[3];  // gyro[0]=X轴, gyro[1]=Y轴, gyro[2]=Z轴(Yaw) 单位: rad/s

/* ===================== 底盘转向通知配置 ===================== */
/* 底盘急转弯时通过串口发送命令，云台开启IMU增稳 */
// 转向命令将在 commUARTCallback 函数中检测

/* ===================== PITCH轴角度限位 ===================== */
/* 以上电位置为0度，往下为正，往上为负 */
#define PITCH_LIMIT_DOWN            80.0f    // 最大往下80度
#define PITCH_LIMIT_UP              (-80.0f) // 最大往上80度（负值）
/* PITCH 方向修正：-1.0f 代表反向 */
#define PITCH_DIR                   (1.0f)

/* ===================== YAW 轴角度限位 (新增) ===================== */
/* 以底盘正前方为0度，左正右负（或者反过来，看你电机定义） */
/* 建议先给小一点，比如左右各 60 度，测试没问题再放开到 90 或 120 */
#define YAW_LIMIT_MAX               60.0f    // 左边最大转到 60 度
#define YAW_LIMIT_MIN               (-60.0f) // 右边最大转到 -60 度

/* ===================== 视觉中心偏移 ===================== */
/* y轴0点下降45像素：即当视觉输出0时，云台应认为偏上，需下压 */
/* 正值表示目标需在图像上方才能平衡 */
#define VISION_PITCH_OFFSET_PIXELS  0
/* x轴0点左移25像素：即当视觉输出25时，云台才认为在中心 */
#define VISION_YAW_OFFSET_PIXELS    15

/* ===================== [新增] 视觉闭环参数 ===================== */
/* 视场角 80度 / 640像素 = 0.125度/像素 */
#define VISION_PIXEL_TO_ANGLE   0.125f 
/* 视觉死区：像素误差小于此值不进行累加，防止震荡 */
#define VISION_DEADZONE         2     
/* 视觉增益：用于调节响应速度，0.1~1.0 之间 */
#define VISION_GAIN             0.08f


typedef struct
{
	FDCAN_HandleTypeDef *hcan;
	uint16_t tx_id;
	uint16_t rx_id;
	float kp;
	float max_speed_deg_s;
} gimbal_axis_config_t;

typedef struct
{
	gimbal_axis_config_t cfg;
	float target_angle_deg;
	float current_angle_deg;
	uint8_t initialized;
	uint8_t feedback_valid;
	uint32_t last_feedback_tick;
} gimbal_axis_runtime_t;

static gimbal_axis_runtime_t g_axis_table[GIMBAL_AXIS_COUNT] =
{
	[ GIMBAL_AXIS_YAW ] =
	{
		.cfg =
		{
			.hcan = GIMBAL_YAW_CAN_HANDLE,
			.tx_id = GIMBAL_YAW_CAN_ID,
			.rx_id = GIMBAL_YAW_CAN_RX_ID,
			.kp = GIMBAL_SPEED_KP_DEFAULT,
			.max_speed_deg_s = GIMBAL_MAX_SPEED_DEG_S,
		},
		.target_angle_deg = 0.0f,
		.current_angle_deg = 0.0f,
	},
	[ GIMBAL_AXIS_PITCH ] =
	{
		.cfg =
		{
			.hcan = GIMBAL_PITCH_CAN_HANDLE,
			.tx_id = GIMBAL_PITCH_CAN_ID,
			.rx_id = GIMBAL_PITCH_CAN_RX_ID,
			.kp = GIMBAL_PITCH_SPEED_KP_DEFAULT,
			.max_speed_deg_s = GIMBAL_PITCH_MAX_SPEED_DEG_S,
		},
		.target_angle_deg = 0.0f,
		.current_angle_deg = 0.0f,
	},
};

/* ===================== 全程融合控制变量 ===================== */
static float g_world_target_yaw = 0.0f;     // 世界坐标系下的目标 Yaw
static float g_world_target_pitch = 0.0f;   // 世界坐标系下的目标 Pitch
static bool g_target_initialized = false;   // 初始化标志
static float g_pitch_base_imu = 0.0f;        // 记录IMU基准pitch
static float g_pitch_base_motor = 0.0f;      // 记录电机基准pitch

float dbg_target_pitch = 0.0f;
float dbg_pitch_error = 0.0f;
float dbg_pitch_speed_rpm = 0.0f;
float dbg_imu_pitch_deg = 0.0f;
float dbg_motor_pitch_deg = 0.0f;
float dbg_yaw_speed_rpm = 0.0f;
float dbg_yaw_error = 0.0f;
float dbg_yaw_ff_rpm = 0.0f;

typedef struct
{
	float kp;
	float ki;
	float kd;
	float target;
	float integral;
	float prev;
	float out_limit;
	float integral_limit;
} pid_ctrl_t;

typedef struct
{
	uint16_t id;
	uint8_t len;
	uint8_t data[8];
	volatile uint8_t pending;
} gimbal_rx_debug_t;

static void gimbal_axis_enable(gimbal_axis_runtime_t *axis);
static void gimbal_request_angle(gimbal_axis_runtime_t *axis);
static void gimbal_send_speed(gimbal_axis_runtime_t *axis, int32_t rpm_value);
static void gimbal_send_position_trapezoid(gimbal_axis_runtime_t *axis, float angle_deg);
static int32_t gimbal_le32_to_i32(const uint8_t *data);
static void gimbal_apply_home(void);
static float pid_calc(pid_ctrl_t *pid, float input);
static float yaw_deg_from_imu(void);
static void gimbal_control_step_200hz(void);
static void gimbal_step_imu_execute(void);
static void start_vision_uart(void);
static void commUARTCallback(char *buf);
static int32_t rpm_to_raw(float rpm);
static float wrap_deg(float deg);

#ifndef M_PI
#define M_PI 3.14159265358979323846f
#endif

static gimbal_rx_debug_t g_rx_debug;

static pid_ctrl_t pid_yaw_speed = {
    .kp = 2.5f, .ki = 0.0f, .kd = 0.0f, .target = 0.0f, .integral = 0.0f, .prev = 0.0f,
    .out_limit = GIMBAL_SAFE_MAX_SPEED_RPM, .integral_limit = NAN};

static pid_ctrl_t pid_pitch_speed = {
    .kp = 2.5f, .ki = 0.0f, .kd = 0.0f, .target = 0.0f, .integral = 0.0f, .prev = 0.0f,
    .out_limit = GIMBAL_SAFE_MAX_SPEED_RPM, .integral_limit = NAN};

static float g_yaw_kp = 2.8f;
static float g_yaw_ki = 0.05f;
static float g_yaw_kd = 0.10f;
static float g_yaw_integral_limit = 600.0f;
static float g_yaw_ff_gain = 7.0f;
static float g_yaw_ff_sign = -1.0f;
static float g_yaw_deg_s_to_rpm = (1.0f / 6.0f);
static float g_yaw_integral_decay = 0.90f;
static float g_yaw_integral_decay_threshold = 1.0f;

static float g_pitch_kp = 2.8f;
static float g_pitch_ki = 0.02f;
static float g_pitch_kd = 0.10f;
static float g_pitch_integral_limit = 300.0f;
static float g_pitch_ff_gain = 1.5f;
static float g_pitch_ff_sign = -1.0f;
static float g_pitch_deg_s_to_rpm = (1.0f / 6.0f);

static volatile int32_t vision_values[8] = {666, 666};
static uint8_t vision_rx_buf[64];
static uint8_t vision_frame_buf[128];      // 帧拼接缓冲区
static volatile uint16_t vision_frame_len = 0;  // 当前帧长度
static volatile uint16_t vision_rx_len;
static volatile bool vision_rx_pending;
static float world_target_yaw_deg = 0.0f;   // 世界坐标系下的目标YAW角度
static bool world_target_initialized = false; // 目标角度是否已初始化

/* ===================== 串口7调试接收 ===================== */
static uint8_t uart7_rx_buf[64];
static uint8_t uart7_frame_buf[128];
static uint16_t uart7_frame_len = 0;

static volatile bool vision_data_new = false;  // 新数据标志，收到新数据时置 true

/* ===================== 底盘转向控制变量 ===================== */
static bool chassis_turning_mode = false;   // 底盘是否处于转弯状态，转弯时强制使用IMU增稳

#ifndef GIMBAL_IMU_UART_DEBUG
#define GIMBAL_IMU_UART_DEBUG 0
#endif

static void start_uart7_rx(void)
{
	HAL_UARTEx_ReceiveToIdle_IT(&huart7, uart7_rx_buf, sizeof(uart7_rx_buf));
}

void GimbalTask_Entry(void const *argument)
{
	UNUSED(argument);
	can_bsp_init();
	BSP_Buzzer_Init();
	start_vision_uart();
	start_uart7_rx();  // 启动串口7接收
	osDelay(500); // 等电机上电稳定
	for (uint32_t i = 0; i < GIMBAL_AXIS_COUNT; ++i)
	{
		gimbal_axis_enable(&g_axis_table[i]);
	}
	gimbal_apply_home();
	osDelay(2000);

	TickType_t last_wake = xTaskGetTickCount();
	uint32_t tick = 0;
	uint32_t imu_print_div = 0;
	uint32_t loop_tick = 0; // 这里的 tick 计数器是关键
	while (1)
	{

		/* 1. 基础动作：发送查询请求 */
        /* 这个建议跟随最高频率，或者跟随 500Hz 也可以 */
        if (loop_tick % 2 == 0) // 500Hz 请求一次数据
        {
            for (uint32_t i = 0; i < GIMBAL_AXIS_COUNT; ++i)
            {
                gimbal_request_angle(&g_axis_table[i]);
            }
        }

        /* 2. 任务分频调度 */
        
        // --- 500Hz 任务 (IMU 增稳执行) ---
        // 1ms 循环一次，% 2 == 0 意味着每 2ms 执行一次 = 500Hz
        if (loop_tick % 2 == 0)
        {
            gimbal_step_imu_execute();
        }

        // --- 200Hz 任务 (视觉逻辑处理) ---
        // % 5 == 0 意味着每 5ms 执行一次 = 200Hz
        if (loop_tick % 5 == 0)
        {
            gimbal_control_step_200hz();  // 改名误导，实际现在是1000Hz
        }

        /* 3. 循环控制 (1000Hz 基准) */
        loop_tick++;

		/* 主控制循环：1000Hz (1ms) */
		
		

		// 串口回显（已禁用，防止阻塞）
		// if (vision_rx_pending)
		// {
		// 	vision_rx_pending = false;
		// 	HAL_UART_Transmit(&huart1, vision_rx_buf, vision_rx_len, 10);
		// 	HAL_UART_Transmit(&huart1, (uint8_t *)"\r\n", 2U, 10);
		// }
		vision_rx_pending = false;  // 只清除标志，不回显

#if GIMBAL_IMU_UART_DEBUG
		// 周期打印 IMU 角度，方便确认 IMU 数据是否更新
		if ((imu_print_div++ % 100U) == 0U) // 约每0.5s (5ms*100)
		{
			char buf[96];
			float yaw_deg = imuAngle[INS_YAW_ADDRESS_OFFSET] * (180.0f / (float)M_PI);
			float pitch_deg = imuAngle[INS_PITCH_ADDRESS_OFFSET] * (180.0f / (float)M_PI);
			float roll_deg = imuAngle[INS_ROLL_ADDRESS_OFFSET] * (180.0f / (float)M_PI);
			int len = snprintf(buf, sizeof(buf), "IMU yaw=%.2f pitch=%.2f roll=%.2f\r\n", yaw_deg, pitch_deg, roll_deg);
			if (len > 0)
			{
				HAL_UART_Transmit(&huart1, (uint8_t *)buf, (uint16_t)len, 10);  // 减小超时
			}
		}
#endif
		++tick;
		vTaskDelayUntil(&last_wake, pdMS_TO_TICKS(1)); // 200Hz 基准 (5ms)
	}
}

void Gimbal_SetTargetAngle(gimbal_axis_t axis, float angle_deg)
{
	if (axis >= GIMBAL_AXIS_COUNT)
	{
		return;
	}
	g_axis_table[axis].target_angle_deg = angle_deg;
	if (g_axis_table[axis].initialized)
	{
		gimbal_send_position_trapezoid(&g_axis_table[axis], angle_deg);
	}
}

void Gimbal_SendPositionTrapezoid(gimbal_axis_t axis, float angle_deg)
{
	if (axis >= GIMBAL_AXIS_COUNT)
	{
		return;
	}
	gimbal_send_position_trapezoid(&g_axis_table[axis], angle_deg);
}

float Gimbal_GetCurrentAngle(gimbal_axis_t axis)
{
	if (axis >= GIMBAL_AXIS_COUNT)
	{
		return 0.0f;
	}
	return g_axis_table[axis].current_angle_deg;
}

static void gimbal_axis_enable(gimbal_axis_runtime_t *axis)
{
	static const uint8_t enable_cmd[8] = {0x2B, 0x00, 0xA2, 0x00, 0x00, 0x01, 0x00, 0x00};
	static const uint8_t speed_mode_cmd[8] = {0x2B, 0x00, 0x60, 0x00, 0x00, 0x01, 0x00, 0x00};
	fdcanx_send_data(axis->cfg.hcan, axis->cfg.tx_id, (uint8_t *)enable_cmd, sizeof(enable_cmd));
	osDelay(2);
	fdcanx_send_data(axis->cfg.hcan, axis->cfg.tx_id, (uint8_t *)speed_mode_cmd, sizeof(speed_mode_cmd));
	osDelay(2);
	gimbal_send_speed(axis, 0);
	osDelay(2);
	axis->initialized = 1;
}

static void gimbal_request_angle(gimbal_axis_runtime_t *axis)
{
	// 读取实时位置：寄存器 0x08，命令字 0x43
	static const uint8_t read_template[8] = {0x43, 0x00, 0x08, 0x00, 0x00, 0x00, 0x00, 0x00};
	uint8_t frame[8];
	memcpy(frame, read_template, sizeof(frame));
	fdcanx_send_data(axis->cfg.hcan, axis->cfg.tx_id, frame, sizeof(frame));
}

static void gimbal_send_speed(gimbal_axis_runtime_t *axis, int32_t rpm_value)
{
	uint8_t frame[8] = {0x23, 0x00, 0x21, 0x00, 0x00, 0x00, 0x00, 0x00};
	/* JC 协议速度字节采用高字节在前（例：500rpm*100=0x0000C350 -> 00 00 C3 50） */
	frame[4] = (uint8_t)((rpm_value >> 24) & 0xFF);
	frame[5] = (uint8_t)((rpm_value >> 16) & 0xFF);
	frame[6] = (uint8_t)((rpm_value >> 8) & 0xFF);
	frame[7] = (uint8_t)(rpm_value & 0xFF);
	fdcanx_send_data(axis->cfg.hcan, axis->cfg.tx_id, frame, sizeof(frame));
}

static int32_t gimbal_le32_to_i32(const uint8_t *data)
{
	return (int32_t)((uint32_t)data[0] | ((uint32_t)data[1] << 8) |
		((uint32_t)data[2] << 16) | ((uint32_t)data[3] << 24));
}

static void gimbal_send_position_trapezoid(gimbal_axis_runtime_t *axis, float angle_deg)
{
	int32_t angle_x100 = (int32_t)lroundf(angle_deg * 100.0f);
	uint8_t frame[8];
	/* 绝对位置命令：寄存器 0x23 */
	frame[0] = 0x23; /* 命令字 */
	frame[1] = 0x00; /* 寄存器高字节 */
	frame[2] = 0x23; /* 寄存器低字节（绝对位置） */
	frame[3] = 0x00;
	frame[4] = (uint8_t)((angle_x100 >> 24) & 0xFF);
	frame[5] = (uint8_t)((angle_x100 >> 16) & 0xFF);
	frame[6] = (uint8_t)((angle_x100 >> 8) & 0xFF);
	frame[7] = (uint8_t)(angle_x100 & 0xFF);
	fdcanx_send_data(axis->cfg.hcan, axis->cfg.tx_id, (uint8_t *)frame, sizeof(frame));
}

static void gimbal_apply_home(void)
{
	g_axis_table[GIMBAL_AXIS_YAW].target_angle_deg = 0.0f;
	g_axis_table[GIMBAL_AXIS_PITCH].target_angle_deg = 0.0f;
}

void Gimbal_GoHome(void)
{
	gimbal_apply_home();
	for (uint32_t i = 0; i < GIMBAL_AXIS_COUNT; ++i)
	{
		if (g_axis_table[i].initialized)
		{
			gimbal_send_position_trapezoid(&g_axis_table[i], g_axis_table[i].target_angle_deg);
		}
	}
}

static float pid_calc(pid_ctrl_t *pid, float input)
{
	const float err = pid->target - input;
	pid->integral += err;
	if (!isnan(pid->integral_limit))
	{
		if (pid->integral > pid->integral_limit)
			pid->integral = pid->integral_limit;
		else if (pid->integral < -pid->integral_limit)
			pid->integral = -pid->integral_limit;
	}
	float out = pid->kp * err + pid->ki * pid->integral + pid->kd * (err - pid->prev);
	pid->prev = err;
	if (!isnan(pid->out_limit))
	{
		if (out > pid->out_limit)
			out = pid->out_limit;
		else if (out < -pid->out_limit)
			out = -pid->out_limit;
	}
	return out;
}

static float wrap_deg(float deg)
{
	while (deg >= 360.0f)
	{
		deg -= 360.0f;
	}
	while (deg < 0.0f)
	{
		deg += 360.0f;
	}
	return deg;
}

// 从电机编码器获取yaw角度（比IMU更准确）
static float yaw_deg_from_motor(void)
{
	return g_axis_table[GIMBAL_AXIS_YAW].current_angle_deg;
}

// 保留IMU版本备用
static float yaw_deg_from_imu(void)
{
	float yaw_rad = imuAngle[INS_YAW_ADDRESS_OFFSET];
	float yaw_deg = yaw_rad * (180.0f / (float)M_PI);
	if (yaw_deg < 0.0f)
	{
		yaw_deg += 360.0f;
	}
	return wrap_deg(yaw_deg);
}

static int32_t rpm_to_raw(float rpm)
{
	return (int32_t)lroundf(rpm * 100.0f); // JC 协议: rpm*100
}

static uint32_t debug_cnt = 0;

/* ===================== 控制主循环 (修改后) ===================== */
static void gimbal_control_step_200hz(void)
{
	return;
   // 1. 获取视觉原始数据
    int32_t raw_yaw = vision_values[0];
    int32_t raw_pitch = vision_values[1];

    // 2. 校验数据有效性
    bool vision_valid = (raw_yaw != 666 && raw_yaw != 404 && abs(raw_yaw) < 640) &&
                        (raw_pitch != 666 && raw_pitch != 404 && abs(raw_pitch) < 480);

    // 3. 只有当收到“新”数据时才进行累加，防止重复计算
    if (vision_valid && vision_data_new)
    {
        vision_data_new = false; // 清除标志位 (原子操作)

        int32_t yaw_err_pixel = raw_yaw; 
        int32_t pitch_err_pixel = raw_pitch;

        // --- YAW 轴目标更新 ---
        if (abs(yaw_err_pixel) > VISION_DEADZONE)
        {
            // 像素 -> 角度
            float yaw_delta_deg = (float)yaw_err_pixel * VISION_PIXEL_TO_ANGLE;
            
            // 累加到世界目标 (注意：这里只改目标，不发指令)
            // 如果之前测试发现方向反了，把这里的 += 改成 -=
            g_world_target_yaw += (yaw_delta_deg * VISION_GAIN); 
            
            // 归一化 (0~360)
            g_world_target_yaw = wrap_deg(g_world_target_yaw);
        }

        // --- PITCH 轴目标更新 ---
        if (abs(pitch_err_pixel) > VISION_DEADZONE)
        {
            float pitch_delta_deg = (float)pitch_err_pixel * VISION_PIXEL_TO_ANGLE;
            
            // 之前调试确定的符号 (通常Pitch这里是-=)
            g_world_target_pitch -= (pitch_delta_deg * VISION_GAIN);
            
            // Pitch 软限位保护 (限制目标值，防止撞击)
            if (g_world_target_pitch > PITCH_LIMIT_DOWN) g_world_target_pitch = PITCH_LIMIT_DOWN;
            if (g_world_target_pitch < PITCH_LIMIT_UP) g_world_target_pitch = PITCH_LIMIT_UP;
        }
    }

}

/* ===================== [任务B] IMU增稳执行 (500Hz) ===================== */
/* 职责：读取IMU -> 计算与目标的误差 -> 发送电机指令 */
static void gimbal_step_imu_execute(void)
{
    // 1. 获取当前传感器数据
    float imu_yaw_current = yaw_deg_from_imu();
    float motor_yaw_current = g_axis_table[GIMBAL_AXIS_YAW].current_angle_deg;
    float imu_pitch_current = imuAngle[INS_PITCH_ADDRESS_OFFSET] * (180.0f / (float)M_PI);
    float motor_pitch_current = g_axis_table[GIMBAL_AXIS_PITCH].current_angle_deg;
    dbg_imu_pitch_deg = imu_pitch_current;
    dbg_motor_pitch_deg = motor_pitch_current;
    
    // 2. 初始化检查 (防止刚上电IMU没稳就乱转)
    if (!g_target_initialized)
    {
        if (fabsf(imu_yaw_current) > 0.1f || HAL_GetTick() > 3000)
        {
            g_world_target_yaw = imu_yaw_current;
            g_world_target_pitch = imu_pitch_current;
            g_pitch_base_imu = imu_pitch_current;
            g_pitch_base_motor = motor_pitch_current;
            g_target_initialized = true;
        }
        return; // 未初始化时不执行控制
    }

    /* ---------------- YAW 轴控制 (IMU 闭环) ---------------- */
    
    // 计算误差：目标世界角度 - 当前世界角度
    float yaw_world_error = g_world_target_yaw - imu_yaw_current;
    
    // 关键：处理过零点 (例如目标是1度，当前是359度，误差应为+2度，而不是-358度)
    if (yaw_world_error > 180.0f) yaw_world_error -= 360.0f;
    if (yaw_world_error < -180.0f) yaw_world_error += 360.0f;
    dbg_yaw_error = yaw_world_error;

    if ((yaw_world_error > 0.0f && pid_yaw_speed.prev < 0.0f) ||
        (yaw_world_error < 0.0f && pid_yaw_speed.prev > 0.0f))
    {
        pid_yaw_speed.integral = 0.0f;
    }

    pid_yaw_speed.kp = g_yaw_kp;
    pid_yaw_speed.ki = g_yaw_ki;
    pid_yaw_speed.kd = g_yaw_kd;
    pid_yaw_speed.integral_limit = g_yaw_integral_limit;

    float yaw_speed_rpm = pid_calc(&pid_yaw_speed, -yaw_world_error);
    float yaw_gyro_deg_s = gyro[2] * (180.0f / (float)M_PI);
    float yaw_ff_rpm = g_yaw_ff_sign * yaw_gyro_deg_s * g_yaw_deg_s_to_rpm * g_yaw_ff_gain;
    yaw_speed_rpm += yaw_ff_rpm;
    dbg_yaw_ff_rpm = yaw_ff_rpm;

    if (fabsf(yaw_world_error) < g_yaw_integral_decay_threshold)
    {
        pid_yaw_speed.integral *= g_yaw_integral_decay;
    }

    if (!isnan(pid_yaw_speed.out_limit))
    {
        if (yaw_speed_rpm > pid_yaw_speed.out_limit)
            yaw_speed_rpm = pid_yaw_speed.out_limit;
        else if (yaw_speed_rpm < -pid_yaw_speed.out_limit)
            yaw_speed_rpm = -pid_yaw_speed.out_limit;
    }
    dbg_yaw_speed_rpm = yaw_speed_rpm;



    /* ---------------- PITCH 轴控制 (IMU 闭环) ---------------- */
    float target_motor_pitch = g_pitch_base_motor - (imu_pitch_current - g_pitch_base_imu);
    if (target_motor_pitch > PITCH_LIMIT_DOWN) target_motor_pitch = PITCH_LIMIT_DOWN;
    if (target_motor_pitch < PITCH_LIMIT_UP) target_motor_pitch = PITCH_LIMIT_UP;
    float pitch_world_error = (target_motor_pitch - motor_pitch_current) * PITCH_DIR;
    dbg_target_pitch = target_motor_pitch;
    dbg_pitch_error = pitch_world_error;
    pid_pitch_speed.kp = g_pitch_kp;
    pid_pitch_speed.ki = g_pitch_ki;
    pid_pitch_speed.kd = g_pitch_kd;
    pid_pitch_speed.integral_limit = g_pitch_integral_limit;

    float pitch_speed_rpm = pid_calc(&pid_pitch_speed, -pitch_world_error);
    float pitch_gyro_deg_s = gyro[1] * (180.0f / (float)M_PI);
    float pitch_ff_rpm = g_pitch_ff_sign * pitch_gyro_deg_s * g_pitch_deg_s_to_rpm * g_pitch_ff_gain;
    pitch_speed_rpm += pitch_ff_rpm;

    if (!isnan(pid_pitch_speed.out_limit))
    {
        if (pitch_speed_rpm > pid_pitch_speed.out_limit)
            pitch_speed_rpm = pid_pitch_speed.out_limit;
        else if (pitch_speed_rpm < -pid_pitch_speed.out_limit)
            pitch_speed_rpm = -pid_pitch_speed.out_limit;
    }
    dbg_pitch_speed_rpm = pitch_speed_rpm;

    if ((motor_pitch_current >= PITCH_LIMIT_DOWN && pitch_speed_rpm > 0.0f) ||
        (motor_pitch_current <= PITCH_LIMIT_UP && pitch_speed_rpm < 0.0f))
    {
        pitch_speed_rpm = 0.0f;
        pid_pitch_speed.integral = 0.0f;
    }

    /* ---------------- 发送指令 ---------------- */
    gimbal_send_speed(&g_axis_table[GIMBAL_AXIS_YAW], rpm_to_raw(yaw_speed_rpm));
    gimbal_send_speed(&g_axis_table[GIMBAL_AXIS_PITCH], rpm_to_raw(pitch_speed_rpm));
}

void CAN_UserRxCallback(FDCAN_HandleTypeDef *hfdcan, const FDCAN_RxHeaderTypeDef *header, const uint8_t *data)
{
	if ((header == NULL) || (data == NULL))
	{
		return;
	}
	for (uint32_t i = 0; i < GIMBAL_AXIS_COUNT; ++i)
	{
		gimbal_axis_runtime_t *axis = &g_axis_table[i];
		if ((axis->cfg.hcan != hfdcan) || (header->Identifier != axis->cfg.rx_id))
		{
			continue;
		}
		// 解析位置回复：命令字0x43，寄存器0x08，数据是大端序
		if ((data[0] == 0x43) && (data[1] == 0x00) && (data[2] == 0x08) && (data[3] == 0x00))
		{
			// 大端序：data[4]是最高字节，data[7]是最低字节
			int32_t raw = (int32_t)(((uint32_t)data[4] << 24) | ((uint32_t)data[5] << 16) |
			                        ((uint32_t)data[6] << 8) | ((uint32_t)data[7]));
			axis->current_angle_deg = (float)raw / 100.0f;
			axis->feedback_valid = 1;
			axis->last_feedback_tick = HAL_GetTick();
		}
		uint8_t len = (uint8_t)(header->DataLength >> 16);
		if (len > sizeof(g_rx_debug.data))
		{
			len = sizeof(g_rx_debug.data);
		}
		UBaseType_t irq_state = taskENTER_CRITICAL_FROM_ISR();
		g_rx_debug.id = header->Identifier;
		g_rx_debug.len = len;
		memcpy(g_rx_debug.data, data, len);
		g_rx_debug.pending = 1;
		taskEXIT_CRITICAL_FROM_ISR(irq_state);
	}
}


static void commUARTCallback(char *buf)
{
	/* ===================== 新增：频率限制 ===================== */
    static uint32_t last_parse_tick = 0;
    uint32_t current_tick = HAL_GetTick();

    // 限制处理频率为 200Hz (即每 5ms 处理一次)
    // 如果距离上次处理不足 4ms，直接忽略这次数据，节省 CPU 算力
    // 注意：底盘转向命令(T,T,T,T)优先级高，不应该被忽略，所以先判断命令
    bool is_command = (buf[0] == 'T' || buf[0] == 'N');
    
    if (!is_command && (current_tick - last_parse_tick < 4)) 
    {
        return; // 丢弃这帧过快的数据
    }
    last_parse_tick = current_tick;
    /* ======================================================== */

	/* 首先检测底盘转向命令 */
	if (strncmp(buf, "T,T,T,T", 7) == 0)
	{
		chassis_turning_mode = true;
		// 记录当前IMU角度作为增稳目标
		world_target_yaw_deg = yaw_deg_from_imu();
		world_target_initialized = true;
		
		/* 打开蜂鸣器指示收到命令 */
		BSP_Buzzer_On();
		
		char dbg[64];
		snprintf(dbg, sizeof(dbg), "[CHASSIS] Turn START, IMU lock at %.1f deg\r\n", world_target_yaw_deg);
		HAL_UART_Transmit(&huart7, (uint8_t*)dbg, strlen(dbg), 10);
		return;
	}
	else if (strncmp(buf, "N,N,N,N", 7) == 0)
	{
		chassis_turning_mode = false;
		
		/* 关闭蜂鸣器 */
		BSP_Buzzer_Off();
		
		char dbg[64];
		snprintf(dbg, sizeof(dbg), "[CHASSIS] Turn END, resume tracking\r\n");
		HAL_UART_Transmit(&huart7, (uint8_t*)dbg, strlen(dbg), 10);
		return;
	}
	
	/* 解析正常的视觉坐标数据 */
	char *subStr = buf;
	char *ptr = buf;
	uint8_t valuesIdx = 0;

	while (true)
	{
		if (((*ptr >= '0') && (*ptr <= '9')) || (*ptr == '+') || (*ptr == '-'))
		{
			ptr++;
		}
		else if (*ptr == ',')
		{
			vision_values[valuesIdx] = atoi(subStr);
			ptr++;
			subStr = ptr;
			valuesIdx += 1U;
			if (valuesIdx >= 7U)
			{
				break;
			}
		}
		else
		{
			vision_values[valuesIdx] = atoi(subStr);
			break;
		}
	}
	vision_data_new = true;  // 标记收到新数据
}

static void start_vision_uart(void)
{
	HAL_UARTEx_ReceiveToIdle_IT(&huart1, vision_rx_buf, sizeof(vision_rx_buf));
}

void HAL_UART_ErrorCallback(UART_HandleTypeDef *huart)
{
	if (huart == &huart1)
	{
		/* 发生错误（如溢出），清除错误标志并重启接收 */
		__HAL_UART_CLEAR_OREFLAG(huart);
		__HAL_UART_CLEAR_NEFLAG(huart);
		__HAL_UART_CLEAR_FEFLAG(huart);
		__HAL_UART_CLEAR_PEFLAG(huart);
		HAL_UARTEx_ReceiveToIdle_IT(&huart1, vision_rx_buf, sizeof(vision_rx_buf));
	}
	else if (huart == &huart7)
	{
		__HAL_UART_CLEAR_OREFLAG(huart);
		HAL_UARTEx_ReceiveToIdle_IT(&huart7, uart7_rx_buf, sizeof(uart7_rx_buf));
	}
}

void HAL_UARTEx_RxEventCallback(UART_HandleTypeDef *huart, uint16_t Size)
{
	/* 串口7接收 - 用于调试，发送坐标格式如: 50,30,0,0 */
	if (huart == &huart7)
	{
		for (uint16_t i = 0; i < Size && uart7_frame_len < sizeof(uart7_frame_buf) - 1; i++)
		{
			uint8_t ch = uart7_rx_buf[i];
			if (ch == '\n' || ch == '\r')
			{
				if (uart7_frame_len > 0)
				{
					uart7_frame_buf[uart7_frame_len] = '\0';
					/* 解析并覆盖视觉数据 */
					commUARTCallback((char *)uart7_frame_buf);
					uart7_frame_len = 0;
				}
			}
			else
			{
				uart7_frame_buf[uart7_frame_len++] = ch;
			}
		}
		HAL_UARTEx_ReceiveToIdle_IT(&huart7, uart7_rx_buf, sizeof(uart7_rx_buf));
		return;
	}
	
	if (huart == &huart1)
	{
		/* 前 5s 内假装收到 404,404（无目标信号） */
		if (HAL_GetTick() < 5000)
		{
			vision_values[0] = 404;
			vision_values[1] = 404;
			HAL_UARTEx_ReceiveToIdle_IT(&huart1, vision_rx_buf, sizeof(vision_rx_buf));
			return;
		}

		/* 将收到的数据追加到帧缓冲区 */
		for (uint16_t i = 0; i < Size; i++)
		{
			uint8_t ch = vision_rx_buf[i];
			
			/* 检测到换行符，说明一帧完整了 */
			if (ch == '\n' || ch == '\r')
			{
				if (vision_frame_len > 0)
				{
					vision_frame_buf[vision_frame_len] = '\0';
					
					/* 禁用逐帧打印，防止串口阻塞 */
					// HAL_UART_Transmit(&huart7, vision_frame_buf, vision_frame_len, 10);
					// HAL_UART_Transmit(&huart7, (uint8_t *)"\r\n", 2, 10);
					
					/* 解析数据 */
					commUARTCallback((char *)vision_frame_buf);
					vision_rx_len = vision_frame_len;
					vision_rx_pending = true;
					
					/* 清空帧缓冲区 */
					vision_frame_len = 0;
				}
			}
			else
			{
				/* 累积数据，防止溢出 */
				if (vision_frame_len < sizeof(vision_frame_buf) - 1)
				{
					vision_frame_buf[vision_frame_len++] = ch;
				}
				else
				{
					/* 缓冲区满且未换行，强制清空，丢弃错误帧 */
					vision_frame_len = 0;
				}
			}
		}
		
		HAL_UARTEx_ReceiveToIdle_IT(&huart1, vision_rx_buf, sizeof(vision_rx_buf));
	}
}
