#include "key_task.h"
#include "adc.h"
#include "cmsis_os.h"
#include "vofa.h"
#include "bsp_user_key.h"
#include "bsp_buzzer.h"
#include "ws2812.h"

float vbus = 0;
volatile uint16_t adc_val[2];

static void KeyTask_UpdateLed(float vbus_v);

void KeyTask_Entry(void const * argument)
{
    /* USER CODE BEGIN KeyTask_Entry */
    BSP_Buzzer_Init();
    HAL_ADCEx_Calibration_Start(&hadc1, ADC_CALIB_OFFSET, ADC_SINGLE_ENDED);
    HAL_ADC_Start_DMA(&hadc1, (uint32_t *)adc_val,2);
    WS2812_Ctrl(0, 10, 0);
    /* Infinite loop */
    for(;;)
    {
        if (BSP_UserKey_Detect() == BUTTON_PRESSED)
        {
            BSP_Buzzer_Toggle();
        }
        vbus = (adc_val[0]*3.3f/65535)*11.0f;
        KeyTask_UpdateLed(vbus);
        osDelay(10);
    }
    /* USER CODE END KeyTask_Entry */
}

/* 根据总线电压设置指示灯颜色，低电压偏红，高电压偏绿 */
static void KeyTask_UpdateLed(float vbus_v)
{
    const float v_min = 10.0f;   /* 3S 低电阈值 */
    const float v_max = 12.6f;   /* 3S 满电电压 */
    float ratio = (vbus_v - v_min) / (v_max - v_min);
    if (ratio < 0.0f)
    {
        ratio = 0.0f;
    }
    else if (ratio > 1.0f)
    {
        ratio = 1.0f;
    }
    uint8_t g = (uint8_t)(ratio * 50.0f);          /* 0~50 */
    uint8_t r = (uint8_t)((1.0f - ratio) * 50.0f); /* 0~50 */
    WS2812_Ctrl(r, g, 0);
}
