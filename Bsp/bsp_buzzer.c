#include "bsp_buzzer.h"
static uint8_t buzzerSta = 0;

void BSP_Buzzer_Init(void)
{
    HAL_TIM_PWM_Start(&htim12, TIM_CHANNEL_2);
}

void BSP_Buzzer_On(void)
{
    TIM12->CCR2 = htim12.Init.Period / 2; /* 提高占空比方便听到声音 */
    buzzerSta = 1;
}

void BSP_Buzzer_Off(void)
{
    TIM12->CCR2 = 0;
    buzzerSta = 0;
}

void BSP_Buzzer_Toggle(void)
{
    buzzerSta == 1 ? BSP_Buzzer_Off():BSP_Buzzer_On();
}
