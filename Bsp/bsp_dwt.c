#include "bsp_dwt.h"

static DWT_Time_t SysTime;
static uint32_t CPU_FREQ_Hz, CPU_FREQ_Hz_ms, CPU_FREQ_Hz_us;
static uint32_t CYCCNT_RountCount;
static uint32_t CYCCNT_LAST;
static uint64_t CYCCNT64;

static void DWT_CNT_Update(void)
{
    static volatile uint8_t bit_locker = 0;
    if (!bit_locker)
    {
        bit_locker = 1;
        volatile uint32_t cnt_now = DWT->CYCCNT;
        if (cnt_now < CYCCNT_LAST)
            CYCCNT_RountCount++;

        CYCCNT_LAST = DWT->CYCCNT;
        bit_locker = 0;
    }
}

void DWT_Init(uint32_t CPU_Freq_mHz)
{
    CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;
    DWT->CYCCNT = (uint32_t)0u;
    DWT->CTRL |= DWT_CTRL_CYCCNTENA_Msk;

    CPU_FREQ_Hz = CPU_Freq_mHz * 1000000U;
    CPU_FREQ_Hz_ms = CPU_FREQ_Hz / 1000U;
    CPU_FREQ_Hz_us = CPU_FREQ_Hz / 1000000U;
    CYCCNT_RountCount = 0;

    DWT_CNT_Update();
}

float DWT_GetDeltaT(uint32_t *cnt_last)
{
    volatile uint32_t cnt_now = DWT->CYCCNT;
    float dt = ((uint32_t)(cnt_now - *cnt_last)) / ((float)(CPU_FREQ_Hz));
    *cnt_last = cnt_now;

    DWT_CNT_Update();

    return dt;
}

double DWT_GetDeltaT64(uint32_t *cnt_last)
{
    volatile uint32_t cnt_now = DWT->CYCCNT;
    double dt = ((uint32_t)(cnt_now - *cnt_last)) / ((double)(CPU_FREQ_Hz));
    *cnt_last = cnt_now;

    DWT_CNT_Update();

    return dt;
}

void DWT_SysTimeUpdate(void)
{
    volatile uint32_t cnt_now = DWT->CYCCNT;
    static uint64_t CNT_TEMP1, CNT_TEMP2, CNT_TEMP3;

    DWT_CNT_Update();

    CYCCNT64 = (uint64_t)CYCCNT_RountCount * (uint64_t)UINT32_MAX + (uint64_t)cnt_now;
    CNT_TEMP1 = CYCCNT64 / CPU_FREQ_Hz;
    CNT_TEMP2 = CYCCNT64 - CNT_TEMP1 * CPU_FREQ_Hz;
    SysTime.s = (uint32_t)CNT_TEMP1;
    SysTime.ms = (uint16_t)(CNT_TEMP2 / CPU_FREQ_Hz_ms);
    CNT_TEMP3 = CNT_TEMP2 - SysTime.ms * CPU_FREQ_Hz_ms;
    SysTime.us = (uint16_t)(CNT_TEMP3 / CPU_FREQ_Hz_us);
}

float DWT_GetTimeline_s(void)
{
    DWT_SysTimeUpdate();

    return SysTime.s + SysTime.ms * 0.001f + SysTime.us * 0.000001f;
}

float DWT_GetTimeline_ms(void)
{
    DWT_SysTimeUpdate();

    return SysTime.s * 1000.0f + SysTime.ms + SysTime.us * 0.001f;
}

uint64_t DWT_GetTimeline_us(void)
{
    DWT_SysTimeUpdate();

    return (uint64_t)SysTime.s * 1000000ULL + (uint64_t)SysTime.ms * 1000ULL + SysTime.us;
}

void DWT_Delay(float Delay)
{
    uint32_t tickstart = DWT->CYCCNT;
    float wait = Delay;

    while ((DWT->CYCCNT - tickstart) < wait * (float)CPU_FREQ_Hz)
        ;
}
