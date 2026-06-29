#include "Timer.h"

/* TIM3 溢出次数 (每 65536ms 溢出一次) */
volatile uint16_t g_timerOverflows = 0;

/**
  * @brief  TIM3 更新中断: 每溢出一次 +65536
  */
void TIM3_IRQHandler(void)
{
    if (TIM_GetITStatus(TIM3, TIM_IT_Update))
    {
        g_timerOverflows++;
        TIM_ClearITPendingBit(TIM3, TIM_IT_Update);
    }
}

/**
  * @brief  初始化 TIM3 为 1ms 自由计数器
  *         APB1 = 72MHz, Prescaler = 72000-1 → 1kHz
  *         Period = 0xFFFF → 每 65536ms 溢出一次
  *         优先级设为 1 (USART1 为 0, 更高)
  */
void Timer_Init(void)
{
    TIM_TimeBaseInitTypeDef tim;
    NVIC_InitTypeDef nvic;

    /* 使能 TIM3 时钟 (APB1) */
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM3, ENABLE);

    TIM_TimeBaseStructInit(&tim);
    tim.TIM_Prescaler     = (uint16_t)72000 - 1;    /* 72MHz / 72000 = 1kHz */
    tim.TIM_Period        = 0xFFFF;       /* 16位最大 */
    tim.TIM_CounterMode   = TIM_CounterMode_Up;
    TIM_TimeBaseInit(TIM3, &tim);

    /* 使能更新中断 */
    TIM_ITConfig(TIM3, TIM_IT_Update, ENABLE);

    /* NVIC 优先级 = 1 (低于 USART1 的 0) */
    nvic.NVIC_IRQChannel                   = TIM3_IRQn;
    nvic.NVIC_IRQChannelPreemptionPriority = 1;
    nvic.NVIC_IRQChannelSubPriority        = 0;
    nvic.NVIC_IRQChannelCmd                = ENABLE;
    NVIC_Init(&nvic);

    /* 启动 TIM3 */
    TIM_Cmd(TIM3, ENABLE);
}

/**
  * @brief  获取毫秒级计数器 (32位, 约49天回绕)
  */
uint32_t GetTick(void)
{
    uint16_t ovf;
    uint16_t cnt;

    /* 原子读取: 关中断防止溢出和计数器不同步 */
    __disable_irq();
    ovf = g_timerOverflows;
    cnt = TIM_GetCounter(TIM3);
    __enable_irq();

    return ((uint32_t)ovf << 16) | cnt;
}
