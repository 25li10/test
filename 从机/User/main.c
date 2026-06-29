#include "stm32f10x.h"
#include "Serial.h"
#include "LED.h"
#include "Delay.h"
#include "Timer.h"

/* ============================================================
 *  从机主函数
 *
 *  功能: 循环等待接收主机帧, 处理后转发
 *  0xA5 — 地址分配: 地址+1保存, 回复应答帧
 *  0x5A — 心跳帧:  若地址在列表中则记录闪烁次数, 原样转发
 * ============================================================ */
int main(void)
{
    NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2);
    Timer_Init();
    Serial_Init();
    LED_Init();

    /* 上电闪烁一次表示就绪 */
    LED_ON();
    Delay_ms(100);
    LED_OFF();

    /* 主循环: 中断驱动收发 + 安全闪烁 */
    while (1)
    {
        Serial_SlaveProcess();
    }
}
