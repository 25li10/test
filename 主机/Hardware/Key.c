#include "stm32f10x.h"
#include "Delay.h"
#include "Key.h"

/**
  * @brief  按键初始化 (7键: PB11, PB1, PB0, PA6, PA4, PA2, PA0)
  *         接线: 按键一端接引脚，另一端接 GND，内部上拉
  */
void Key_Init(void)
{
    GPIO_InitTypeDef GPIO_InitStructure;

    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA, ENABLE);
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB, ENABLE);

    /* ---------- GPIOB: PB0, PB1, PB11 ---------- */
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPU;
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_0 | GPIO_Pin_1 | GPIO_Pin_11;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init(GPIOB, &GPIO_InitStructure);

    /* ---------- GPIOA: PA0, PA1, PA2, PA3, PA4, PA5, PA6 ---------- */
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_0 | GPIO_Pin_1 | GPIO_Pin_2
                                | GPIO_Pin_3 | GPIO_Pin_4 | GPIO_Pin_5
                                | GPIO_Pin_6;
    GPIO_Init(GPIOA, &GPIO_InitStructure);
}

/**
  * @brief  获取按键值 (消抖20ms，松开后返回)
  * @retval KEY_NONE~KEY_C15 (0~7)
  */
uint8_t Key_GetNum(void)
{
    uint8_t KeyNum = KEY_NONE;

    /* KEY1: PB11 - 自动地址分配 */
    if (GPIO_ReadInputDataBit(GPIOB, GPIO_Pin_11) == 0)
    {
        Delay_ms(20);
        while (GPIO_ReadInputDataBit(GPIOB, GPIO_Pin_11) == 0);
        Delay_ms(20);
        KeyNum = KEY1;
    }
    /* KEY2: PB1 - 发送心跳 */
    if (GPIO_ReadInputDataBit(GPIOB, GPIO_Pin_1) == 0)
    {
        Delay_ms(20);
        while (GPIO_ReadInputDataBit(GPIOB, GPIO_Pin_1) == 0);
        Delay_ms(20);
        KeyNum = KEY2;
    }
    /* KEY3: PB0 - 上一个Modbus功能码 */
    if (GPIO_ReadInputDataBit(GPIOB, GPIO_Pin_0) == 0)
    {
        Delay_ms(20);
        while (GPIO_ReadInputDataBit(GPIOB, GPIO_Pin_0) == 0);
        Delay_ms(20);
        KeyNum = KEY3;
    }
    /* KEY4: PA6 - 下一个Modbus功能码 */
    if (GPIO_ReadInputDataBit(GPIOA, GPIO_Pin_6) == 0)
    {
        Delay_ms(20);
        while (GPIO_ReadInputDataBit(GPIOA, GPIO_Pin_6) == 0);
        Delay_ms(20);
        KeyNum = KEY4;
    }
    /* KEY5: PA2 - 确认从机地址 */
    if (GPIO_ReadInputDataBit(GPIOA, GPIO_Pin_2) == 0)
    {
        Delay_ms(20);
        while (GPIO_ReadInputDataBit(GPIOA, GPIO_Pin_2) == 0);
        Delay_ms(20);
        KeyNum = KEY5;
    }
    /* KEY6: PA4 - 从机地址+ */
    if (GPIO_ReadInputDataBit(GPIOA, GPIO_Pin_4) == 0)
    {
        Delay_ms(20);
        while (GPIO_ReadInputDataBit(GPIOA, GPIO_Pin_4) == 0);
        Delay_ms(20);
        KeyNum = KEY6;
    }
    /* KEY7: PA0 - 从机地址- */
    if (GPIO_ReadInputDataBit(GPIOA, GPIO_Pin_0) == 0)
    {
        Delay_ms(20);
        while (GPIO_ReadInputDataBit(GPIOA, GPIO_Pin_0) == 0);
        Delay_ms(20);
        KeyNum = KEY7;
    }

    /* KEY8: PA1 - 读取选中从机寄存器 */
    if (GPIO_ReadInputDataBit(GPIOA, GPIO_Pin_1) == 0)
    {
        Delay_ms(20);
        while (GPIO_ReadInputDataBit(GPIOA, GPIO_Pin_1) == 0);
        Delay_ms(20);
        KeyNum = KEY8;
    }
    /* KEY9: PA3 - 写入选中从机寄存器 */
    if (GPIO_ReadInputDataBit(GPIOA, GPIO_Pin_3) == 0)
    {
        Delay_ms(20);
        while (GPIO_ReadInputDataBit(GPIOA, GPIO_Pin_3) == 0);
        Delay_ms(20);
        KeyNum = KEY9;
    }
    /* KEY10: PA5 - 广播写入所有从机 */
    if (GPIO_ReadInputDataBit(GPIOA, GPIO_Pin_5) == 0)
    {
        Delay_ms(20);
        while (GPIO_ReadInputDataBit(GPIOA, GPIO_Pin_5) == 0);
        Delay_ms(20);
        KeyNum = KEY10;
    }

    return KeyNum;
}
