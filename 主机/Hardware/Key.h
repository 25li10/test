#ifndef __KEY_H
#define __KEY_H

#include "stm32f10x.h"

void Key_Init(void);
uint8_t Key_GetNum(void);

/* 按键编号定义 */
#define KEY_NONE    0
#define KEY1        1       /* PB11 - 自动地址分配 */
#define KEY2        2       /* PB1  - 发送心跳到目标从机 */
#define KEY3        3       /* PB0  - 上一个Modbus功能码 */
#define KEY4        4       /* PA6  - 下一个Modbus功能码 */
#define KEY5        5       /* PA2  - 确认从机地址 */
#define KEY6        6       /* PA4  - 从机地址+ */
#define KEY7        7       /* PA0  - 从机地址- */
#define KEY8        8       /* PA1  - 读取选中从机寄存器 */
#define KEY9        9       /* PA3  - 写入选中从机寄存器 */
#define KEY10       10      /* PA5  - 广播写入所有从机 */

#endif
