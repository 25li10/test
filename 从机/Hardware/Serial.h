#ifndef __SERIAL_H
#define __SERIAL_H

#include "stm32f10x.h"
#include <stdio.h>

/* ============================================================
 * 帧格式定义
 * | 0x7E | 指令类型 | 0x7F | 从机地址 | 0x7F | 指令内容 | 0x7F | CRC8 | 0x7E |
 * CRC8 范围: 0x7E之后 → CRC字段之前的所有字节
 * 多项式: x^8+x^5+x^4+1 (0x31), 初始值 0xFF, 左输入
 * ============================================================ */

#define FRAME_DELIMITER      0x7E   /* 起始/结束界定符 */
#define FRAME_SEPARATOR      0x7F   /* 字段分隔符 */
#define FRAME_ESC            0x7D   /* 转义引导符 */
#define FRAME_BUF_SIZE       64     /* 帧缓冲区最大长度 */

/* ---- 指令类型 ---- */
#define CMD_ADDR_ALLOC       0xA5   /* 自动地址分配 */
#define CMD_HEARTBEAT        0x5A   /* 心跳保持帧 */
#define CMD_READ_REGS        0x5B   /* 读取从机寄存器 (单播) */
#define CMD_WRITE_REGS       0x5C   /* 写入从机寄存器 (单播) */
#define CMD_BCAST_WRITE      0x5D   /* 广播写入所有从机 (无应答) */

/* ---- 心跳帧下的 Modbus RTU 功能码 ---- */
#define MODBUS_READ_COILS            0x01
#define MODBUS_READ_DISCRETE_INPUTS  0x02
#define MODBUS_READ_HOLDING_REGS     0x03
#define MODBUS_READ_INPUT_REGS       0x04
#define MODBUS_WRITE_SINGLE_COIL     0x05
#define MODBUS_WRITE_SINGLE_REG      0x06
#define MODBUS_WRITE_MULTI_COILS     0x0F
#define MODBUS_WRITE_MULTI_REGS      0x10

/* ---- 外部变量 ---- */
extern uint8_t Serial_TxPacket[];
extern uint8_t Serial_RxPacket[];
extern uint8_t Serial_RxFlag;
extern uint8_t Serial_SlaveAddr;      /* 本从机地址 (由地址分配帧设置) */

/* ---- CRC8 校验 ---- */
uint8_t CRC8_Calculate(uint8_t *data, uint8_t length);
uint8_t CRC8_VerifyFrame(uint8_t *frame, uint8_t frameLen);

/* ---- 基础串口 ---- */
void Serial_Init(void);
void Serial_SendByte(uint8_t Byte);
void Serial_SendArray(uint8_t *Array, uint16_t Length);
void Serial_SendString(char *String);
void Serial_SendNumber(uint32_t Number, uint8_t Length);
void Serial_Printf(char *format, ...);

/* ---- 帧操作 ---- */
uint8_t Serial_BuildFrame(uint8_t *frameBuf, uint8_t cmdType,
                          uint8_t addr, uint8_t *content, uint8_t contentLen);

/* ---- 从机核心：接收 → 处理 → 转发 ---- */
uint8_t Serial_SlaveProcess(void);

/* ---- 接收标志 ---- */
uint8_t Serial_GetRxFlag(void);

#endif

