#include "stm32f10x.h"
#include "Delay.h"
#include "Timer.h"
#include "Serial.h"
#include "LED.h"               /* LED_Blink */
#include <stdarg.h>
#include <string.h>

/* 转义缓冲区: 原始帧最大可膨胀2倍 */
#define FRAME_ESC_BUF_SIZE    (FRAME_BUF_SIZE * 2)

/* ---------- 全局变量 ---------- */
uint8_t Serial_TxPacket[4];
uint8_t Serial_RxPacket[4];
uint8_t Serial_RxFlag;

uint8_t Serial_SlaveAddr = 0x00;      /* 本从机地址，由地址分配帧设置 */

/* ============================================================
 *  模拟 Modbus 寄存器 (供 5.3/5.5 读写测试使用)
 * ============================================================ */
#define REG_HOLD_CNT   16               /* 保持寄存器数量 */
#define REG_COIL_CNT   16               /* 线圈数量 */
static uint16_t regHold[REG_HOLD_CNT];  /* 保持寄存器 (0x03读, 0x06写, 0x10多写) */
static uint8_t  regCoil[REG_COIL_CNT];  /* 线圈 (0x01读, 0x05写, 0x0F多写) */

/* ──────────────────────────────────────────────────────────────
 *  中断驱动状态
 * ────────────────────────────────────────────────────────────── */

/* RX (IRQ 接收) */
static volatile uint8_t  g_rxBuf[FRAME_ESC_BUF_SIZE];
static volatile uint8_t  g_rxLen   = 0;
static volatile uint8_t  g_rxReady = 0;      /* 1 = 已收到完整帧 */
static volatile uint8_t  g_rxState = 0;      /* 0=等起始, 1=收帧体 */

/* TX (IRQ 发送) */
static volatile uint8_t  g_txBuf[FRAME_ESC_BUF_SIZE];
static volatile uint8_t  g_txLen   = 0;
static volatile uint8_t  g_txIdx   = 0;
static volatile uint8_t  g_txBusy  = 0;      /* 1 = 正在发送 */
static volatile uint8_t  g_txDone  = 0;      /* 1 = 全部字节已送入移位寄存器 */

/* 待处理闪烁 */
static volatile uint8_t  g_pendingBlink = 0;  /* 0 = 无待闪, >0 = 闪烁次数 */

/* ============================================================
 *  CRC8 计算 (同主机)
 * ============================================================ */
uint8_t CRC8_Calculate(uint8_t *data, uint8_t length)
{
    uint8_t crc = 0xFF;
    uint8_t i, j;
    for (i = 0; i < length; i++)
    {
        crc ^= data[i];
        for (j = 0; j < 8; j++)
        {
            if (crc & 0x80)
                crc = (crc << 1) ^ 0x31;
            else
                crc = (crc << 1);
        }
    }
    return crc;
}

uint8_t CRC8_VerifyFrame(uint8_t *frame, uint8_t frameLen)
{
    uint8_t calcCrc;
    if (frameLen < 8) return 0;
    calcCrc = CRC8_Calculate(frame + 1, frameLen - 3);
    return (calcCrc == frame[frameLen - 2]) ? 1 : 0;
}

/* ============================================================
 *  转义 / 解转义 (帧体介于首尾 0x7E 之间的数据)
 *  - 0x7E → 0x7D 0x5E
 *  - 0x7F → 0x7D 0x5F
 *  - 0x7D → 0x7D 0x5D
 * ============================================================ */
static uint8_t Frame_Escape(uint8_t *raw, uint8_t rawLen,
                            uint8_t *esc, uint8_t *escLen)
{
    uint8_t i, ei = 0;
    if (rawLen < 2 || raw[0] != FRAME_DELIMITER || raw[rawLen-1] != FRAME_DELIMITER)
        return 0;
    esc[ei++] = FRAME_DELIMITER;
    for (i = 1; i < rawLen - 1; i++)
    {
        uint8_t b = raw[i];
        if (b == FRAME_DELIMITER || b == FRAME_SEPARATOR || b == FRAME_ESC)
        {
            if (ei + 1 >= FRAME_ESC_BUF_SIZE) return 0;
            esc[ei++] = FRAME_ESC;
            esc[ei++] = (b == FRAME_DELIMITER) ? 0x5E :
                        (b == FRAME_SEPARATOR) ? 0x5F : 0x5D;
        }
        else
        {
            if (ei >= FRAME_ESC_BUF_SIZE) return 0;
            esc[ei++] = b;
        }
    }
    esc[ei++] = FRAME_DELIMITER;
    *escLen = ei;
    return 1;
}

static uint8_t Frame_Unescape(uint8_t *esc, uint8_t escLen,
                              uint8_t *raw, uint8_t *rawLen)
{
    uint8_t i, ri = 0;
    if (escLen < 2 || esc[0] != FRAME_DELIMITER || esc[escLen-1] != FRAME_DELIMITER)
        return 0;
    raw[ri++] = FRAME_DELIMITER;
    i = 1;
    while (i < escLen - 1)
    {
        if (esc[i] == FRAME_ESC)
        {
            i++;
            if (i >= escLen - 1) return 0;
            if (ri >= FRAME_BUF_SIZE) return 0;
            if      (esc[i] == 0x5E) raw[ri++] = FRAME_DELIMITER;
            else if (esc[i] == 0x5F) raw[ri++] = FRAME_SEPARATOR;
            else if (esc[i] == 0x5D) raw[ri++] = FRAME_ESC;
            else return 0;
        }
        else
        {
            if (ri >= FRAME_BUF_SIZE) return 0;
            raw[ri++] = esc[i];
        }
        i++;
    }
    raw[ri++] = FRAME_DELIMITER;
    *rawLen = ri;
    return 1;
}

/* ============================================================
 *  构建帧 (同主机)
 * ============================================================ */
uint8_t Serial_BuildFrame(uint8_t *frameBuf, uint8_t cmdType,
                          uint8_t addr, uint8_t *content, uint8_t contentLen)
{
    uint8_t idx = 0;
    uint8_t crc;
    frameBuf[idx++] = FRAME_DELIMITER;
    frameBuf[idx++] = cmdType;
    frameBuf[idx++] = FRAME_SEPARATOR;
    frameBuf[idx++] = addr;
    frameBuf[idx++] = FRAME_SEPARATOR;
    if (contentLen > 0 && content != NULL)
    {
        if (idx + contentLen + 3 > FRAME_BUF_SIZE) return 0;
        memcpy(frameBuf + idx, content, contentLen);
        idx += contentLen;
    }
    frameBuf[idx++] = FRAME_SEPARATOR;
    crc = CRC8_Calculate(frameBuf + 1, idx - 1);
    frameBuf[idx++] = crc;
    frameBuf[idx++] = FRAME_DELIMITER;
    return idx;
}

/* ============================================================
 *  中断驱动发送 (非阻塞启动)
 *  主流程等待 g_txDone 来确保物理传输完成
 * ============================================================ */
static void Serial_SendFrame_IT(uint8_t *frameBuf, uint8_t frameLen)
{
    uint8_t escLen;

    while (g_txBusy);  /* 等待前一次发送完成 */

    /* 转义后放入发送缓冲区 */
    if (!Frame_Escape(frameBuf, frameLen, (uint8_t*)g_txBuf, &escLen))
        return;
    g_txLen  = escLen;
    g_txIdx  = 0;
    g_txDone = 0;
    g_txBusy = 1;

    USART_ITConfig(USART1, USART_IT_TXE, ENABLE);
}

/* ============================================================
 *  USART1 中断服务函数 (优先级 0 — 最高)
 *  处理 RXNE (接收) 和 TXE (发送)
 * ============================================================ */
void USART1_IRQHandler(void)
{
    /* ── RXNE: 接收数据 ── */
    if (USART_GetITStatus(USART1, USART_IT_RXNE))
    {
        uint8_t data = USART_ReceiveData(USART1);

        if (g_rxReady) return;              /* 待处理帧尚未读取，丢弃新字节 */

        if (g_rxState == 0)
        {
            if (data == FRAME_DELIMITER)
            {
                g_rxBuf[g_rxLen++] = data;
                g_rxState = 1;
            }
        }
        else
        {
            g_rxBuf[g_rxLen++] = data;
            if (g_rxLen >= FRAME_ESC_BUF_SIZE)
            {
                g_rxLen   = 0;
                g_rxState = 0;
            }
            else if (data == FRAME_DELIMITER && g_rxLen >= 8)
            {
                g_rxReady = 1;
                g_rxState = 0;
                /* g_rxLen 保留不归零，供 main 读取帧长度 */
            }
        }
    }

    /* ── TXE: 发送数据 ── */
    if (USART_GetITStatus(USART1, USART_IT_TXE))
    {
        if (g_txIdx < g_txLen)
        {
            USART_SendData(USART1, g_txBuf[g_txIdx++]);
        }
        else
        {
            /* 所有字节已进入移位寄存器 */
            USART_ITConfig(USART1, USART_IT_TXE, DISABLE);
            g_txBusy = 0;
            g_txDone = 1;                   /* 通知主循环: 传输结束 */
        }
    }
}

/* ============================================================
 *  从机核心处理流程 (中断驱动版本)
 *  在主循环中周期性调用:
 *    - 检查是否有完整帧已接收 (g_rxReady)
 *    - 检查是否有发送已完成 (g_txDone), 执行LED闪烁
 * ============================================================ */
uint8_t Serial_SlaveProcess(void)
{
    uint8_t frame[FRAME_BUF_SIZE];
    uint8_t escFrame[FRAME_ESC_BUF_SIZE];
    uint8_t idx;
    uint8_t i;
    uint8_t cmdType, addr;
    uint8_t content[FRAME_BUF_SIZE];
    uint8_t contentLen;
    uint8_t match;
    uint8_t newFrame[FRAME_BUF_SIZE];
    uint8_t newLen;

    /* ───── 检查发送完成 → 安全闪烁 ───── */
    if (g_txDone)
    {
        g_txDone = 0;
        /* 等待最后一字节在物理层面上完全发送完毕 */
        Delay_ms(2);

        if (g_pendingBlink > 0)
        {
            LED_Blink(g_pendingBlink);
            g_pendingBlink = 0;
        }
    }

    /* ───── 检查是否有完整帧到达 ───── */
    if (!g_rxReady) return 0;

    /* 从中断缓冲区复制转义帧到本地数组 (关中断保护) */
    __disable_irq();
    g_rxReady = 0;
    g_rxState = 0;
    idx = g_rxLen;
    for (i = 0; i < idx && i < FRAME_ESC_BUF_SIZE; i++)
        escFrame[i] = g_rxBuf[i];
    g_rxLen = 0;
    __enable_irq();

    if (idx < 8) return 0;

    /* ───── 解转义得到原始帧 ───── */
    if (!Frame_Unescape(escFrame, idx, frame, &idx))
        return 0;

    if (idx < 8) return 0;                  /* 解转义后帧太短 */

    /* ───── 验证CRC ───── */
    if (!CRC8_VerifyFrame(frame, idx))
    {
        return 0;
    }

    /* ───── 解析帧字段 ───── */
    cmdType = frame[1];
    addr    = frame[3];
    contentLen = 0;

    for (i = 5; i < idx - 3; i++)
    {
        if (frame[i] == FRAME_SEPARATOR) break;
        content[contentLen++] = frame[i];
    }

    /* ───── 按指令类型处理 ───── */
    if (cmdType == CMD_ADDR_ALLOC)
    {
        /* ---- 自动地址分配 ---- */
        Serial_SlaveAddr = addr + 1;
        addr = Serial_SlaveAddr;

        newLen = Serial_BuildFrame(newFrame, CMD_ADDR_ALLOC, addr, NULL, 0);
        if (newLen > 0)
        {
            Serial_SendFrame_IT(newFrame, newLen);
            /* 不等待，不闪烁，主循环在 g_txDone 后会自动处理 */
        }
        return 1;
    }
    else if (cmdType == CMD_HEARTBEAT)
    {
        match = 0;

        for (i = 1; i < contentLen; i++)
        {
            if (content[i] == Serial_SlaveAddr)
            {
                match = 1;
                break;
            }
        }

        /* 转发原帧 (无论是否匹配) */
        Serial_SendFrame_IT(frame, idx);

        if (match)
        {
            uint8_t modbusCode = content[0];
            uint8_t blinkCount = 0;

            if      (modbusCode >= 0x01 && modbusCode <= 0x06)
                blinkCount = modbusCode;
            else if (modbusCode == 0x0F)
                blinkCount = 7;
            else if (modbusCode == 0x10)
                blinkCount = 8;

            /* 不立即闪烁！记录次数，等 g_txDone 后再闪 */
            g_pendingBlink = blinkCount;
        }

        return 1;
    }
    else if (cmdType == CMD_READ_REGS)
    {
        /* ---- 读取寄存器 (0x5B) ----
         *  content: [funcCode, regAddrH, regAddrL, qtyH, qtyL]
         *  地址匹配 → 处理并应答
         *  地址不匹配 → 原样转发 (环形拓扑中需让应答帧回传至主机)
         */
        if (addr == Serial_SlaveAddr)
        {
            uint8_t funcCode = content[0];
            uint16_t regAddr = ((uint16_t)content[1] << 8) | content[2];
            uint16_t qty     = ((uint16_t)content[3] << 8) | content[4];
            uint8_t respData[32];
            uint8_t respLen = 0;
            uint8_t j;

            if (funcCode == 0x01)
            {
                /* 读取线圈 */
                uint8_t byteCnt = (qty + 7) / 8;
                respData[respLen++] = funcCode;
                respData[respLen++] = byteCnt;
                for (j = 0; j < qty && (regAddr + j) < REG_COIL_CNT; j++)
                {
                    if (regCoil[regAddr + j])
                        respData[respLen] |= (0x01 << (j % 8));
                    if ((j % 8) == 7) respLen++;
                }
                if (j % 8 != 0) respLen++;
            }
            else if (funcCode == 0x03)
            {
                /* 读取保持寄存器 */
                uint8_t byteCnt = qty * 2;
                respData[respLen++] = funcCode;
                respData[respLen++] = byteCnt;
                for (j = 0; j < qty && (regAddr + j) < REG_HOLD_CNT; j++)
                {
                    respData[respLen++] = (uint8_t)(regHold[regAddr + j] >> 8);
                    respData[respLen++] = (uint8_t)(regHold[regAddr + j]);
                }
            }
            else if (funcCode == 0x02)
            {
                /* 读取离散输入 (返回0) */
                uint8_t byteCnt = (qty + 7) / 8;
                respData[respLen++] = funcCode;
                respData[respLen++] = byteCnt;
                for (j = 0; j < byteCnt; j++)
                    respData[respLen++] = 0x00;
            }
            else if (funcCode == 0x04)
            {
                /* 读取输入寄存器 (返回0) */
                uint8_t byteCnt = qty * 2;
                respData[respLen++] = funcCode;
                respData[respLen++] = byteCnt;
                for (j = 0; j < byteCnt; j++)
                    respData[respLen++] = 0x00;
            }

            if (respLen > 0)
            {
                newLen = Serial_BuildFrame(newFrame, CMD_READ_REGS,
                                           Serial_SlaveAddr, respData, respLen);
                if (newLen > 0) Serial_SendFrame_IT(newFrame, newLen);
            }
        }
        else
        {
            /* 地址不匹配 → 原样转发，让应答帧沿环回传至主机 */
            newLen = Serial_BuildFrame(newFrame, cmdType, addr, content, contentLen);
            if (newLen > 0) Serial_SendFrame_IT(newFrame, newLen);
        }
        return 1;
    }
    else if (cmdType == CMD_WRITE_REGS)
    {
        /* ---- 写入寄存器 (0x5C) ----
         *  content: [funcCode, regAddrH, regAddrL, valueH, valueL]
         *  地址匹配 → 处理并回显
         *  地址不匹配 → 原样转发
         */
        if (addr == Serial_SlaveAddr)
        {
            uint8_t funcCode = content[0];
            uint16_t regAddr = ((uint16_t)content[1] << 8) | content[2];
            uint16_t value   = ((uint16_t)content[3] << 8) | content[4];

            if (funcCode == 0x05)
            {
                /* 写单个线圈 */
                if (regAddr < REG_COIL_CNT)
                    regCoil[regAddr] = (value == 0xFF00) ? 1 : 0;
            }
            else if (funcCode == 0x06)
            {
                /* 写单个保持寄存器 */
                if (regAddr < REG_HOLD_CNT)
                    regHold[regAddr] = value;
            }

            /* 回显原内容 */
            newLen = Serial_BuildFrame(newFrame, CMD_WRITE_REGS,
                                       Serial_SlaveAddr, content, contentLen);
            if (newLen > 0) Serial_SendFrame_IT(newFrame, newLen);
        }
        else
        {
            /* 地址不匹配 → 原样转发 */
            newLen = Serial_BuildFrame(newFrame, cmdType, addr, content, contentLen);
            if (newLen > 0) Serial_SendFrame_IT(newFrame, newLen);
        }
        return 1;
    }
    else if (cmdType == CMD_BCAST_WRITE)
    {
        /* ---- 广播写入 (0x5D, 无应答) ----
         *  content: [funcCode, regAddrH, regAddrL, valueH, valueL]
         */
        uint8_t funcCode = content[0];
        uint16_t regAddr = ((uint16_t)content[1] << 8) | content[2];
        uint16_t value   = ((uint16_t)content[3] << 8) | content[4];

        if (funcCode == 0x0F)
        {
            /* 写多个线圈 */
            uint8_t j;
            for (j = 0; j < 16 && (regAddr + j) < REG_COIL_CNT; j++)
            {
                regCoil[regAddr + j] = (value >> j) & 0x01;
            }
        }
        else if (funcCode == 0x10)
        {
            /* 写多个保持寄存器 */
            uint8_t j;
            for (j = 0; j < 16 && (regAddr + j) < REG_HOLD_CNT; j++)
            {
                regHold[regAddr + j] = value;  /* 同一个值写入连续地址 */
            }
        }
        /* 广播无应答, 但需转发至下一个从机 */
        newLen = Serial_BuildFrame(newFrame, cmdType, addr, content, contentLen);
        if (newLen > 0) Serial_SendFrame_IT(newFrame, newLen);
        return 1;
    }

    return 0;
}

/* ============================================================
 *  串口初始化
 *  USART1: PA9-TX, PA10-RX, 9600-8N1
 *  NVIC 优先级: Preemption=0 (最高), Sub=0
 *  使能 RXNE 中断 (始终接收)
 * ============================================================ */
void Serial_Init(void)
{
    GPIO_InitTypeDef GPIO_InitStructure;
    USART_InitTypeDef USART_InitStructure;
    NVIC_InitTypeDef NVIC_InitStructure;
    uint8_t i;

    RCC_APB2PeriphClockCmd(RCC_APB2Periph_USART1, ENABLE);
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA, ENABLE);

    /* PA9 = TX (复用推挽) */
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_9;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init(GPIOA, &GPIO_InitStructure);

    /* PA10 = RX (上拉输入) */
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPU;
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_10;
    GPIO_Init(GPIOA, &GPIO_InitStructure);

    USART_InitStructure.USART_BaudRate = 9600;
    USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
    USART_InitStructure.USART_Mode = USART_Mode_Tx | USART_Mode_Rx;
    USART_InitStructure.USART_Parity = USART_Parity_No;
    USART_InitStructure.USART_StopBits = USART_StopBits_1;
    USART_InitStructure.USART_WordLength = USART_WordLength_8b;
    USART_Init(USART1, &USART_InitStructure);

    /* ── NVIC: USART1 优先级 = 0 (最高) ── */
    NVIC_InitStructure.NVIC_IRQChannel                   = USART1_IRQn;
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;
    NVIC_InitStructure.NVIC_IRQChannelSubPriority        = 0;
    NVIC_InitStructure.NVIC_IRQChannelCmd                = ENABLE;
    NVIC_Init(&NVIC_InitStructure);

    /* 使能 RXNE 中断 (始终接收) */
    USART_ITConfig(USART1, USART_IT_RXNE, ENABLE);

    USART_Cmd(USART1, ENABLE);

    /* ---- 初始化模拟寄存器 (每个从机有独特的初始值用于验证) ---- */
    for (i = 0; i < REG_HOLD_CNT; i++)
        regHold[i] = (uint16_t)Serial_SlaveAddr * 256 + i;  /* 从机1: 0x0100~0x010F */
    for (i = 0; i < REG_COIL_CNT; i++)
        regCoil[i] = i % 2;  /* 交替 0/1 */
}

/* ---------- 基础发送函数 (阻塞, 调试用) ---------- */
void Serial_SendByte(uint8_t Byte)
{
    USART_SendData(USART1, Byte);
    while (USART_GetFlagStatus(USART1, USART_FLAG_TXE) == RESET);
}

void Serial_SendArray(uint8_t *Array, uint16_t Length)
{
    uint16_t i;
    for (i = 0; i < Length; i++)
        Serial_SendByte(Array[i]);
}

void Serial_SendString(char *String)
{
    uint8_t i;
    for (i = 0; String[i] != '\0'; i++)
        Serial_SendByte(String[i]);
}

static uint32_t Serial_Pow(uint32_t X, uint32_t Y)
{
    uint32_t Result = 1;
    while (Y--) Result *= X;
    return Result;
}

void Serial_SendNumber(uint32_t Number, uint8_t Length)
{
    uint8_t i;
    for (i = 0; i < Length; i++)
        Serial_SendByte(Number / Serial_Pow(10, Length - i - 1) % 10 + '0');
}

int fputc(int ch, FILE *f)
{
    Serial_SendByte(ch);
    return ch;
}

void Serial_Printf(char *format, ...)
{
    char String[100];
    va_list arg;
    va_start(arg, format);
    vsprintf(String, format, arg);
    va_end(arg);
    Serial_SendString(String);
}

uint8_t Serial_GetRxFlag(void)
{
    if (Serial_RxFlag == 1) { Serial_RxFlag = 0; return 1; }
    return 0;
}
