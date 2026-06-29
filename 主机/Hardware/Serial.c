#include "stm32f10x.h"
#include "Delay.h"
#include "Timer.h"
#include "Serial.h"
#include <stdio.h>
#include <stdarg.h>
#include <string.h>

/* 转义缓冲区: 原始帧最大可膨胀2倍 */
#define FRAME_ESC_BUF_SIZE    (FRAME_BUF_SIZE * 2)

/* ---- 全局变量 (保留兼容) ---- */
uint8_t Serial_TxPacket[4];
uint8_t Serial_RxPacket[4];
uint8_t Serial_RxFlag;

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

/* ============================================================
 *  CRC8 计算 (不变)
 *  多项式: x^8+x^5+x^4+1 (0x31), 初始值 0xFF, 左输入
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
 *  构建帧 (不变)
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
 *  USART1 中断服务函数 (优先级 0 — 最高)
 *  处理 RXNE (接收) 和 TXE (发送, 发送时开启)
 * ============================================================ */
void USART1_IRQHandler(void)
{
    /* ── RXNE: 接收数据 ── */
    if (USART_GetITStatus(USART1, USART_IT_RXNE))
    {
        uint8_t data = USART_ReceiveData(USART1);

        /* 如果已有待处理帧，不再写入（防止覆盖，主机一次只等一帧回复） */
        if (g_rxReady) return;

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
            /* 全部字节已送入发送移位寄存器 */
            USART_ITConfig(USART1, USART_IT_TXE, DISABLE);
            g_txBusy = 0;
            g_txDone = 1;
        }
    }
}

/* ============================================================
 *  中断驱动发送 (非阻塞启动, 然后等待完成)
 * ============================================================ */
void Serial_SendFrame_IT(uint8_t *frameBuf, uint8_t frameLen)
{
    uint8_t escLen;

    /* 等待上一次发送完成 */
    while (g_txBusy);

    /* 转义后放入发送缓冲区 */
    if (!Frame_Escape(frameBuf, frameLen, (uint8_t*)g_txBuf, &escLen))
        return;
    g_txLen = escLen;
    g_txIdx = 0;
    g_txDone = 0;
    g_txBusy = 1;

    /* 启动 TXE 中断 */
    USART_ITConfig(USART1, USART_IT_TXE, ENABLE);

    /* 阻塞等待发送完成 */
    while (!g_txDone);
}

/* ============================================================
 *  中断驱动接收 (等待 g_rxReady, 超时返回)
 * ============================================================ */
uint8_t Serial_ReceiveFrame_IT(uint8_t *buf, uint8_t *len, uint32_t timeoutMs)
{
    uint32_t start = GetTick();
    uint8_t frameLen;
    uint8_t i;
    uint8_t escFrame[FRAME_ESC_BUF_SIZE];

    /* 检查是否已有完整帧到达 (防止清空导致数据丢失) */
    if (g_rxReady)
    {
        __disable_irq();
        g_rxReady = 0;
        g_rxState = 0;
        frameLen = g_rxLen;
        for (i = 0; i < frameLen && i < FRAME_ESC_BUF_SIZE; i++)
            escFrame[i] = g_rxBuf[i];
        g_rxLen = 0;
        __enable_irq();
        if (Frame_Unescape(escFrame, frameLen, buf, &frameLen))
        {
            *len = frameLen;
            return 1;
        }
        return 0;
    }

    /* 无数据等待: 清除残留, 准备接收 */
    __disable_irq();
    g_rxLen = 0;
    g_rxReady = 0;
    g_rxState = 0;
    __enable_irq();

    while (1)
    {
        if (GetTick() - start > timeoutMs)
        {
            __disable_irq();
            g_rxLen = 0;
            g_rxReady = 0;
            g_rxState = 0;
            __enable_irq();
            return 0;
        }

        if (g_rxReady)
        {
            __disable_irq();
            g_rxReady = 0;
            g_rxState = 0;
            frameLen = g_rxLen;
            for (i = 0; i < frameLen && i < FRAME_ESC_BUF_SIZE; i++)
                escFrame[i] = g_rxBuf[i];
            g_rxLen = 0;
            __enable_irq();
            if (Frame_Unescape(escFrame, frameLen, buf, &frameLen))
            {
                *len = frameLen;
                return 1;
            }
            return 0;
        }
    }
}

/* ============================================================
 *  高层命令 (使用 IT 收发)
 * ============================================================ */

void Serial_SendAddrAlloc(void)
{
    uint8_t frame[FRAME_BUF_SIZE];
    uint8_t len = Serial_BuildFrame(frame, CMD_ADDR_ALLOC, 0x00, NULL, 0);
    if (len > 0) Serial_SendFrame_IT(frame, len);
}

uint8_t Serial_ReceiveAddrAllocResp(uint8_t *addrList, uint8_t *lastAddr)
{
    uint8_t frame[FRAME_BUF_SIZE];
    uint8_t frameLen;
    uint8_t i, cnt = 0;

    if (!Serial_ReceiveFrame_IT(frame, &frameLen, 3000))
        return 0;

    if (!CRC8_VerifyFrame(frame, frameLen))
        return 0xFF;

    if (frameLen >= 8 && frame[1] == CMD_ADDR_ALLOC)
    {
        *lastAddr = frame[3];
        cnt = (*lastAddr > 0) ? *lastAddr : 0;
        for (i = 0; i < cnt && i < 10; i++)
            addrList[i] = i + 1;
        return cnt;
    }
    return 0;
}

void Serial_SendHeartbeat(uint8_t modbusFuncCode, uint8_t *addrList, uint8_t addrCount)
{
    uint8_t frame[FRAME_BUF_SIZE];
    uint8_t content[FRAME_BUF_SIZE];
    uint8_t i, contentLen = 0, len;

    content[contentLen++] = modbusFuncCode;
    for (i = 0; i < addrCount; i++)
    {
        if (contentLen >= FRAME_BUF_SIZE - 4) break;
        content[contentLen++] = addrList[i];
    }

    len = Serial_BuildFrame(frame, CMD_HEARTBEAT, 0xFF, content, contentLen);
    if (len > 0) Serial_SendFrame_IT(frame, len);
}

/* ============================================================
 *  读取从机寄存器 (CMD_READ_REGS)
 *  content: [funcCode, regAddrH, regAddrL, qtyH, qtyL]
 *  返回后调用 Serial_ReceiveFrame_IT 获取应答
 * ============================================================ */
void Serial_SendReadRegs(uint8_t slaveAddr, uint8_t funcCode,
                         uint16_t regAddr, uint16_t quantity)
{
    uint8_t frame[FRAME_BUF_SIZE];
    uint8_t content[8];
    uint8_t len;

    content[0] = funcCode;
    content[1] = (uint8_t)(regAddr >> 8);
    content[2] = (uint8_t)(regAddr);
    content[3] = (uint8_t)(quantity >> 8);
    content[4] = (uint8_t)(quantity);

    len = Serial_BuildFrame(frame, CMD_READ_REGS, slaveAddr, content, 5);
    if (len > 0) Serial_SendFrame_IT(frame, len);
}

/* ============================================================
 *  写入从机寄存器 (CMD_WRITE_REGS)
 *  content: [funcCode, regAddrH, regAddrL, valueH, valueL]
 *  返回后调用 Serial_ReceiveFrame_IT 获取回显
 * ============================================================ */
void Serial_SendWriteRegs(uint8_t slaveAddr, uint8_t funcCode,
                          uint16_t regAddr, uint16_t value)
{
    uint8_t frame[FRAME_BUF_SIZE];
    uint8_t content[8];
    uint8_t len;

    content[0] = funcCode;
    content[1] = (uint8_t)(regAddr >> 8);
    content[2] = (uint8_t)(regAddr);
    content[3] = (uint8_t)(value >> 8);
    content[4] = (uint8_t)(value);

    len = Serial_BuildFrame(frame, CMD_WRITE_REGS, slaveAddr, content, 5);
    if (len > 0) Serial_SendFrame_IT(frame, len);
}

/* ============================================================
 *  广播写入所有从机 (CMD_BCAST_WRITE, 无应答)
 *  content: [funcCode, regAddrH, regAddrL, valueH, valueL]
 *  从机收到后不回复, 避免总线冲突
 * ============================================================ */
void Serial_SendBcastWrite(uint8_t funcCode, uint16_t regAddr, uint16_t value)
{
    uint8_t frame[FRAME_BUF_SIZE];
    uint8_t content[8];
    uint8_t len;

    content[0] = funcCode;
    content[1] = (uint8_t)(regAddr >> 8);
    content[2] = (uint8_t)(regAddr);
    content[3] = (uint8_t)(value >> 8);
    content[4] = (uint8_t)(value);

    len = Serial_BuildFrame(frame, CMD_BCAST_WRITE, 0xFF, content, 5);
    if (len > 0) Serial_SendFrame_IT(frame, len);
}

/* ============================================================
 *  串口初始化
 *  USART1: PA9-TX, PA10-RX, 9600-8N1
 *  NVIC 优先级: Preemption=0 (最高)
 *  使能 RXNE 中断 (始终接收)
 *  TXE 中断在发送时动态开启/关闭
 * ============================================================ */
void Serial_Init(void)
{
    GPIO_InitTypeDef GPIO_InitStructure;
    USART_InitTypeDef USART_InitStructure;
    NVIC_InitTypeDef NVIC_InitStructure;

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

    /* TXE 中断不在此开启, 由 Serial_SendFrame_IT 开启 */

    USART_Cmd(USART1, ENABLE);
}

/* ──────────── 兼容的阻塞发送函数 (调试/保留) ──────────── */
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
