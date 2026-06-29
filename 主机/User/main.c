#include "stm32f10x.h"
#include "Delay.h"
#include "OLED.h"
#include "Serial.h"
#include "Key.h"
#include "LED.h"
#include "Timer.h"

/* ============================================================
 *  Modbus 功能码列表
 * ============================================================ */
static const uint8_t ModbusFuncList[] = {
    0x01, 0x02, 0x03, 0x04, 0x05, 0x06, 0x0F, 0x10
};
#define MODBUS_FUNC_COUNT   8   /* 功能码数量 */
#define MAX_SLAVES         10   /* 最大从机数量 */

/**
 * @brief  Modbus 功能码 → 短名称
 * @param  code  功能码
 * @return 名称字符串指针
 */
static char *ModbusFuncName(uint8_t code)
{
    switch (code)
    {
        case 0x01: return "Read Coils";
        case 0x02: return "Read DiscIn";
        case 0x03: return "Read HoldR";
        case 0x04: return "Read InpRg";
        case 0x05: return "Wr1 Coil";
        case 0x06: return "Wr1 Reg";
        case 0x0F: return "WrN Coils";
        case 0x10: return "WrN Regs";
        default:   return "Unknown";
    }
}

/* ============================================================
 *  CRC 校验结果跟踪 (全局状态, 供 ShowStatus 显示)
 * ============================================================ */
static uint8_t lastCrcStatus = 0;   /* 0=未知, 1=OK, 2=FAIL */
static uint8_t lastCrcCmd    = 0;   /* 最近校验的指令类型 */
static uint8_t lastCrcCalc   = 0;   /* 主机计算的 CRC 值 */
static uint8_t lastCrcRecv   = 0;   /* 从机返回的 CRC 值 */

/* ============================================================
 *  界面显示
 * ============================================================ */

/**
 * @brief  刷新 OLED 主界面
 * @param  addrList      从机地址列表
 * @param  slaveCount    从机数量
 * @param  cursorIdx     光标当前位置
 * @param  selectedAddrs 选中标记数组 (1=选中)
 * @param  modbusCode    当前 Modbus 功能码
 */
static void ShowStatus(uint8_t *addrList, uint8_t slaveCount,
                       uint8_t cursorIdx, uint8_t *selectedAddrs,
                       uint8_t modbusCode)
{
    uint8_t i;
    uint8_t selectedCount = 0;

    for (i = 0; i < slaveCount; i++)
        if (selectedAddrs[i]) selectedCount++;

    OLED_Clear();

    /* ---- Row1: 选中数量 + 光标 ----
     * 格式: Sel:N *ADDR(V)
     *       Sel:3 *01(V)     ← 选中3个, 光标在01(已选中)
     */
    if (slaveCount > 0)
    {
        OLED_ShowString(1, 1, "Sel:");
        OLED_ShowNum(1, 5, selectedCount, 1);
        OLED_ShowChar(1, 7, ' ');
        OLED_ShowChar(1, 8, '*');
        OLED_ShowHexNum(1, 9, addrList[cursorIdx], 2);
        if (selectedAddrs[cursorIdx])
            OLED_ShowChar(1, 12, 'V');
    }
    else
    {
        OLED_ShowString(1, 1, "No Slaves");
    }

    /* ---- Row2: Modbus 功能码 ---- */
    OLED_ShowHexNum(2, 1, modbusCode, 2);
    OLED_ShowChar(2, 4, ' ');
    OLED_ShowString(2, 4, ModbusFuncName(modbusCode));

    /* ---- Row3: CRC 校验结果 ----
     *  OK:   "CRC:OK(0xA5)"
     *  FAIL: "FAIL(0xA5) C:XX"
     *  未知: "CRC:----"
     */
    if (lastCrcStatus == 0)
    {
        OLED_ShowString(3, 1, "CRC:----");
    }
    else if (lastCrcStatus == 1)
    {
        OLED_ShowString(3, 1, "CRC:OK(0x");
        OLED_ShowHexNum(3, 10, lastCrcCmd, 2);
        OLED_ShowChar(3, 12, ')');
    }
    else
    {
        OLED_ShowString(3, 1, "FAIL(0x");
        OLED_ShowHexNum(3, 8, lastCrcCmd, 2);
        OLED_ShowChar(3, 10, ')');
        OLED_ShowString(3, 11, "C:");
        OLED_ShowHexNum(3, 13, lastCrcCalc, 2);

        /* ---- Row4: 收到的 CRC ---- */
        OLED_ShowString(4, 1, "Rcvd:0x");
        OLED_ShowHexNum(4, 8, lastCrcRecv, 2);
    }

    /* ---- Row4: 空闲状态 (CRC 正常时) ---- */
    if (lastCrcStatus != 2)
    {
        if (selectedCount > 0)
            OLED_ShowString(4, 1, "Ready");
        else
            OLED_ShowString(4, 1, "Ready(nosel)");
    }
}

/* ============================================================
 *  主函数
 * ============================================================ */
int main(void)
{
    uint8_t KeyNum;
    uint8_t modbusIdx = 0;
    uint8_t modbusFunc;
    uint8_t addrList[MAX_SLAVES];
    uint8_t slaveCount = 0;                 /* 已发现的从机数 */
    uint8_t cursorIdx  = 0;                 /* 光标位置 */
    uint8_t selectedAddrs[MAX_SLAVES];      /* 选中的从机 */
    uint8_t i;

    /* 初始化 */
    for (i = 0; i < MAX_SLAVES; i++)
        selectedAddrs[i] = 0;

    NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2);
    Timer_Init();
    OLED_Init();
    Key_Init();
    Serial_Init();
    LED_Init();

    ShowStatus(addrList, slaveCount, cursorIdx, selectedAddrs, ModbusFuncList[modbusIdx]);

    /* ============================================================
     *  主循环
     * ============================================================ */
    while (1)
    {
        KeyNum = Key_GetNum();
        modbusFunc = ModbusFuncList[modbusIdx];

        /* ============================================================
         *  KEY1 — 自动地址分配
         *  发送 0xA5 地址分配帧, 等待从机应答
         * ============================================================ */
        if (KeyNum == KEY1)
        {
            uint8_t lastAddr;

            OLED_Clear();
            OLED_ShowString(1, 1, "AddrAlloc...");
            OLED_ShowString(2, 1, "Sending 0xA5");

            LED_ON();
            Serial_SendAddrAlloc();

            OLED_ShowString(3, 1, "Waiting Resp...");
            slaveCount = Serial_ReceiveAddrAllocResp(addrList, &lastAddr);

            LED_OFF();
            OLED_Clear();

            if (slaveCount == 0xFF)
            {
                /* CRC 校验失败 */
                OLED_ShowString(1, 1, "AddrAlloc: FAIL");
                OLED_ShowString(2, 1, "CRC Error!");
                OLED_ShowString(3, 1, "Check wiring");
                slaveCount = 0;
                lastCrcStatus = 2;
                lastCrcCmd    = 0xA5;
                lastCrcCalc   = 0;
                lastCrcRecv   = 0;
            }
            else if (slaveCount == 0)
            {
                /* 超时无应答 */
                OLED_ShowString(1, 1, "AddrAlloc: TIMEOUT");
                OLED_ShowString(2, 1, "No Slave Found");
                OLED_ShowString(3, 1, "Check wiring");
                lastCrcStatus = 2;
                lastCrcCmd    = 0xA5;
                lastCrcCalc   = 0;
                lastCrcRecv   = 0;
            }
            else
            {
                /* 成功 */
                for (i = 0; i < slaveCount; i++)
                    selectedAddrs[i] = 0;
                cursorIdx = 0;
                lastCrcStatus = 1;
                lastCrcCmd    = 0xA5;

                OLED_ShowString(1, 1, "AddrAlloc: OK");
                OLED_ShowString(2, 1, "Slaves:");
                OLED_ShowNum(2, 8, slaveCount, 1);
                OLED_ShowString(2, 10, "CRC:OK");

                /* 显示从机地址列表 (分两行, 每行最多4个) */
                OLED_ShowString(3, 1, "Ad:");
                for (i = 0; i < slaveCount && i < 4; i++)
                {
                    uint8_t col = 4 + i * 3;
                    OLED_ShowHexNum(3, col, addrList[i], 2);
                    if (i < slaveCount - 1 && i < 3)
                        OLED_ShowChar(3, col + 2, ',');
                }
                if (slaveCount > 4)
                {
                    for (i = 4; i < slaveCount && i < 8; i++)
                    {
                        uint8_t col = 1 + (i - 4) * 3;
                        OLED_ShowHexNum(4, col, addrList[i], 2);
                        if (i < slaveCount - 1 && i < 7)
                            OLED_ShowChar(4, col + 2, ',');
                    }
                }
            }
            Delay_ms(1500);
            ShowStatus(addrList, slaveCount, cursorIdx, selectedAddrs, modbusFunc);
        }

        /* ============================================================
         *  KEY2 — 发送心跳帧到选中从机
         *  帧内容: [Modbus功能码, 地址1, 地址2, ...]
         *  重试 3 次, 跟踪 CRC 结果
         * ============================================================ */
        if (KeyNum == KEY2)
        {
            uint8_t selectedList[MAX_SLAVES];
            uint8_t addrCount = 0;

            for (i = 0; i < slaveCount; i++)
                if (selectedAddrs[i])
                    selectedList[addrCount++] = addrList[i];

            if (addrCount > 0)
            {
                uint8_t frame[FRAME_BUF_SIZE];
                uint8_t frameLen;
                uint8_t retry;
                uint8_t success = 0;

                lastCrcStatus = 0;  /* 发送前清空状态 */

                for (retry = 0; retry < 3 && !success; retry++)
                {
                    LED_ON();
                    Serial_SendHeartbeat(modbusFunc, selectedList, addrCount);

                    if (Serial_ReceiveFrame_IT(frame, &frameLen, 300))
                    {
                        lastCrcCmd  = frame[1];
                        lastCrcCalc = CRC8_Calculate(frame + 1, frameLen - 3);
                        lastCrcRecv = frame[frameLen - 2];
                        if (lastCrcCalc == lastCrcRecv)
                        {
                            lastCrcStatus = 1;
                            success = 1;
                        }
                        else
                        {
                            lastCrcStatus = 2;  /* CRC 不匹配, 继续重试 */
                        }
                    }
                    else
                    {
                        lastCrcStatus = 2;      /* 超时 */
                    }
                    LED_OFF();
                }

                if (!success)
                {
                    lastCrcCmd = modbusFunc;
                }
            }
            else
            {
                /* 未选中任何从机 */
                lastCrcStatus = 2;
                lastCrcCmd    = modbusFunc;
                lastCrcCalc   = 0;
                lastCrcRecv   = 0;
            }

            ShowStatus(addrList, slaveCount, cursorIdx, selectedAddrs, modbusFunc);
        }

        /* ============================================================
         *  KEY3 — 上一个 Modbus 功能码
         * ============================================================ */
        if (KeyNum == KEY3)
        {
            if (modbusIdx == 0)
                modbusIdx = MODBUS_FUNC_COUNT - 1;
            else
                modbusIdx--;

            ShowStatus(addrList, slaveCount, cursorIdx, selectedAddrs,
                       ModbusFuncList[modbusIdx]);
            Delay_ms(300);
        }

        /* ============================================================
         *  KEY4 — 下一个 Modbus 功能码
         * ============================================================ */
        if (KeyNum == KEY4)
        {
            if (modbusIdx >= MODBUS_FUNC_COUNT - 1)
                modbusIdx = 0;
            else
                modbusIdx++;

            ShowStatus(addrList, slaveCount, cursorIdx, selectedAddrs,
                       ModbusFuncList[modbusIdx]);
            Delay_ms(300);
        }

        /* ============================================================
         *  KEY6 — 光标移到下一个从机 (PA4)
         * ============================================================ */
        if (KeyNum == KEY6)
        {
            if (slaveCount > 0)
            {
                cursorIdx = (cursorIdx + 1) % slaveCount;
                ShowStatus(addrList, slaveCount, cursorIdx, selectedAddrs, modbusFunc);
                Delay_ms(200);
            }
        }

        /* ============================================================
         *  KEY7 — 光标移到上一个从机 (PA0)
         * ============================================================ */
        if (KeyNum == KEY7)
        {
            if (slaveCount > 0)
            {
                if (cursorIdx == 0)
                    cursorIdx = slaveCount - 1;
                else
                    cursorIdx--;

                ShowStatus(addrList, slaveCount, cursorIdx, selectedAddrs, modbusFunc);
                Delay_ms(200);
            }
        }

        /* ============================================================
         *  KEY5 — 切换当前光标从机的选中状态 (PA2)
         * ============================================================ */
        if (KeyNum == KEY5)
        {
            if (slaveCount > 0)
            {
                selectedAddrs[cursorIdx] = !selectedAddrs[cursorIdx];

                /* 短暂提示 */
                OLED_Clear();
                OLED_ShowString(1, 1, "Slave 0x");
                OLED_ShowHexNum(1, 8, addrList[cursorIdx], 2);
                if (selectedAddrs[cursorIdx])
                    OLED_ShowString(2, 1, "SELECTED!");
                else
                    OLED_ShowString(2, 1, "REMOVED!");
                OLED_ShowNum(3, 1, cursorIdx + 1, 1);
                OLED_ShowChar(3, 3, '/');
                OLED_ShowNum(3, 4, slaveCount, 1);
                Delay_ms(400);
                ShowStatus(addrList, slaveCount, cursorIdx, selectedAddrs, modbusFunc);
            }
        }

        /* ============================================================
         *  KEY8 — 读取选中从机的保持寄存器 (PA1)
         *  发送: funcCode=0x03, regAddr=0, qty=1
         * ============================================================ */
        if (KeyNum == KEY8)
        {
            if (slaveCount > 0 && selectedAddrs[cursorIdx])
            {
                uint8_t frame[FRAME_BUF_SIZE];
                uint8_t frameLen;
                uint8_t regValH, regValL;

                OLED_Clear();
                OLED_ShowString(1, 1, "Read Reg0 from");
                OLED_ShowHexNum(1, 14, addrList[cursorIdx], 2);
                OLED_ShowString(2, 1, "Sending...");

                LED_ON();
                Serial_SendReadRegs(addrList[cursorIdx], 0x03, 0, 1);

                if (Serial_ReceiveFrame_IT(frame, &frameLen, 500))
                {
                    if (CRC8_VerifyFrame(frame, frameLen) && frame[1] == CMD_READ_REGS)
                    {
                        /* 应答: 0x7E | CMD | 0x7F | addr | 0x7F | func | byteCnt | dataH | dataL | 0x7F | CRC | 0x7E */
                        /* 索引:  0     1      2      3      4      5      6        7       8      9    10    11 */
                        if (frameLen >= 12)
                        {
                            regValH = frame[7];   /* 第1个寄存器的高字节 */
                            regValL = frame[8];   /* 第1个寄存器的低字节 */
                        }
                        else
                        {
                            regValH = 0;
                            regValL = 0;
                        }

                        OLED_ShowString(2, 1, "Reg0 = 0x");
                        OLED_ShowHexNum(2, 10, regValH, 2);
                        OLED_ShowHexNum(2, 12, regValL, 2);
                        OLED_ShowString(3, 1, "CRC:OK");
                    }
                    else
                    {
                        OLED_ShowString(2, 1, "CRC FAIL!");
                    }
                    lastCrcStatus = 1;
                    lastCrcCmd    = CMD_READ_REGS;
                }
                else
                {
                    OLED_ShowString(2, 1, "No Response!");
                }
                LED_OFF();
                Delay_ms(1500);
                ShowStatus(addrList, slaveCount, cursorIdx, selectedAddrs, modbusFunc);
            }
            else
            {
                OLED_Clear();
                OLED_ShowString(1, 1, "No slave sel!");
                Delay_ms(800);
                ShowStatus(addrList, slaveCount, cursorIdx, selectedAddrs, modbusFunc);
            }
        }

        /* ============================================================
         *  KEY9 — 写入选中从机的保持寄存器 (PA3)
         *  发送: funcCode=0x06, regAddr=0, value=(addr*256+5)
         * ============================================================ */
        if (KeyNum == KEY9)
        {
            if (slaveCount > 0 && selectedAddrs[cursorIdx])
            {
                uint8_t frame[FRAME_BUF_SIZE];
                uint8_t frameLen;
                uint16_t testVal = (uint16_t)addrList[cursorIdx] * 256 + 0xAA;

                OLED_Clear();
                OLED_ShowString(1, 1, "Write Reg0 to");
                OLED_ShowHexNum(1, 14, addrList[cursorIdx], 2);
                OLED_ShowString(2, 1, "Val=0x");
                OLED_ShowHexNum(2, 6, (uint8_t)(testVal >> 8), 2);
                OLED_ShowHexNum(2, 8, (uint8_t)(testVal), 2);
                OLED_ShowString(3, 1, "Sending...");

                LED_ON();
                Serial_SendWriteRegs(addrList[cursorIdx], 0x06, 0, testVal);

                if (Serial_ReceiveFrame_IT(frame, &frameLen, 500))
                {
                    if (CRC8_VerifyFrame(frame, frameLen) && frame[1] == CMD_WRITE_REGS)
                    {
                        OLED_ShowString(3, 1, "Echo OK!");
                    }
                    else
                    {
                        OLED_ShowString(3, 1, "CRC/ECHO FAIL");
                    }
                    lastCrcStatus = 1;
                    lastCrcCmd    = CMD_WRITE_REGS;
                }
                else
                {
                    OLED_ShowString(3, 1, "No Response!");
                }
                LED_OFF();
                Delay_ms(1500);
                ShowStatus(addrList, slaveCount, cursorIdx, selectedAddrs, modbusFunc);
            }
            else
            {
                OLED_Clear();
                OLED_ShowString(1, 1, "No slave sel!");
                Delay_ms(800);
                ShowStatus(addrList, slaveCount, cursorIdx, selectedAddrs, modbusFunc);
            }
        }

        /* ============================================================
         *  KEY10 — 广播写入所有从机 (PA5)
         *  发送: funcCode=0x10, regAddr=0, value=0xA5A5
         *  所有从机收到后同时写入, 无应答
         * ============================================================ */
        if (KeyNum == KEY10)
        {
            OLED_Clear();
            OLED_ShowString(1, 1, "Bcast Write");
            OLED_ShowString(2, 1, "All:  Reg0=0xA5A5");
            OLED_ShowString(3, 1, "No resp expected");
            LED_ON();
            Serial_SendBcastWrite(0x10, 0, 0xA5A5);
            LED_OFF();
            Delay_ms(500);
            ShowStatus(addrList, slaveCount, cursorIdx, selectedAddrs, modbusFunc);
        }
    }
}
