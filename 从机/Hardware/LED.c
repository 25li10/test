#include "stm32f10x.h"                  // Device header
#include "Delay.h"

void LED_Init(void)
{
	
	GPIO_InitTypeDef GPIO_InitStructure;
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOC,ENABLE);

	GPIO_InitStructure.GPIO_Mode=GPIO_Mode_Out_PP;
	GPIO_InitStructure.GPIO_Pin=GPIO_Pin_13;
	GPIO_InitStructure.GPIO_Speed=GPIO_Speed_50MHz;
	GPIO_Init(GPIOC,&GPIO_InitStructure);
	
	GPIO_SetBits(GPIOC,GPIO_Pin_13); 
}

void LED_ON(void)
{
	GPIO_ResetBits(GPIOC,GPIO_Pin_13);
}
void LED_OFF(void)
{
	GPIO_SetBits(GPIOC,GPIO_Pin_13);
}
void LED_Turn(void)
{
	if(GPIO_ReadOutputDataBit(GPIOC,GPIO_Pin_13)==0)
	{
		GPIO_SetBits(GPIOC,GPIO_Pin_13);
	}
	else
	{
		GPIO_ResetBits(GPIOC,GPIO_Pin_13);
	}

}

/* LED 闪烁指定次数 (亮200ms, 灭300ms, 最后保持灭) */
void LED_Blink(uint8_t times)
{
    uint8_t i;
    for (i = 0; i < times; i++)
    {
        LED_ON();
        Delay_ms(200);
        LED_OFF();
        if (i < times - 1)
            Delay_ms(300);
    }
}
