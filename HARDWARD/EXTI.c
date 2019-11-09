#include"Exti.h"
#include"delay.h"
#include"main.h"
u16 Ramount=0,Gamount=0,Bamount=0;//用于存放读取的RGB值
u16 amount=0;//中断计数

void EXTI2_IRQHandler(void)
{
   EXTI_ClearITPendingBit(EXTI_Line2);    //清除LINE4上的中断标志位
	if(amount<999999)
	{
		amount++;
	}
}

void EXTIX_Init(void)
{
   NVIC_InitTypeDef NVIC_InitStructure;
	 EXTI_InitTypeDef EXTI_InitStructure;
	 RCC_APB2PeriphClockCmd(RCC_APB2Periph_SYSCFG,ENABLE);
	 
	 SYSCFG_EXTILineConfig(EXTI_PortSourceGPIOG,EXTI_PinSource2);
	 
	 
     /* ??EXTI_Line0 */
  EXTI_InitStructure.EXTI_Line = EXTI_Line2;//LINE0
  EXTI_InitStructure.EXTI_Mode = EXTI_Mode_Interrupt;//????
  EXTI_InitStructure.EXTI_Trigger = EXTI_Trigger_Rising; //????? 
  EXTI_InitStructure.EXTI_LineCmd = ENABLE;//??LINE0
  EXTI_Init(&EXTI_InitStructure);//??
	
 
	NVIC_InitStructure.NVIC_IRQChannel = EXTI2_IRQn;//????0
  NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0x00;//?????0
  NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0x02;//????2
  NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;//????????
  NVIC_Init(&NVIC_InitStructure);//??
	
	
}
