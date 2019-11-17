#include "sys.h"
#include "delay.h"
#include "usart.h"
#include "math.h"
#include "Exti.h"
#define green 1
#define white 2
#define red 3
#define black 4
#define blue 5
#define SEN GPIO_ReadInputDataBit//传感器读数
u16 amount=0;
u16 Ramount=0,Gamount=0,Bamount=0;
u16 Rgena=21,Ggena=22,Bgena=27;
void TCS230_Init()
{
	GPIO_InitTypeDef  GPIO_InitStructure;
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA, ENABLE);//使能 GPIOA 时钟 
	//S0--->PA9		S1--->PA10		S2--->PA11		S3--->PA12
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_9|GPIO_Pin_10|GPIO_Pin_11|GPIO_Pin_12;				 
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;//普通输出模式
  GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;//推挽输出
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;//100MHz
  GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;//上拉
  GPIO_Init(GPIOA, &GPIO_InitStructure);//初始化 GPIO			
	GPIO_SetBits(GPIOA,GPIO_Pin_10);
	GPIO_ResetBits(GPIOA,GPIO_Pin_9);
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOG, ENABLE);
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_3;				 
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN;//普通输出模式
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;//100MHz
  GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;//上拉
  GPIO_Init(GPIOG, &GPIO_InitStructure);
}
void exit_init()
{
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_SYSCFG, ENABLE);
	NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2);
	NVIC_InitTypeDef   NVIC_InitStructure;
	EXTI_InitTypeDef   EXTI_InitStructure;
	SYSCFG_EXTILineConfig(EXTI_PortSourceGPIOG, EXTI_PinSource3);//不确定(中断线连接)
	EXTI_InitStructure.EXTI_Line = EXTI_Line3;//不定（中断线）
  EXTI_InitStructure.EXTI_Mode = EXTI_Mode_Interrupt;//输入
  EXTI_InitStructure.EXTI_Trigger = EXTI_Trigger_Rising; //上升沿触发
  EXTI_InitStructure.EXTI_LineCmd = ENABLE;//使能
  EXTI_Init(&EXTI_InitStructure);//初始化中断线
	NVIC_InitStructure.NVIC_IRQChannel = EXTI3_IRQn;//中断线连接（不确定）
  NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0x02;//抢占优先级2
  NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0x02;//响应优先级2
  NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;//使能
  NVIC_Init(&NVIC_InitStructure);//初始化
}
//舵机初始化
void steering_init()
{
		GPIO_InitTypeDef GPIO_InitStructure;
		TIM_TimeBaseInitTypeDef TIM_TimeBaseStructure;
		TIM_OCInitTypeDef TIM_OCInitStructure;
		RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM4,ENABLE);//TIM10 时钟使能
		RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOD, ENABLE); //使能 PORTF 时
		GPIO_PinAFConfig(GPIOD,GPIO_PinSource15,GPIO_AF_TIM4);
		GPIO_PinAFConfig(GPIOD,GPIO_PinSource14,GPIO_AF_TIM4);	
		GPIO_PinAFConfig(GPIOD,GPIO_PinSource13,GPIO_AF_TIM4); //GF9 复用为 TIM10
		GPIO_PinAFConfig(GPIOD,GPIO_PinSource12,GPIO_AF_TIM4); //GF9 复用为 TIM10
		GPIO_InitStructure.GPIO_Pin = GPIO_Pin_13 | GPIO_Pin_12 | GPIO_Pin_14 |GPIO_Pin_15; //GPIOF6
		GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF; //复用功能
		GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz; //速度 50MHz
		GPIO_InitStructure.GPIO_OType = GPIO_OType_PP; //推挽复用输出
		GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP; //上拉
		GPIO_Init(GPIOD,&GPIO_InitStructure); //初始化 PF9
		TIM_TimeBaseStructure.TIM_Prescaler=84-1; //定时器分频    2k
		TIM_TimeBaseStructure.TIM_CounterMode=TIM_CounterMode_Up; //向上计数模式
		TIM_TimeBaseStructure.TIM_Period=20000-1; //自动重装载值
		TIM_TimeBaseStructure.TIM_ClockDivision=TIM_CKD_DIV1;
		TIM_TimeBaseInit(TIM4,&TIM_TimeBaseStructure);//初始化定时器 14
		//初始化 TIM14 Channel1 PWM 模式
		TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_PWM1; //PWM 调制模式 1
		TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable; //比较输出使能
		TIM_OCInitStructure.TIM_OCPolarity = TIM_OCPolarity_High; //输出极性高
		TIM_OC1Init(TIM4, &TIM_OCInitStructure); //初始化外设 TIM1 4OC1
		TIM_OC1PreloadConfig(TIM4, TIM_OCPreload_Enable); //使能预装载寄存器
		TIM_OC2Init(TIM4, &TIM_OCInitStructure); //初始化外设 TIM1 4OC1
		TIM_OC2PreloadConfig(TIM4, TIM_OCPreload_Enable); //使能预装载寄存器
		TIM_OC3Init(TIM4, &TIM_OCInitStructure); //初始化外设 TIM1 4OC1
		TIM_OC3PreloadConfig(TIM4, TIM_OCPreload_Enable); //使能预装载寄存器
		TIM_OC4Init(TIM4, &TIM_OCInitStructure); //初始化外设 TIM1 4OC1
		TIM_OC4PreloadConfig(TIM4, TIM_OCPreload_Enable); //使能预装载寄存器
		TIM_ARRPreloadConfig(TIM4,ENABLE);//ARPE 使能
		TIM_Cmd(TIM4, ENABLE); //使能 TIM11
}
//电机初始化
void motor_init()
{
		GPIO_InitTypeDef GPIO_InitStructure;
		RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOF, ENABLE);//使能 GPIOF 时钟
		GPIO_InitStructure.GPIO_Pin = GPIO_Pin_2|GPIO_Pin_4|GPIO_Pin_1;
		GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;//普通输出模式
		GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;//推挽输出
		GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;//100MHz
		GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;//上拉
		GPIO_Init(GPIOF, &GPIO_InitStructure);//初始化 GPIO
		RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOI, ENABLE);//使能 GPIOI 时钟
		GPIO_InitStructure.GPIO_Pin = GPIO_Pin_9|GPIO_Pin_11;//STBY
		GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;//普通输出模式
		GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;//推挽输出
		GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;//100MHz
		GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;//上拉
		GPIO_Init(GPIOI, &GPIO_InitStructure);//初始化 GPIO
		TIM_TimeBaseInitTypeDef TIM_TimeBaseStructure;
		TIM_OCInitTypeDef TIM_OCInitStructure;
		RCC_APB2PeriphClockCmd(RCC_APB2Periph_TIM10,ENABLE);//TIM10 时钟使能
		RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOF, ENABLE); //使能 PORTF 时钟
		GPIO_PinAFConfig(GPIOF,GPIO_PinSource6,GPIO_AF_TIM10); //GF9 复用为 TIM10
		GPIO_InitStructure.GPIO_Pin = GPIO_Pin_6; //GPIOF6
		GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF; //复用功能
		GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz; //速度 50MHz
		GPIO_InitStructure.GPIO_OType = GPIO_OType_PP; //推挽复用输出
		GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP; //上拉
		GPIO_Init(GPIOF,&GPIO_InitStructure); //初始化 PF9
		TIM_TimeBaseStructure.TIM_Prescaler=84-1; //定时器分频    2k
		TIM_TimeBaseStructure.TIM_CounterMode=TIM_CounterMode_Up; //向上计数模式
		TIM_TimeBaseStructure.TIM_Period=500-1; //自动重装载值
		TIM_TimeBaseStructure.TIM_ClockDivision=TIM_CKD_DIV1;
		TIM_TimeBaseInit(TIM10,&TIM_TimeBaseStructure);//初始化定时器 10
		//初始化 TIM10 Channel1 PWM 模式
		TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_PWM1; //PWM 调制模式 1
		TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable; //比较输出使能
		TIM_OCInitStructure.TIM_OCPolarity = TIM_OCPolarity_High; //输出极性高
		TIM_OC1Init(TIM10, &TIM_OCInitStructure); //初始化外设 TIM1 4OC1
		TIM_OC1PreloadConfig(TIM10, TIM_OCPreload_Enable); //使能预装载寄存器
		TIM_ARRPreloadConfig(TIM10,ENABLE);//ARPE 使能
		TIM_Cmd(TIM10, ENABLE); //使能 TIM10
		RCC_APB2PeriphClockCmd(RCC_APB2Periph_TIM11,ENABLE);//TIM11 时钟使能
		RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOF, ENABLE); //使能 PORTF 时钟
		GPIO_PinAFConfig(GPIOF,GPIO_PinSource7,GPIO_AF_TIM11); //GF7 复用为 TIM11
		GPIO_InitStructure.GPIO_Pin = GPIO_Pin_7; //GPIOF7
		GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF; //复用功能
		GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz; //速度 50MHz
		GPIO_InitStructure.GPIO_OType = GPIO_OType_PP; //推挽复用输出
		GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP; //上拉
		GPIO_Init(GPIOF,&GPIO_InitStructure); //初始化 PF7
		TIM_TimeBaseStructure.TIM_Prescaler=84-1; //定时器分频    2k
		TIM_TimeBaseStructure.TIM_CounterMode=TIM_CounterMode_Up; //向上计数模式
		TIM_TimeBaseStructure.TIM_Period=500-1; //自动重装载值
		TIM_TimeBaseStructure.TIM_ClockDivision=TIM_CKD_DIV1;
		TIM_TimeBaseInit(TIM11,&TIM_TimeBaseStructure);//初始化定时器 11
		//初始化 TIM11 Channel1 PWM 模式
		TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_PWM1; //PWM 调制模式 1
		TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable; //比较输出使能
		TIM_OCInitStructure.TIM_OCPolarity = TIM_OCPolarity_High; //输出极性高
		TIM_OC1Init(TIM11, &TIM_OCInitStructure); //初始化外设 TIM1 4OC1
		TIM_OC1PreloadConfig(TIM11, TIM_OCPreload_Enable); //使能预装载寄存器
		TIM_ARRPreloadConfig(TIM11,ENABLE);//ARPE 使能
		TIM_Cmd(TIM11, ENABLE); //使能 TIM11
}
//电机控制
void motor_control(int pwm_left,int pwm_right)
{
		TIM_SetCompare1(TIM10,pwm_right);
		TIM_SetCompare1(TIM11,pwm_left);
}
//传感器初始化
void sensor_init()
{
    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOE, ENABLE);
		RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOF, ENABLE);
		RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOG, ENABLE);
		RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOB, ENABLE);
		//GPIOE
		GPIO_InitTypeDef GPIO_InitStructureE;
		GPIO_InitStructureE.GPIO_Pin = GPIO_Pin_7|GPIO_Pin_8;
		GPIO_InitStructureE.GPIO_Mode = GPIO_Mode_IN;
		GPIO_InitStructureE.GPIO_Speed = GPIO_Speed_100MHz;
		GPIO_InitStructureE.GPIO_PuPd = GPIO_PuPd_UP;
		GPIO_Init(GPIOE, &GPIO_InitStructureE);
		//GPIOF
		GPIO_InitTypeDef GPIO_InitStructureF;
		GPIO_InitStructureF.GPIO_Pin = GPIO_Pin_12|GPIO_Pin_13|GPIO_Pin_14|GPIO_Pin_15|GPIO_Pin_11;
		GPIO_InitStructureF.GPIO_Mode = GPIO_Mode_IN;
		GPIO_InitStructureF.GPIO_Speed = GPIO_Speed_100MHz;
		GPIO_InitStructureF.GPIO_PuPd = GPIO_PuPd_UP;
		GPIO_Init(GPIOF, &GPIO_InitStructureF);
		//GPIOG
	  GPIO_InitTypeDef GPIO_InitStructureG;
		GPIO_InitStructureG.GPIO_Pin = GPIO_Pin_1|GPIO_Pin_0;
		GPIO_InitStructureG.GPIO_Mode = GPIO_Mode_IN;
		GPIO_InitStructureG.GPIO_Speed = GPIO_Speed_100MHz;
		GPIO_InitStructureG.GPIO_PuPd = GPIO_PuPd_UP;
		GPIO_Init(GPIOG, &GPIO_InitStructureG);
		//GPIOB
		GPIO_InitTypeDef GPIO_InitStructureB;
		GPIO_InitStructureB.GPIO_Pin = GPIO_Pin_0;
		GPIO_InitStructureB.GPIO_Mode = GPIO_Mode_IN;
		GPIO_InitStructureB.GPIO_Speed = GPIO_Speed_100MHz;
		GPIO_InitStructureB.GPIO_PuPd = GPIO_PuPd_UP;
		GPIO_Init(GPIOB, &GPIO_InitStructureB);
}
//刹车
void stop()
{
		GPIO_SetBits(GPIOF,GPIO_Pin_4|GPIO_Pin_1|GPIO_Pin_2);
		GPIO_SetBits(GPIOI,GPIO_Pin_11|GPIO_Pin_9);
}
//向前循迹
void track()
{
		GPIO_SetBits(GPIOI,GPIO_Pin_9);
		GPIO_ResetBits(GPIOF,GPIO_Pin_2);
		GPIO_SetBits(GPIOF,GPIO_Pin_4|GPIO_Pin_1);
		GPIO_ResetBits(GPIOI,GPIO_Pin_11);
		int error;
		int lasterror;
		int kp=40;            //比例
		int kd=20;             //微分
		int derivative=0;      //导数值
		int pid=0;	
		int b;  
		b=((SEN(GPIOF,GPIO_Pin_15)<<3)|(SEN(GPIOF,GPIO_Pin_11)<<2)|(SEN(GPIOE,GPIO_Pin_7)<<1)|(SEN(GPIOE,GPIO_Pin_8)));
		int x=b;
		if( x==0x00) error=0;
		if( x==0x00) error=0;
		else if(x==0x0F) error=0;
		else if(x==0x06) error=0;
		else if(x==0x09) error=0;
		else if(x==0x02) error=1;
		else if(x==0x03) error=5;
		else if(x==0x01) error=10; 
		else if(x==0x04) error=-1;
		else if(x==0x0C) error=-5;
		else if(x==0x08) error=-10; 
		else error=0;
		
		derivative=error-lasterror;
		pid=kp*error+kd*derivative;
		pid=pid/8;
		 if(pid>50)
			{
			 pid=50;
			}
		 else if(pid<(-50))
			{
			 pid=-50;          
			}
		lasterror=error;
		motor_control(90+pid,100-pid);
}
//后退循迹
void back_track()
{
		GPIO_SetBits(GPIOI,GPIO_Pin_9|GPIO_Pin_11);
		GPIO_ResetBits(GPIOF,GPIO_Pin_1|GPIO_Pin_4);
		GPIO_SetBits(GPIOF,GPIO_Pin_2);
		int error;
		int lasterror;
		int kp=40;            //比例         
		int kd=200;             //微分
		int derivative=0;      //导数值
		int pid=0;	
		int b;  
		b=((SEN(GPIOG,GPIO_Pin_1)<<3)|(SEN(GPIOF,GPIO_Pin_14)<<2)|(SEN(GPIOF,GPIO_Pin_12)<<1)|(SEN(GPIOF,GPIO_Pin_13)));
		int x=b;
		if( x==0x00) error=0;
		if( x==0x00) error=0;
		else if(x==0x0F) error=0;
		else if(x==0x06) error=0;
		else if(x==0x09) error=0;
		else if(x==0x02) error=1;
		else if(x==0x03) error=5;
		else if(x==0x01) error=15; 
		else if(x==0x04) error=-1;
		else if(x==0x0C) error=-5;
		else if(x==0x08) error=-15; 
		else error=0;
		
		derivative=error-lasterror;
		pid=kp*error+kd*derivative;
		pid=pid/8;
		 if(pid>60)
			{
			 pid=60;
			}
		 else if(pid<(-60))
			{
			 pid=-60;          
			}
		lasterror=error;
		motor_control(100-pid,100+pid);
}
//前进计数
void count(int num)
{
	int b=0;
	while(1)
	{
		track();
		if(SEN(GPIOG,GPIO_Pin_0))
		{
			
			b++;
			if(b==num)
			{
				stop();
				delay_ms(60);
				break;
			}
			delay_ms(80);
			while(SEN(GPIOG,GPIO_Pin_0))
			{
				track();
				delay_ms(3);
			}
		}
		delay_ms(10);
	}
}
//后退计数
void back_count(int num)
{
	int b=0;
	while(1)
	{
		back_track();
		if(SEN(GPIOG,GPIO_Pin_0))
		{
			b++;
			if(b==num)
			{
				stop();
				delay_ms(1000);
				break;
			}
			delay_ms(90);
			while(SEN(GPIOG,GPIO_Pin_0))
			{
				back_track();
				delay_ms(3);
			}
		}
		delay_ms(10);
	}
}
//右侧机械臂捕获
void capture_right()
{
	TIM_SetCompare2(TIM4,2000);
	delay_ms(2000);
	TIM_SetCompare1(TIM4,2000);
	delay_ms(2000);
}
//右侧机械臂释放
void right_free()
{
	TIM_SetCompare2(TIM4,1300);
	delay_ms(1000);
	TIM_SetCompare1(TIM4,1600);
	delay_ms(1000);
}
//左转向
void left_turn(int num)
{
		GPIO_SetBits(GPIOI,GPIO_Pin_9);
		GPIO_ResetBits(GPIOF,GPIO_Pin_2|GPIO_Pin_1);
		GPIO_SetBits(GPIOF,GPIO_Pin_4);
		GPIO_SetBits(GPIOI,GPIO_Pin_11);
		int b=0;
	while(1)
	{
		motor_control(120,120);
		if(SEN(GPIOG,GPIO_Pin_0))
		{
			
			b++;
			if(b==num)
			{
				stop();
				delay_ms(60);
				break;
			}
			delay_ms(80);
			while(SEN(GPIOG,GPIO_Pin_0))
			{
				motor_control(120,120);
				delay_ms(3);
			}
		}
		delay_ms(10);
	}
}
//右转向
void right_turn(int num)
{
		GPIO_SetBits(GPIOI,GPIO_Pin_9);
		GPIO_ResetBits(GPIOF,GPIO_Pin_4);
		GPIO_SetBits(GPIOF,GPIO_Pin_2|GPIO_Pin_1);
		GPIO_ResetBits(GPIOI,GPIO_Pin_11);
		int b=0;
	while(1)
	{
		motor_control(120,120);
		if(SEN(GPIOG,GPIO_Pin_0))
		{
			
			b++;
			if(b==num)
			{
				stop();
				delay_ms(60);
				break;
			}
			delay_ms(80);
			while(SEN(GPIOG,GPIO_Pin_0))
			{
				motor_control(120,120);
				delay_ms(3);
			}
		}
		delay_ms(10);
	}
}
void EXTI3_IRQHandler(void)
{
   EXTI_ClearITPendingBit(EXTI_Line3);   
	if(amount<999)
	{
		amount++;
	}
}
/*void WhiteBalance(void)
{
	GPIO_ResetBits(GPIOA,GPIO_Pin_11|GPIO_Pin_12);
	amount=0;			 //开始计数
	delay_ms(10);
	Rgena = amount;
Ramount=amount;	//求出红色因子      
	amount=0;

	GPIO_SetBits(GPIOA,GPIO_Pin_11|GPIO_Pin_12);
	amount=0;
	delay_ms(10);
	Ggena = amount;
Gamount=amount;	//求出绿色因子
	amount=0;

	GPIO_ResetBits(GPIOA,GPIO_Pin_11);
	GPIO_SetBits(GPIOA,GPIO_Pin_12);
	amount=0;
	delay_ms(10);
	Bgena = amount;	  
	Bamount=amount;//求出蓝色因子
	amount=0;
}*/
u16 tcs230_RED(void)
{
	GPIO_ResetBits(GPIOA,GPIO_Pin_11|GPIO_Pin_12);;
	amount=0;
	delay_ms(30);
	GPIO_ResetBits(GPIOA,GPIO_Pin_12);
	GPIO_SetBits(GPIOA,GPIO_Pin_11);//关闭通道  	 
	Ramount=(u32) amount*255/Rgena;	 //取R值
	if(Ramount>255) Ramount = 255;
	return Ramount;
}

u16 tcs230_GREEN(void)
{
	GPIO_SetBits(GPIOA,GPIO_Pin_11|GPIO_Pin_12);
	amount=0;
	delay_ms(30);
	GPIO_ResetBits(GPIOA,GPIO_Pin_12);
	GPIO_SetBits(GPIOA,GPIO_Pin_11);//关闭通道  	 
	Gamount=(u32) amount*255/Ggena;	//取G值
	if(Gamount>255) Gamount = 255;
	return Gamount;
}

u16 tcs230_BLUE(void)
{
	GPIO_ResetBits(GPIOA,GPIO_Pin_11);
	GPIO_SetBits(GPIOA,GPIO_Pin_12);
	amount=0;
	delay_ms(30);
	GPIO_ResetBits(GPIOA,GPIO_Pin_12);
	GPIO_SetBits(GPIOA,GPIO_Pin_11);//关闭通道  	 
	Bamount=(u32) amount*255/Bgena;//取B值	
	if(Bamount>255) Bamount = 255;
	return Bamount;
} 

int get_color()
{
	while(1)
	{
		Ramount=tcs230_RED();
	Gamount=tcs230_GREEN();
	Bamount=tcs230_BLUE();
	if(Gamount>Ramount&&Gamount>Bamount&&Gamount>=90)
	{
		return green;
	}
	if(Ramount<=110&&Gamount<=110&&Bamount<=100)
	{
		return black;
	}
	if(Ramount>=230&&Gamount>=230&&Bamount>=230)
		{
		return white;
		}
	if(Ramount-Gamount>=40&&Ramount-Bamount>=40)
		{
		return red;
		}
	
	if(Bamount>Ramount&&Ramount<=170&&Bamount-Gamount>=15)
	{
		return blue;
	}
	delay_ms(200);
}
	
}
int main()
{      
    delay_init(168);		
		motor_init();
		sensor_init();
		steering_init();
		TCS230_Init();
		exit_init();
		int color;
	while(1)
	{	
		color=get_color();
		if(color==red) track();
		else if(color==blue) back_track();
		//else stop();
	}	
}
