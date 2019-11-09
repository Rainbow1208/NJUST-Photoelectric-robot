#include "imu.h"
#include "kalman.h"
#include "math.h"
#include "delay.h"
#include "Euler_angles.h"
/***************************************数据定义区***************************************/

kalman p_IMU;
IMU_RAW_DATA_Typedef   IMU_Raw_Data;
IMU_REAL_DATA_Typedef  IMU_Real_Data;
IMU_REAL_DATA_Typedef  IMU_Offset_Data;

float Y_PPPPPPP;

short PTZ_MPU_Ignore=3;
/***************************************函数处理区***************************************/
/**
  * @brief  MPU6600配置写入  新板
  * @param  void
  * @retval void
  * @notes  Write a register to MPU6600     
  */
uint8_t MPU6500_Write_Reg(uint8_t const reg, uint8_t const data)
{
  MPU6500_NSS_Low();
  SPI4_ReadWriteByte(reg&0x7f);
  SPI4_ReadWriteByte(data);
  MPU6500_NSS_High(); 
  return 0;
}

/**
  * @brief  MPU6500角度获取
  * @param  void
  * @retval void
  * @notes  Read a register from MPU6500      
  */
uint8_t MPU6500_Read_Reg(uint8_t  reg)
{
  uint8_t MPU_Rx;
  
  MPU6500_NSS_Low();
  SPI4_ReadWriteByte(reg|0x80);
  MPU_Rx=SPI4_ReadWriteByte(0xff);
  MPU6500_NSS_High();
  
  return MPU_Rx;
}

/**
  * @brief  MPU6500角度获取
  * @param  void
  * @retval void
  * @notes  Read registers from MPU6500,address begin with regAddr      
  */
uint8_t MPU6500_Read_Regs(uint8_t  regAddr, uint8_t *pData, uint8_t len)
{
  int i; 
  int a;
		
  for (i=0;i<len;i++)
  {
    *pData = MPU6500_Read_Reg(regAddr+i);
    pData++;
    a=10;
    while(a--);
//    delay_ms(10);
  }
  return 0;
}

/**
  * @brief  MPU6500角度获取
  * @param  void
  * @retval void
  * @notes  Get 6 axis data from MPU6500       
  */
void IMU_Get_Data(void)
{
	uint8_t mpu_buff[6];
	uint8_t mpu_buff2[6];

	MPU6500_Read_Regs(MPU6500_GYRO_XOUT_H, mpu_buff, 6);
	 MPU6500_Read_Regs(MPU6500_ACCEL_XOUT_H, mpu_buff2, 6);
	IMU_Raw_Data.Gyro_X = mpu_buff[0] <<8 |mpu_buff[1];
   IMU_Raw_Data.Gyro_Y = mpu_buff[2]<<8 |mpu_buff[3] ;
	IMU_Raw_Data.Gyro_Z = mpu_buff[4]<<8 |mpu_buff[5] ;
	IMU_Raw_Data.Accel_X = mpu_buff2[0] <<8 |mpu_buff2[1];
   IMU_Raw_Data.Accel_Y = mpu_buff2[2]<<8 |mpu_buff2[3] ;
	IMU_Raw_Data.Accel_Z = mpu_buff2[4]<<8 |mpu_buff2[5] ;
	


	
	IMUupdate(IMU_Real_Data.Gyro_X, IMU_Real_Data.Gyro_Y, IMU_Real_Data.Gyro_Z, ax, ay, az);
}

/**
  * @brief  MPU6500角度获取
  * @param  void
  * @retval void
  * @notes  Get 6 axis data from MPU6500       
  */
/*void IMU_Get_Data_Task(void *pvParameters)
{
	uint32_t err;
	while(1)
	{
		err=ulTaskNotifyTake(pdTRUE,portMAX_DELAY);
		if(err==1)
		{
			IMU_Get_Data();
			USART3_Send_NUC();
		}
	}
}
*/
/**
  * @brief  MPU6500初始化角度处理
  * @param  void
  * @retval void
  * @notes  void       
  */
void MPU6500_Gyro_Cali(void)
{
  u16 i;
  int num=300;
  float Gyro_X_temp=0,Gyro_Y_temp=0,Gyro_Z_temp=0,ax=0,ay=0,az=0;
  
  IMU_Offset_Data.Gyro_X = 0;
  IMU_Offset_Data.Gyro_Y = 0;
  IMU_Offset_Data.Gyro_Z = 0;
	IMU_Offset_Data.Accel_X= 0;
	IMU_Offset_Data.Accel_Y= 0;
	IMU_Offset_Data.Accel_Z= 0;
  for (i=0;i<num;i++)
  {
    IMU_Get_Data();
    Gyro_X_temp += IMU_Raw_Data.Gyro_X;
    Gyro_Y_temp += IMU_Raw_Data.Gyro_Y;
    Gyro_Z_temp += IMU_Raw_Data.Gyro_Z;
		ax+=IMU_Raw_Data.Accel_X;
		ay+=IMU_Raw_Data.Accel_Y;
		az+=IMU_Raw_Data.Accel_Z;
		delay_ms(2);
  }
  IMU_Offset_Data.Gyro_X =  Gyro_X_temp/num;
  IMU_Offset_Data.Gyro_Y =  Gyro_Y_temp/num;
  IMU_Offset_Data.Gyro_Z =  Gyro_Z_temp/num;
	IMU_Offset_Data.Accel_X=  ax/num;
	IMU_Offset_Data.Accel_Y=  ay/num;
	IMU_Offset_Data.Accel_Z=  az/num;
}


/**
  * @brief  IMU初始化
  * @param  void
  * @retval void
  * @notes  Initialize the MPU6500  500Hz采样率       
  */
uint8_t IMU_Init(void)
{

	uint8_t index = 0;
	uint8_t MPU6500_Init_Data[10][2] = 
	{
		{MPU6500_PWR_MGMT_1,    0x83},      // 6500第41页   Reset Device       Auto selects the best available clock source – PLL if ready, else use the Internal oscillator
//		{MPU6500_SMPLRT_DIV,    0x07},      // Clock Source - Gyro-Z
//		{MPU6500_SMPLRT_DIV,    0x01},
//		{MPU6500_CONFIG,        0x03},      // Enable Acc & Gyro
		{MPU6500_SMPLRT_DIV,    0x02},      // 6500第12页   采样频率分频器 SAMPLE_RATE =INTERNAL_SAMPLE_RATE / (1 + SMPLRT_DIV)   INTERNAL_SAMPLE_RATE = 1kHz
		{MPU6500_CONFIG,        0x03},      // 6500第13页   配置寄存器
		{MPU6500_ACCEL_CONFIG,  0x08},      // 6500第14页   加速度量程±4g
		{MPU6500_ACCEL_CONFIG_2,0x01},      // 6500第15页   enable LowPassFilter  Set Acc LPF
		{MPU6500_GYRO_CONFIG,   0x08},      // 6500第14页   陀螺仪量程±500dps
		{MPU6500_INT_PIN_CFG,   0x02},      // 6500第29页   enable I2C_MASTER interface pins (ES_CL and ES_DA)
		{MPU6500_INT_ENABLE,    0x01},      // 6500第29页   Enable Raw Sensor Data Ready interrupt to propagate to interrupt pin.
		{MPU6500_USER_CTRL,     0x20},      // 6500第40页   Enable AUX
   //  {MPU6500_PWR_MGMT_2,    0x00},      // Enable Acc & Gyro
{MPU6500_PWR_MGMT_2,    0x38},      // 6500第42页
	};  

	
	while(MPU6500_ID != MPU6500_Read_Reg(MPU6500_WHO_AM_I))
    {
		//LED_RED_ON;	   
	printf("MPU6500_id=%d\r\n",MPU6500_ID);
	printf("MPU_id=%d\r\n",MPU6500_Read_Reg(MPU6500_WHO_AM_I));
    };  //read id of device,check if MPU6500 or not
	//  printf("MPU_id=%d\r\n",MPU_id);
		//LED_RED_OFF;

	for(index = 0; index < 10; index++)//配置6500
	{
		MPU6500_Write_Reg(MPU6500_Init_Data[index][0], MPU6500_Init_Data[index][1]);
		delay_ms(1);
	}
	delay_ms(15);
	MPU6500_Gyro_Cali();
	//  IST8310_Init();
	MPU_6500_Interrupt_InitConfig();
	kalmanCreate(&p_IMU,1,40);
	return 0;
}

/**
  * @brief  6600数据溢出中断对应的输入捕获初始化
  * @param  void
  * @retval void
  * @notes  EXTI_Line8-->PB8         
  */
void MPU_6500_Interrupt_InitConfig(void)
{
	GPIO_InitTypeDef GPIO_InitStruct;
	EXTI_InitTypeDef EXIT_InitStruct;
	NVIC_InitTypeDef NVIC_InitStruct;

	RCC_APB2PeriphClockCmd(RCC_APB2Periph_SYSCFG,ENABLE );
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOB,ENABLE );

	GPIO_InitStruct.GPIO_Pin = GPIO_Pin_8;
	GPIO_InitStruct.GPIO_Mode = GPIO_Mode_IN;
	GPIO_InitStruct.GPIO_OType = GPIO_OType_PP;
	GPIO_InitStruct.GPIO_PuPd = GPIO_PuPd_UP;
	GPIO_InitStruct.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init (GPIOB,&GPIO_InitStruct);

	SYSCFG_EXTILineConfig(EXTI_PortSourceGPIOB ,GPIO_PinSource8);

	EXIT_InitStruct.EXTI_Line = EXTI_Line8;
	EXIT_InitStruct.EXTI_Mode = EXTI_Mode_Interrupt;
	EXIT_InitStruct.EXTI_Trigger = EXTI_Trigger_Falling;
	EXIT_InitStruct.EXTI_LineCmd = ENABLE;
	EXTI_Init(&EXIT_InitStruct);

	NVIC_InitStruct.NVIC_IRQChannel = EXTI9_5_IRQn;
	NVIC_InitStruct.NVIC_IRQChannelPreemptionPriority = 7;
	NVIC_InitStruct.NVIC_IRQChannelSubPriority = 0;
	NVIC_InitStruct.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Init(&NVIC_InitStruct); 
}

/**
  * @brief  定时器4外部中断处理函数()输入捕获  1KHz
  * @param  void
  * @retval void
  * @notes  
  */
void EXTI9_5_IRQHandler(void)
{
	/*BaseType_t pxHigherPriorityTaskWoken;
   if (EXTI_GetITStatus (EXTI_Line8 ) == SET)
   {
     vTaskNotifyGiveFromISR(IMU_Get_Data_Task_Handler,&pxHigherPriorityTaskWoken);
		
   }
	*/
   EXTI_ClearITPendingBit(EXTI_Line8);
}

