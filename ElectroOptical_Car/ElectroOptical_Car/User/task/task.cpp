#include "task.h"
#include "ita_car.h"  

Class_EoCar EoCar;
int mpu6050_init_flag=1;
float pitch;
float roll;
float yaw;
int16_t gx,gy,gz,ax,ay,az;
void Chassis_Device_CAN1_Callback(Struct_CAN_Rx_Buffer *CAN_RxMessage)
{
    switch (CAN_RxMessage->Header.StdId)
    {
    }
}
void Task_UART3_Callback(uint8_t *Buffer, uint16_t Length)
{
}
void Task1ms_TIM7_Callback()
{
    EoCar.TIM_Calculate_PeriodElapsedCallback();
		MPU6050_DMP_Get_Data(&pitch, &roll, &yaw,&gx,&gy,&gz,&ax,&ay,&az);
}

extern "C" void Task_Init(void)
{
    //集中总线can1/can2
    CAN_Init(&hcan1, Chassis_Device_CAN1_Callback);
    //定时器循环任务
    TIM_Init(&htim7, Task1ms_TIM7_Callback);
    //串口初始化
     UART_Init(&huart3, Task_UART3_Callback, 15); 
    /********************************* 交互层初始化 *********************************/
    EoCar.Init();
    /********************************* 使能调度时钟 *********************************/
    HAL_TIM_Base_Start_IT(&htim7);
    //PWM初始化
    HAL_TIM_PWM_Start(&htim10,TIM_CHANNEL_1);
    HAL_TIM_PWM_Start(&htim11,TIM_CHANNEL_1);
    HAL_TIM_PWM_Start(&htim9,TIM_CHANNEL_1);
    HAL_TIM_PWM_Start(&htim9,TIM_CHANNEL_2);
    HAL_TIM_PWM_Start(&htim1,TIM_CHANNEL_1);
    HAL_TIM_PWM_Start(&htim1,TIM_CHANNEL_2);
    HAL_TIM_PWM_Start(&htim1,TIM_CHANNEL_3);
    HAL_TIM_PWM_Start(&htim1,TIM_CHANNEL_4);
	
		while(mpu6050_init_flag)
		{
		mpu6050_init_flag = MPU6050_DMP_Init();
		HAL_Delay(1000);
		}
}

void Task_Loop(void)
{
}