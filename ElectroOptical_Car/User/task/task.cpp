#include "task.h"
#include "ita_car.h"  

Class_EoCar EoCar;
uint8_t mod10;
uint8_t lock_flag;
float yaw=135;
float pitch=90;
float yaw_last=135;
float pitch_last=90;

//ESP32相关变量
struct Struct_ESP32_Data
{ 	
		uint8_t header;
		uint8_t Key;
		uint8_t X;
		uint8_t Y;
		uint8_t re;
    uint8_t Frame_tail;
} __attribute__((packed)); 


Struct_ESP32_Data ESP32_Data;
uint16_t ESP32_Online_flag;
uint16_t Pre_ESP32_Online_flag;




void Chassis_Device_CAN1_Callback(Struct_CAN_Rx_Buffer *CAN_RxMessage)
{
		uint32_t temp_id;
		if(CAN_RxMessage->Header.IDE == CAN_ID_STD)
	{
		temp_id = CAN_RxMessage->Header.StdId;
//        switch (temp_id)
//        {
//            case (0xa1):
//            { 
//								EoCar.Minipc.Navigation_Data_Process(CAN_RxMessage->Data);
//								EoCar.Chassis.Target_Velocity_X=EoCar.Minipc.Get_Velocity_X();
//								EoCar.Chassis.Target_Velocity_Y=EoCar.Minipc.Get_Velocity_Y();
//								EoCar.Chassis.Target_Omega=EoCar.Minipc.Get_Velocity_Z();
//            }
//            break;
//            case (0xa2):
//            {
//               EoCar.Minipc.Selfaiming_Data_Process(CAN_RxMessage->Data);
//            }
//            break;
//        }
	}
	
}

void Task_UART3_Callback(uint8_t *Buffer, uint16_t Length)
{
	 EoCar.Minipc.MiniPc_UART_RxCpltCallback(Buffer);
}
void Task_UART6_Callback(uint8_t *Buffer, uint16_t Length)
{
	    if((Buffer[0]==0xA5)&&(Buffer[5]==0x5A)&&(Buffer[1]==1))
			{
			memcpy(&ESP32_Data, &Buffer[0], sizeof(Struct_ESP32_Data));		
			ESP32_Online_flag++;
			EoCar.Chassis.Target_Velocity_X=(float)((ESP32_Data.X-100)/50.f);
			EoCar.Chassis.Target_Velocity_Y=(float)((ESP32_Data.Y-100)/50.f);
			}
				
}
void Unlock_task()//扫描任务
{
		static uint8_t yaw_flag;
		static uint8_t picth_flag;
	if(yaw_flag)
	{
			yaw+=0.08;
		if(yaw>=270)yaw_flag=0;
	}
	else
	{
		yaw-=0.08;
	if(yaw<=0)yaw_flag=1;
	}
	
	if(picth_flag)
	{
			pitch+=0.05;
		if(pitch>=120)picth_flag=0;
	}
	else
	{
		pitch-=0.05;
	if(pitch<=85)picth_flag=1;
	}
}


void Task1ms_TIM6_Callback()
{	
	
EoCar.Minipc.UART_Tx_UI();	
	if(lock_flag)//锁定标志位
	{
		
	}
	else
	Unlock_task();

	
	
	EoCar.TIM_Calculate_PeriodElapsedCallback();
}

uint16_t GetCCRFromAngle(float InputAngle,float Maxangle){
float Ret=InputAngle/Maxangle * 2000 +500;
return Ret;
}

extern "C" void Task_Init(void)
{
    //集中总线can1/can2
      CAN_Init(&hcan1, Chassis_Device_CAN1_Callback);
    //定时器循环任务
    TIM_Init(&htim6, Task1ms_TIM6_Callback);
    //串口初始化
     UART_Init(&huart4, Task_UART3_Callback, 24); 
		 UART_Init(&huart6, Task_UART6_Callback, 10); 
	
			
    /********************************* 交互层初始化 *********************************/
    EoCar.Init();
	    //PWM初始化
    HAL_TIM_PWM_Start(&htim10,TIM_CHANNEL_1);
    HAL_TIM_PWM_Start(&htim11,TIM_CHANNEL_1);
    HAL_TIM_PWM_Start(&htim9,TIM_CHANNEL_1);
    HAL_TIM_PWM_Start(&htim9,TIM_CHANNEL_2);
    HAL_TIM_PWM_Start(&htim1,TIM_CHANNEL_1);
    HAL_TIM_PWM_Start(&htim1,TIM_CHANNEL_2);
    HAL_TIM_PWM_Start(&htim1,TIM_CHANNEL_3);
    HAL_TIM_PWM_Start(&htim1,TIM_CHANNEL_4);
		HAL_TIM_PWM_Start(&htim8,TIM_CHANNEL_3);
    HAL_TIM_PWM_Start(&htim8,TIM_CHANNEL_4);
    /********************************* 使能调度时钟 *********************************/
    HAL_TIM_Base_Start_IT(&htim6);
		


}

void Task_Loop(void)
{
	__HAL_TIM_SET_COMPARE(&htim8,TIM_CHANNEL_3,GetCCRFromAngle(yaw,270.f));
	__HAL_TIM_SET_COMPARE(&htim8,TIM_CHANNEL_4,GetCCRFromAngle(pitch,180.f));

}