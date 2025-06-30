#include "task.h"
#include "ita_car.h"  

Class_EoCar EoCar;
uint8_t mod50;
uint8_t mod10;
uint8_t unlock_count;
uint16_t mod_2s;
uint8_t lock_flag;
float yaw=135;
float pitch=90;
float yaw_last=135;
float pitch_last=90;
float yaw_slope=135;
float pitch_slope=90;
float k_p_pitch=0.00015;


float last_picth_error;
float k_p_yaw=-0.0006;
float k_d_yaw=0.2;
float last_yaw_error;
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
uint8_t ByteRecv=0x5a;
uint8_t start_flag=0;
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart){
if (huart == &huart2){
	
HAL_UART_Receive_IT(&huart2, &ByteRecv, 1);//接收了一次后需要再次打开接收中断
}
}

void Chassis_Device_CAN1_Callback(Struct_CAN_Rx_Buffer *CAN_RxMessage)
{
		uint32_t temp_id;
		if(CAN_RxMessage->Header.IDE == CAN_ID_STD)
	{
		temp_id = CAN_RxMessage->Header.StdId;
	}
	
}

void Task_UART4_Callback(uint8_t *Buffer, uint16_t Length)
{
	 EoCar.Minipc.MiniPc_UART_RxCpltCallback(Buffer);
	 EoCar.Chassis.Target_Velocity_X= EoCar.Minipc.Get_Velocity_X();
	 EoCar.Chassis.Target_Velocity_Y= EoCar.Minipc.Get_Velocity_Y();
	 EoCar.Chassis.Target_Omega=EoCar.Minipc.Get_Velocity_Z();
	
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
			yaw_slope+=0.2;
		if(yaw_slope>=270)yaw_flag=0;
	}
	else
	{
		yaw_slope-=0.2;
	if(yaw_slope<=0)yaw_flag=1;
	}
	
	if(picth_flag)
	{
			pitch_slope+=0.06;
		if(pitch_slope>=94)picth_flag=0;
	}
	else
	{
		pitch_slope-=0.06;
	if(pitch_slope<=78)picth_flag=1;
	}
		EoCar.Pitch_Slope.Set_Target(pitch_slope);
		
		EoCar.Yaw_Slope.Set_Target(yaw_slope);
		pitch=EoCar.Pitch_Slope.Get_Out();
		yaw=EoCar.Yaw_Slope.Get_Out();//pid
		if(EoCar.Pitch_Slope.Get_Out()<70)
		{
			EoCar.Pitch_Slope.Set_Decrease_Value(1);
			EoCar.Pitch_Slope.Set_Increase_Value(1);
		}
		else
		{
			EoCar.Pitch_Slope.Set_Decrease_Value(0.003);
			EoCar.Pitch_Slope.Set_Increase_Value(0.003);
		}
		}
	

void Task2ms_TIM6_Callback()
{	
	
	mod50++;
	mod10++;
	
	if(mod10==10)		{EoCar.Minipc.UART_Tx_UI();	
		EoCar.Minipc.Send_Data.encoder_speed_LF=EoCar.Chassis.E_Motor[1].Get_Now_Velocity();
		EoCar.Minipc.Send_Data.encoder_speed_LR=EoCar.Chassis.E_Motor[0].Get_Now_Velocity();
		EoCar.Minipc.Send_Data.encoder_speed_RF=-EoCar.Chassis.E_Motor[2].Get_Now_Velocity();
		EoCar.Minipc.Send_Data.encoder_speed_RR=-EoCar.Chassis.E_Motor[3].Get_Now_Velocity();}
	if(mod50==50)
	{
		
	

		EoCar.Minipc.TIM1msMod50_Alive_PeriodElapsedCallback();
	
	}
	if(start_flag==0)
	{
		mod_2s++;
		ByteRecv=0x5a;
		
	}
	if(mod_2s>=2000)
	start_flag=1;
	if(huart4.ErrorCode!=0)
	{
	HAL_UART_DMAStop(&huart4); // 停止以重启
  HAL_Delay(10); // 等待错误结束
  HAL_UARTEx_ReceiveToIdle_DMA(&huart4, UART4_Manage_Object.Rx_Buffer, UART4_Manage_Object.Rx_Buffer_Length);	
	}
//	if(EoCar.Minipc.Get_Minipc_status()==MiniPc_Status_DISABLE)
//	{
//		
//	EoCar.Chassis.Set_Chassis_Control_Type(Chassis_Control_Type_DISABLE);
//	}
//	else
//	{
//	EoCar.Chassis.Set_Chassis_Control_Type(Chassis_Control_Type_FLLOW);
//	}
//	ENABLE_SWTICH
//	if(EoCar.Minipc.Get_Flag_2()==2)//锁定标志位
//	{
	
		if(HAL_GPIO_ReadPin(ENABLE_SWTICH_GPIO_Port,ENABLE_SWTICH_Pin)==GPIO_PIN_SET)
	{
		EoCar.Chassis.Set_Chassis_Control_Type(Chassis_Control_Type_FLLOW);
	
	}
	else
	{
	EoCar.Chassis.Set_Chassis_Control_Type(Chassis_Control_Type_DISABLE);
	}
	
	
	
	
	
		if(EoCar.Minipc.Get_Flag_1()==1)
		{
//			if((EoCar.Minipc.Get_Pixel_difference_X()<100)&&(EoCar.Minipc.Get_Pixel_difference_X()>-100))
//			{
//	
//				k_p_yaw=-0.005;
//				EoCar.Yaw_Slope.Set_Decrease_Value(0.005);
//					EoCar.Yaw_Slope.Set_Increase_Value(0.05);
//			}
//			else
//			{
//			
//					k_p_yaw=-0.0010;
//					EoCar.Yaw_Slope.Set_Decrease_Value(0.1);
//					EoCar.Yaw_Slope.Set_Increase_Value(0.1);
//			}
		pitch_slope+=EoCar.Minipc.Get_Pixel_difference_Y()*k_p_pitch;
		
		yaw_slope+=EoCar.Minipc.Get_Pixel_difference_X()*k_p_yaw+k_d_yaw*(last_yaw_error-EoCar.Minipc.Get_Pixel_difference_Y());
		Math_Constrain(&yaw_slope,0.f,270.f);
		Math_Constrain(&pitch_slope,70.f,94.f);
		EoCar.Pitch_Slope.Set_Target(pitch_slope);
		EoCar.Yaw_Slope.Set_Target(yaw_slope);
		pitch=EoCar.Pitch_Slope.Get_Out();
		yaw=EoCar.Yaw_Slope.Get_Out();
		last_yaw_error=EoCar.Minipc.Get_Pixel_difference_Y();
			
		}//pid
		else
		{
			if((EoCar.Minipc.Get_Flag_3()==4)&&(EoCar.Minipc.Get_Flag_1()==0)&&(EoCar.Minipc.Get_Flag_2()==0))//不动
			{
			pitch=pitch=EoCar.Pitch_Slope.Get_Out();
			yaw=EoCar.Yaw_Slope.Get_Out();
			}
			else// if((EoCar.Minipc.Get_Flag_3()==0)&&(EoCar.Minipc.Get_Flag_1()==0)&&(EoCar.Minipc.Get_Flag_2()==0))//
			{	
				Unlock_task();
				unlock_count++;
				yaw_slope=yaw;
				pitch_slope=pitch;
			}
		}
		
//	}
//	else
//	{
//	//Unlock_task();
//			yaw_slope=yaw;
//		pitch_slope=pitch;
//	
//	}
		EoCar.Yaw_Slope.TIM_Calculate_PeriodElapsedCallback();
		EoCar.Pitch_Slope.TIM_Calculate_PeriodElapsedCallback();
		
	if(EoCar.Minipc.Get_Flag_1()==8)
	{
	HAL_GPIO_WritePin(BEEP_GPIO_Port, BEEP_Pin, GPIO_PIN_SET);
		HAL_GPIO_WritePin(GPIOB, GPIO_PIN_15, GPIO_PIN_SET);
	}
	else
	{
	HAL_GPIO_WritePin(BEEP_GPIO_Port, BEEP_Pin, GPIO_PIN_RESET);
		HAL_GPIO_WritePin(GPIOB, GPIO_PIN_15, GPIO_PIN_RESET);
	}
	
//	if(ByteRecv==0x5A)
//	{
//	EoCar.Chassis.Target_Velocity_X=0;
//	EoCar.Chassis.Target_Velocity_Y=0;
//		EoCar.Chassis.Target_Omega=0;
//	}
//	else if(ByteRecv==0x41)///qianjin
//	{
//	EoCar.Chassis.Target_Velocity_X=0;
//	EoCar.Chassis.Target_Velocity_Y=1;
//		EoCar.Chassis.Target_Omega=0;
//	}
//	else if(ByteRecv==0x45)//houtui
//	{
//	EoCar.Chassis.Target_Velocity_X=0;
//	EoCar.Chassis.Target_Velocity_Y=-1;
//		EoCar.Chassis.Target_Omega=0;
//	}
//	else if(ByteRecv==0x47)//zipzhuan
//	{
//	EoCar.Chassis.Target_Velocity_X=-1;
//	EoCar.Chassis.Target_Velocity_Y=0;
//		EoCar.Chassis.Target_Omega=0;
//	}
//	else if(ByteRecv==0x44)
//	{
//	EoCar.Chassis.Target_Velocity_X=0;
//	EoCar.Chassis.Target_Velocity_Y=0;
//		EoCar.Chassis.Target_Omega=-3.14;
//	}
//	else if(ByteRecv==0x42)
//	{
//	EoCar.Chassis.Target_Velocity_X=0;
//	EoCar.Chassis.Target_Velocity_Y=0;
//	EoCar.Chassis.Target_Omega=3.14;
//	}
//	else if(ByteRecv==0x43)
//	{
//	EoCar.Chassis.Target_Velocity_X=1;
//	EoCar.Chassis.Target_Velocity_Y=0;
//	}else 
//	{
//	EoCar.Chassis.Target_Velocity_X=0;
//	EoCar.Chassis.Target_Velocity_Y=0;
//		EoCar.Chassis.Target_Omega=0;
//	}

		if(mod50==50)mod50=0;
		if(mod10==10)mod10=0;
		EoCar.TIM_Calculate_PeriodElapsedCallback();
		EoCar.OLED.TIM_Process_PeriodElapsedCallback();
		if(EoCar.Minipc.MiniPc_Status==MiniPc_Status_ENABLE)
		EoCar.OLED.OLED_ShowString(10,0,(const uint8_t*)"minipcok");
		else
		EoCar.OLED.OLED_ShowString(10,0,(const uint8_t*)"wait");

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
    TIM_Init(&htim6, Task2ms_TIM6_Callback);
    //串口初始化
     UART_Init(&huart4, Task_UART4_Callback, 24); 
		 UART_Init(&huart6, Task_UART6_Callback, 10); 
		 
			HAL_UART_Receive_IT(&huart2, &ByteRecv, 1);
			ADC_Init(&hadc2, 8);
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
		Math_Constrain(&yaw,0.f,270.f);
		Math_Constrain(&pitch,70.f,94.f);
//		EoCar.OLED.OLED_ShowNumber(1, 1, 1,1,1);
	__HAL_TIM_SET_COMPARE(&htim8,TIM_CHANNEL_3,GetCCRFromAngle(yaw,270.f));
	__HAL_TIM_SET_COMPARE(&htim8,TIM_CHANNEL_4,GetCCRFromAngle(pitch,180.f));

}