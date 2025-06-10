#include "crt_chassis.h"

void Class_Tricycle_Chassis::Init(float __Velocity_X_Max, float __Velocity_Y_Max, float __Omega_Max)
{
    Velocity_X_Max = __Velocity_X_Max;
    Velocity_Y_Max = __Velocity_Y_Max;
    Omega_Max = __Omega_Max;
    //轮向电机初始化
    E_Motor[0].Init(&htim2,encoder_resolution,65535,GearRation,WheelDiameter);
    E_Motor[1].Init(&htim3,encoder_resolution,65535,GearRation,WheelDiameter);   
    E_Motor[2].Init(&htim4,encoder_resolution,65535,GearRation,WheelDiameter);
    E_Motor[3].Init(&htim5,encoder_resolution,65535,GearRation,WheelDiameter);
    //电机PID初始化
    E_Motor[0].PID_Omage.Init(400000.0f,50.0f,5.0f,0.0f,2000.0f,16799.0f);
    E_Motor[1].PID_Omage.Init(400000.0f,50.0f,5.0f,0.0f,2000.0f,16799.0f);
    E_Motor[2].PID_Omage.Init(400000.0f,50.0f,5.0f,0.0f,2000.0f,16799.0f);
    E_Motor[3].PID_Omage.Init(400000.0f,50.0f,5.0f,0.0f,2000.0f,16799.0f);
}   
void Class_Tricycle_Chassis::Speed_Resolution()
{
    switch (Chassis_Control_Type)
    {
        case (Chassis_Control_Type_DISABLE):
        {
            //底盘失能 四轮子自锁
            for (int i = 0; i < 4; i++)
            { 
                E_Motor[i].Set_EncoderMotor_PWMOutput(0);
            }            
        }
        break;
				case (Chassis_Control_Type_SPIN_Positive) :
        case (Chassis_Control_Type_SPIN_Negative):
        case (Chassis_Control_Type_FLLOW):
        {
            //底盘四电机模式配置
            //底盘限速
            if (Velocity_X_Max != 0)
            {
                Math_Constrain(&Target_Velocity_X, -Velocity_X_Max, Velocity_X_Max);
            }
            if (Velocity_Y_Max != 0)
            {
                Math_Constrain(&Target_Velocity_Y, -Velocity_Y_Max, Velocity_Y_Max);
            }
            if (Omega_Max != 0)
            {
                Math_Constrain(&Target_Omega, -Omega_Max, Omega_Max);
            }

            //速度换算，正运动学分解 
            float motor1_temp_linear_vel = k*Target_Velocity_Y - k*Target_Velocity_X - Target_Omega*(HALF_LENGTH);
            float motor2_temp_linear_vel = k*Target_Velocity_Y + k*Target_Velocity_X - Target_Omega*(HALF_LENGTH);
            float motor3_temp_linear_vel = -k*Target_Velocity_Y + k*Target_Velocity_X - Target_Omega*(HALF_LENGTH);
            float motor4_temp_linear_vel = -k*Target_Velocity_Y - k*Target_Velocity_X - Target_Omega*(HALF_LENGTH);         
            //角速度*减速比  设定目标 直接给到电机输出轴
            E_Motor[0].Set_EncoderMotor_Target_Velocity(motor1_temp_linear_vel);
            E_Motor[1].Set_EncoderMotor_Target_Velocity(motor2_temp_linear_vel);
            E_Motor[2].Set_EncoderMotor_Target_Velocity(motor3_temp_linear_vel);
            E_Motor[3].Set_EncoderMotor_Target_Velocity(motor4_temp_linear_vel);
						Now_Omega=-(E_Motor[0].Get_Now_Velocity()+E_Motor[1].Get_Now_Velocity()+E_Motor[2].Get_Now_Velocity()+E_Motor[3].Get_Now_Velocity())/(4*HALF_LENGTH);
						Now_Velocity_X=(-E_Motor[0].Get_Now_Velocity()+E_Motor[1].Get_Now_Velocity()+E_Motor[2].Get_Now_Velocity()-E_Motor[3].Get_Now_Velocity())/(4*k);
						Now_Velocity_Y=(E_Motor[0].Get_Now_Velocity()+E_Motor[1].Get_Now_Velocity()-E_Motor[2].Get_Now_Velocity()-E_Motor[3].Get_Now_Velocity())/(4*k);
				
            //各个电机具体PID
            for (int i = 0; i < 4; i++)
            {
                E_Motor[i].TIM_Calculate_PeriodElapsedCallback();
            }
        }
        break;
    }   
}
void Class_Tricycle_Chassis::TIM_Calculate_PeriodElapsedCallback()
{
    Speed_Resolution();
		if(((Target_Velocity_Y-Now_Velocity_Y)<0.2)&&((Now_Velocity_Y-Target_Velocity_Y)<0.2))
			 Length_Y+=Now_Velocity_Y*0.001f;
		else
			 Length_Y+=Target_Velocity_Y*0.001f;
		if(((Target_Velocity_X-Now_Velocity_X)<0.2)&&((Now_Velocity_X-Target_Velocity_X)<0.2))
			Length_X+=Now_Velocity_X*0.001f;
		else
				Length_X+=Target_Velocity_X*0.001f;
		
    //电机输出力矩纠正
    //偶数为左侧轮子 奇数为右侧轮子
    for (auto i = 0; i < 4; i++)
    {
        int16_t PWM_Out = E_Motor[i].Get_EncoderMotor_PWMOutput();

        if (i == 0)
        {
            if (PWM_Out < 0)
            {
                __HAL_TIM_SET_COMPARE(&htim10, TIM_CHANNEL_1, -PWM_Out);
                __HAL_TIM_SET_COMPARE(&htim11, TIM_CHANNEL_1, 0);
            }
            else
            {
                __HAL_TIM_SET_COMPARE(&htim10, TIM_CHANNEL_1, 0);
                __HAL_TIM_SET_COMPARE(&htim11, TIM_CHANNEL_1, PWM_Out);
            }
        }
        else if (i == 1)
        {
            if (PWM_Out < 0)
            {
                __HAL_TIM_SET_COMPARE(&htim9, TIM_CHANNEL_1, 0);
                __HAL_TIM_SET_COMPARE(&htim9, TIM_CHANNEL_2, -PWM_Out);
            }
            else
            {
                __HAL_TIM_SET_COMPARE(&htim9, TIM_CHANNEL_1, PWM_Out);
                __HAL_TIM_SET_COMPARE(&htim9, TIM_CHANNEL_2, 0);
            }
        }
        else if (i == 2)
        {
            if (PWM_Out < 0)
            {
                __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_2, -PWM_Out);
                __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, 0);
            }
            else
            {
                __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_2, 0);
                __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, PWM_Out);
            }
        }
        else if (i == 3)
        {
            if (PWM_Out < 0)
            {
                __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_4, 0);
                __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_3, -PWM_Out);
            }
            else
            {
                __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_4, PWM_Out);
                __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_3, 0);
            }
        }
    }
}