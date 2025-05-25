#include "ita_car.h"

#define encoder_resolution 500
void Class_EoCar::Init()
{
    //底盘初始化
    E_Motor[0].Init(&htim2,encoder_resolution,65535);
    E_Motor[1].Init(&htim3,encoder_resolution,65535);   
    E_Motor[2].Init(&htim4,encoder_resolution,65535);
    E_Motor[3].Init(&htim5,encoder_resolution,65535);

//    E_Motor[0].PID_Omage.Init(500000.0f,100.0f,5.0f,0.0f,2000.0f,20000.0f);
//    E_Motor[1].PID_Omage.Init(500000.0f,100.0f,5.0f,0.0f,2000.0f,20000.0f);
	
	  E_Motor[0].PID_Omage.Init(800000.0f,125.0f,0.0f,0.0f,2000.0f,20000.0f);
    E_Motor[1].PID_Omage.Init(800000.0f,125.0f,0.0f,0.0f,2000.0f,20000.0f);
		E_Motor[0].PID_Omage.Init(800000.0f,125.0f,0.0f,0.0f,2000.0f,20000.0f);
    E_Motor[1].PID_Omage.Init(800000.0f,125.0f,0.0f,0.0f,2000.0f,20000.0f);
}
int32_t  Count = 0;
void Class_EoCar::TIM_Calculate_PeriodElapsedCallback()
{
    // 底盘编码器更新
    for (auto i = 0; i < 4; i++)
    {
        E_Motor[i].TIM_Calculate_PeriodElapsedCallback();

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
                __HAL_TIM_SET_COMPARE(&htim9, TIM_CHANNEL_1, -PWM_Out);
                __HAL_TIM_SET_COMPARE(&htim9, TIM_CHANNEL_2, 0);
            }
            else
            {
                __HAL_TIM_SET_COMPARE(&htim9, TIM_CHANNEL_1, 0);
                __HAL_TIM_SET_COMPARE(&htim9, TIM_CHANNEL_2, PWM_Out);
            }
        }
        else if (i == 2)
        {
            if (PWM_Out < 0)
            {
                __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, -PWM_Out);
                __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_2, 0);
            }
            else
            {
                __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, 0);
                __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_2, PWM_Out);
            }
        }
        else if (i == 3)
        {
            if (PWM_Out < 0)
            {
                __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_3, -PWM_Out);
                __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_4, 0);
            }
            else
            {
                __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_3, 0);
                __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_4, PWM_Out);
            }
        }
    }
}