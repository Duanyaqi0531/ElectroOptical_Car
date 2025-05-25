#include "dvc_encodermotor.h"

void Class_EncoderMotor::Init(TIM_HandleTypeDef *__TIM_EncoderHandle,uint16_t __encoder_resolution, uint32_t __max_encoder_sum,float __GearRation,float __WheelDiameter)
{
    //定时器编码器外设初始化
    HAL_TIM_Encoder_Start(__TIM_EncoderHandle,TIM_CHANNEL_1);
    HAL_TIM_Encoder_Start(__TIM_EncoderHandle,TIM_CHANNEL_2);

    TIM_EncoderHandle = __TIM_EncoderHandle;
    Max_Encoder_Sum = __max_encoder_sum;
    Encoder_Resolution = __encoder_resolution;
    GearRation = __GearRation;
    WheelDiameter = __WheelDiameter;

    Now_Encoder = 0;
    //上次脉冲值
    Last_Encoder = 0;
    //脉冲差值
    Encoder_Diff = 0;
    //脉冲累加和 ±
    Encoder_Diff_Sum = 0;
    //当前圈数
    Total_Round = 0;    
    //当前角度
    Now_Angle = 0;
}

void Class_EncoderMotor::TIM_Encoder_Calculate()
{
    //获取当前脉冲值
    Now_Encoder = TIM_EncoderHandle->Instance->CNT;
    //计算脉冲差值
    Encoder_Diff = (int32_t)(Now_Encoder - Last_Encoder);
    //更新上次编码器值
    Last_Encoder = Now_Encoder;

    if(Encoder_Diff > Max_Encoder_Sum/2)
    {
        Encoder_Diff = Encoder_Diff - Max_Encoder_Sum;
    }
    else if(Encoder_Diff < -(int32_t)Max_Encoder_Sum/2)
    {
        Encoder_Diff = Encoder_Diff + Max_Encoder_Sum;
    }

    //模拟单圈编码器 0-Encoder_Resolution 计算圈数
    Encoder_Diff_Sum += Encoder_Diff;
    if(Encoder_Diff_Sum > (int32_t)Encoder_Resolution * 4)
    {
        Encoder_Diff_Sum -= (int32_t)Encoder_Resolution * 4;
        Total_Round++;
    }
    else if(Encoder_Diff_Sum < 0)
    {
        Encoder_Diff_Sum += (int32_t)Encoder_Resolution * 4;
        Total_Round--;
    }
    //计算角度
    Now_Angle = (float)(Encoder_Diff_Sum+Total_Round*Encoder_Resolution*4)/((int32_t)Encoder_Resolution * 4) * 360.0f / GearRation;
    //M法测速
    Now_Omega_Angle = (Now_Angle - Last_Angle) / 0.001f;
    Now_Velocity = Now_Omega_Angle * DEG_TO_RAD * WheelDiameter / 2.0f;

    Last_Angle = Now_Angle;
    //Total_Angle = (float)(Encoder_Diff_Sum + Total_Round*(int32_t)Encoder_Resolution)/(int32_t)Encoder_Resolution*360.0f;
}
void Class_EncoderMotor::OutPut_PWM()
{
    // 电机A
}
void Class_EncoderMotor::TIM_Calculate_PeriodElapsedCallback()
{
    //计算轮实际速度
    TIM_Encoder_Calculate();
    //PID
    PID_Omage.Set_Target(Target_Velocity);
    PID_Omage.Set_Now(Now_Velocity);
    PID_Omage.TIM_Adjust_PeriodElapsedCallback();
    PWM_Out = PID_Omage.Get_Out();
    //OutPut_PWM();
}
