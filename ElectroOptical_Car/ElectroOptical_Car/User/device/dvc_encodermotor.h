#ifndef DVC_ENCODERMOTOR_H
#define DVC_ENCODERMOTOR_H

#include "drv_tim.h"
#include "drv_math.h"
#include "alg_pid.h"
class Class_EncoderMotor
{
public:

    Class_PID PID_Omage;
    
    void Init(TIM_HandleTypeDef *__TIM_EncoderHandle,uint16_t __encoder_resolution = 4096, uint32_t __max_encoder_sum = 65535 , float __GearRation = 20.25f,float __WheelDiameter = 0.065f); 
    void TIM_Encoder_Calculate();
    void TIM_Calculate_PeriodElapsedCallback();
    void inline Set_EncoderMotor_Target_Velocity(float __Target_Velocity);
    int16_t inline Get_EncoderMotor_PWMOutput();
protected:
  //编码器外设句柄
    TIM_HandleTypeDef * TIM_EncoderHandle;
    //编码器最大值 65535
    uint16_t Max_Encoder_Sum;
    //编码器分辨率 一圈多少个脉冲
    uint32_t Encoder_Resolution = 4096;
    //齿轮比例
    float GearRation;
    //电机直径
    float WheelDiameter;
    //当前脉冲值
    int32_t Now_Encoder;
    //上次脉冲值
    int32_t Last_Encoder;
		//脉冲累加和 ±
    int32_t Encoder_Diff_Sum;
    //脉冲差值
    int32_t Encoder_Diff;
    //当前圈数
    int16_t Total_Round;
    //当前角度
    float Now_Angle;
    float Last_Angle;
    //当前角速度
    float Now_Omega_Angle;//度每秒
    float Now_Velocity;//m/s
    
    float Target_Velocity;
    //输出PWM值
    int16_t PWM_Out;
    void OutPut_PWM();
};
void Class_EncoderMotor::Set_EncoderMotor_Target_Velocity(float __Target_Velocity)
{
  Target_Velocity = __Target_Velocity;
}
int16_t Class_EncoderMotor::Get_EncoderMotor_PWMOutput()
{
  return PWM_Out;
}
#endif // !DVC_ENCODERMOTOR_H
