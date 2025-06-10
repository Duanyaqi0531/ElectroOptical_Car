#include "crt_gimbal.h"

void Class_Gimbal::Init()
{
    //lk初始化
    LK_Motor.Init(&hcan1,LK_Motor_ID_0x142,0.0f);
    LK_Motor.PID_Angle.Init(10.0f,1.0f,0.0f,0.0f,20.0f,100.0f);
    LK_Motor.PID_Omega.Init(5.0f,1.0f,0.0f,0.0f,50.0f,200.0f);
}
void Class_Gimbal::Output()
{   
    LK_Motor.Set_LK_Motor_Control_ID(LK_Motor_Control_Torque);

    switch (Gimbal_Control_Type)
    {
    case Gimbal_Control_Type_Disable:
    {
        LK_Motor.Set_LK_Motor_Control_Method(LK_Motor_Control_Method_TORQUE);
        LK_Motor.Set_Target_Torque(0.0f);
    }
    break;
    case Gimbal_Control_Type_Enable:
    {
        LK_Motor.Set_LK_Motor_Control_Method(LK_Motor_Control_Method_ANGLE);
        LK_Motor.Set_Target_Angle(Yaw_Lk_Angle);
    }
    break;
    }
}
void Class_Gimbal::TIM_Calculate_PeriodElapsedCallback()
{
    #ifdef __LK_Motor_Control_OneRound_Position_PIDControler
    LK_Motor.Set_LK_Motor_Control_ID(LK_Motor_Control_OneRound_Position_PIDControler);
    LK_Motor.Set_Target_Angle(Yaw_Lk_Angle);
    LK_Motor.Output();
    #endif
    Output();
    LK_Motor.TIM_Process_PeriodElapsedCallback();
}