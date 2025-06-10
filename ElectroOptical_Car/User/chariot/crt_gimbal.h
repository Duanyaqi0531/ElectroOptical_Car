#ifndef CRT_GIMBAL
#define CRT_GIMBAL
#include "dvc_lkmotor.h"
enum Enum_Gimbal_Control_Type :uint8_t
{
    Gimbal_Control_Type_Disable = 0,
    Gimbal_Control_Type_Enable,
};
class Class_Gimbal
{
public:
    Class_LK_Motor LK_Motor;    
    
    void Init();
    void TIM_Calculate_PeriodElapsedCallback();
    inline void Set_Yaw_Lk_Angle(float __Yaw_Lk_Angle);
    inline void Set_Gimbal_Control_Type(Enum_Gimbal_Control_Type __Gimbal_Control_Type);
protected:
    Enum_Gimbal_Control_Type Gimbal_Control_Type = Gimbal_Control_Type_Disable; 
    float Yaw_Lk_Angle;
    void Output();
};

void Class_Gimbal::Set_Yaw_Lk_Angle(float __Yaw_Lk_Angle)
{
    Yaw_Lk_Angle = __Yaw_Lk_Angle;
}
void Class_Gimbal::Set_Gimbal_Control_Type(Enum_Gimbal_Control_Type __Gimbal_Control_Type)
{
    Gimbal_Control_Type = __Gimbal_Control_Type;
}
#endif // !CRT_GIMBAL