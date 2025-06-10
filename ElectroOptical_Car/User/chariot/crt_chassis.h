#ifndef CRT_CHASSIS_H
#define CRT_CHASSIS_H

#include "dvc_encodermotor.h"

#define encoder_resolution (500)
#define HALF_WIDTH (0.15f)
#define HALF_LENGTH (0.15f)
#define GearRation (20.25f)
#define WheelDiameter (0.075f)
#define   k   sqrt(2)/2.0f
/**
 * @brief 底盘控制类型
 *
 */
enum Enum_Chassis_Control_Type :uint8_t
{
    Chassis_Control_Type_DISABLE = 0,
    Chassis_Control_Type_FLLOW,
    Chassis_Control_Type_SPIN_Positive,
    Chassis_Control_Type_SPIN_Negative,
};

//omnidirectional 全向轮
class Class_Tricycle_Chassis
{
public:
    void Init(float __Velocity_X_Max = 4.0f, float __Velocity_Y_Max = 4.0f, float __Omega_Max = 8.0f);

    void TIM_Calculate_PeriodElapsedCallback();
    inline void Set_Chassis_Control_Type(Enum_Chassis_Control_Type __Chassis_Control_Type);
    //目标速度X
    float Target_Velocity_X = 0.0f;
    //目标速度Y
    float Target_Velocity_Y = 0.0f;
    //目标角速度
    float Target_Omega = 0.0f;
   //当前速度X
    float Now_Velocity_X = 0.0f;
    //当前速度Y
    float Now_Velocity_Y = 0.0f;
    //当前角速度
    float Now_Omega = 0.0f;
		//目标航向角
		float Target_yaw;
		//x方向里程计
		float Length_X;
		//y方向里程计
		float Length_Y;
protected:
    //初始化相关常量
    Class_EncoderMotor E_Motor[4];

    //速度X限制
    float Velocity_X_Max;
    //速度Y限制
    float Velocity_Y_Max;
    //角速度限制
    float Omega_Max;
    //底盘小陀螺模式角速度
    float Spin_Omega = 4.0f;
    //常量

    //内部变量
    //转动电机目标值
    float Target_Wheel_Omega[4];

    //读变量
    //写变量

    //读写变量

    //底盘控制方法
    Enum_Chassis_Control_Type Chassis_Control_Type = Chassis_Control_Type_FLLOW;

    //内部函数
    void Speed_Resolution();
};
void Class_Tricycle_Chassis::Set_Chassis_Control_Type(Enum_Chassis_Control_Type __Chassis_Control_Type)
{
    Chassis_Control_Type = __Chassis_Control_Type;
}

#endif
