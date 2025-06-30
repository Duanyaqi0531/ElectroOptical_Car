#include "ita_car.h"

void Class_EoCar::Init()
{
    //底盘初始化
    Chassis.Init();
		Minipc.Init(&huart4);
		OLED.Init();
		Yaw_Slope.Init(0.09,0.09);
		Pitch_Slope.Init(0.003,0.003);
		Yaw_Slope.Set_Now(135);
		Pitch_Slope.Set_Now(90);
  //  Gimbal.Init();
}

void Class_EoCar::TIM_Calculate_PeriodElapsedCallback()
{
    Chassis.TIM_Calculate_PeriodElapsedCallback();
  //  Gimbal.TIM_Calculate_PeriodElapsedCallback();
}