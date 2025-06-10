#include "ita_car.h"

void Class_EoCar::Init()
{
    //底盘初始化
    Chassis.Init();
		Minipc.Init(&huart4);
  //  Gimbal.Init();
}

void Class_EoCar::TIM_Calculate_PeriodElapsedCallback()
{
    Chassis.TIM_Calculate_PeriodElapsedCallback();
  //  Gimbal.TIM_Calculate_PeriodElapsedCallback();
}