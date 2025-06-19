#ifndef ITA_CAR_H
#define ITA_CAR_H
/* USER CODE BEGIN Includes */
#include "drv_can.h"
#include "drv_uart.h"
#include "crt_chassis.h"
#include "crt_gimbal.h"
#include "dvc_minipc.h"

#include "drv_adc.h"
#include "dvc_oled.h"

#include "alg_slope.h"


#include "drv_math.h"
/* USER CODE BEGIN  */
class Class_EoCar;
// extern Class_EoCar EoCar;

class Class_EoCar
{
public:
		Class_Minipc Minipc;
		Class_Slope Yaw_Slope;
		Class_Slope Pitch_Slope;
		Class_OLED OLED;
    Class_Tricycle_Chassis Chassis;
   // Class_Gimbal Gimbal;
    void Init();
    void TIM_Calculate_PeriodElapsedCallback();
protected:
    void TIM_Unline_Protect_PeriodElapsedCallback();
    void TIM1msMod50_Alive_PeriodElapsedCallback();
};

#endif 
