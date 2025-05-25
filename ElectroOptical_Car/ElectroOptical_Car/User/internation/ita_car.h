#ifndef ITA_CAR_H
#define ITA_CAR_H
/* USER CODE BEGIN Includes */
#include "dvc_encodermotor.h"
#include "drv_can.h"
#include "drv_uart.h"
#include "MPU6050.h"
/* USER CODE BEGIN  */
class Class_EoCar;
// extern Class_EoCar EoCar;

class Class_EoCar
{
public:
    Class_EncoderMotor E_Motor[4];

    void Init();
    void TIM_Calculate_PeriodElapsedCallback();
protected:
    void TIM_Unline_Protect_PeriodElapsedCallback();
    void TIM1msMod50_Alive_PeriodElapsedCallback();
};

#endif 
