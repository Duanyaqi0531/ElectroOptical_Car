#include "dvc_minipc.h"


void Class_Minipc::Init(UART_HandleTypeDef *huart)
{
    //Minipc串口
    if (huart->Instance == USART1)
    {
        UART_Manage_Object = &UART1_Manage_Object;
    }
    else if (huart->Instance == USART2)
    {
        UART_Manage_Object = &UART2_Manage_Object;
    }
    else if (huart->Instance == USART3)
    {
        UART_Manage_Object = &UART3_Manage_Object;
    }
    else if (huart->Instance == UART4)
    {
        UART_Manage_Object = &UART4_Manage_Object;
    }
    else if (huart->Instance == UART5)
    {
        UART_Manage_Object = &UART5_Manage_Object;
    }
    else if (huart->Instance == USART6)
    {
        UART_Manage_Object = &UART6_Manage_Object;
    }
}
/**
 * @brief tim定时器中断定期检测迷你主机是否存活
 *
 */
void Class_Minipc::TIM1msMod50_Alive_PeriodElapsedCallback()
{
//    //判断该时间段内是否接收过迷你主机数据
//    if (Navigation_Flag == Pre_Navigation_Flag)
//    {
//        //迷你主机断开连接
//        Navigation_Status =  Navigation_Status_DISABLE;
//    }
//    else
//    {
//        //迷你主机保持连接
//        Navigation_Status =  Navigation_Status_ENABLE ;
//    }
//    Pre_Navigation_Flag = Navigation_Flag;
//		
		 //判断该时间段内是否接收过迷你主机数据
//    if (Aim_Flag == Pre_Aim_Flag)
//    {
//        //迷你主机断开连接
//        Selfaiming_Status =  Selfaiming_Status_DISABLE;
//    }
//    else
//    {
//        //迷你主机保持连接
//        Selfaiming_Status =  Selfaiming_Status_ENABLE ;
//    }
//    Pre_Aim_Flag = Aim_Flag;
//		
		 //判断该时间段内是否接收过迷你主机数据
    if (MiniPc_Flag == Pre_MiniPc_Flag)
    {
        //迷你主机断开连接
        MiniPc_Status =  MiniPc_Status_DISABLE;
    }
    else
    {
        //迷你主机保持连接
        MiniPc_Status =  MiniPc_Status_ENABLE ;
    }
    Pre_MiniPc_Flag = MiniPc_Flag;



}
void Class_Minipc::MiniPc_UART_RxCpltCallback(uint8_t *Rx_Data)
{
	
	MiniPc_Data_Process();



}
void Class_Minipc::MiniPc_Data_Process()
{

if(Verify_CRC16_Check_Sum(UART_Manage_Object->Rx_Buffer,UART_Manage_Object->Rx_Buffer_Length))
	{
			memcpy(&MiniPc_Data, UART_Manage_Object->Rx_Buffer,sizeof(Struct_MiniPc_Data));
    //数据处理过程
    Struct_MiniPc_Data *tmp_buffer = (Struct_MiniPc_Data *)UART_Manage_Object->Rx_Buffer;

    /*源数据转为对外数据*/
		MiniPc_Data.linear_x=tmp_buffer->linear_x;
		MiniPc_Data.linear_y=tmp_buffer->linear_y;
		MiniPc_Data.angular_yaw=tmp_buffer->angular_yaw;
		MiniPc_Data.pixel_dx=tmp_buffer->pixel_dx;
		MiniPc_Data.pixel_dy=tmp_buffer->pixel_dy;
		MiniPc_Data.flags=tmp_buffer->flags;
	
		MiniPc_Flag++;
		
	
	}

	
}

void Class_Minipc::UART_Tx_UI()
{
	memcpy(&UART_Manage_Object->Tx_Buffer[0], &Send_Data, sizeof(&Send_Data));
	UART_Send_Data(UART_Manage_Object->UART_Handler, UART_Manage_Object->Tx_Buffer, sizeof(Send_Data));

}
/**
  * @brief CRC16 Caculation function
  * @param[in] pchMessage : Data to Verify,
  * @param[in] dwLength : Stream length = Data + checksum
  * @param[in] wCRC : CRC16 init value(default : 0xFFFF)
  * @return : CRC16 checksum
  */
uint16_t Class_Minipc::Get_CRC16_Check_Sum(const uint8_t * pchMessage, uint32_t dwLength, uint16_t wCRC)
{
  uint8_t ch_data;

  if (pchMessage == NULL) return 0xFFFF;
  while (dwLength--) {
    ch_data = *pchMessage++;
    wCRC = (wCRC >> 8) ^ W_CRC_TABLE[(wCRC ^ ch_data) & 0x00ff];
  }

  return wCRC;
}

/**
  * @brief CRC16 Verify function
  * @param[in] pchMessage : Data to Verify,
  * @param[in] dwLength : Stream length = Data + checksum
  * @return : True or False (CRC Verify Result)
  */

bool Class_Minipc::Verify_CRC16_Check_Sum(const uint8_t * pchMessage, uint32_t dwLength)
{
  uint16_t w_expected = 0;

  if ((pchMessage == NULL) || (dwLength <= 2)) return false;

  w_expected = Get_CRC16_Check_Sum(pchMessage, dwLength - 2, CRC16_INIT);
  return (
    (w_expected & 0xff) == pchMessage[dwLength - 2] &&
    ((w_expected >> 8) & 0xff) == pchMessage[dwLength - 1]);
}

/**

@brief Append CRC16 value to the end of the buffer
@param[in] pchMessage : Data to Verify,
@param[in] dwLength : Stream length = Data + checksum
@return none
*/
void Class_Minipc::Append_CRC16_Check_Sum(uint8_t * pchMessage, uint32_t dwLength)
{
  uint16_t w_crc = 0;

  if ((pchMessage == NULL) || (dwLength <= 2)) return;

  w_crc = Get_CRC16_Check_Sum(pchMessage, dwLength - 2, CRC16_INIT);

  pchMessage[dwLength - 2] = (uint8_t)(w_crc & 0x00ff);
  pchMessage[dwLength - 1] = (uint8_t)((w_crc >> 8) & 0x00ff);
}

//void Class_Minipc::Navigation_Data_Process(uint8_t *Rx_Data)
//{
//		Navigation_Flag++;
//	
//		Navigation_Data.Velocity_X = ((int16_t)((Rx_Data[1] << 8) | Rx_Data[0]))/1000.f;
//    // 解包y方向线速度
//		Navigation_Data.Velocity_Y = ((int16_t)((Rx_Data[3] << 8) | Rx_Data[2]))/1000.f;
//		// 解包yaw旋转角速度
//		Navigation_Data.Velocity_Z = ((int16_t)((Rx_Data[5] << 8) | Rx_Data[4]))/1000.f;
//	
//}
//void Class_Minipc::Selfaiming_Data_Process(uint8_t *Rx_Data)
//{
//	Aim_Flag++;
//		Selfaiming_Data.Pixel_difference_X=(int16_t)((Rx_Data[1] << 8) | Rx_Data[0]);
//		Selfaiming_Data.Pixel_difference_Y=(int16_t)((Rx_Data[3] << 8) | Rx_Data[2]);
//		Selfaiming_Data.Lock_Flag=Rx_Data[4];
//		Selfaiming_Data.Win_Flag=Rx_Data[4];
//	
//}