#include "drv_uart.h"
#include "string.h"
//enum Enum_Navigation_Status
//{
//		Navigation_Status_DISABLE = 0,
//		Navigation_Status_ENABLE,
//};
//enum Enum_Selfaiming_Status
//{
//		Selfaiming_Status_DISABLE = 0,
//		Selfaiming_Status_ENABLE,
//};
//struct Struct_Navigation_Data
//{ 
//    int16_t Velocity_X;
//    int16_t Velocity_Y;
//    int16_t Velocity_Z;
//    int16_t Reserved;
//} __attribute__((packed)); 


//struct Struct_Selfaiming_Data
//{ 
//    int16_t Pixel_difference_X;
//    int16_t Pixel_difference_Y;
//    uint8_t Lock_Flag;
//    uint8_t Win_Flag;
//		int16_t Reserved;
//} __attribute__((packed)); 
static const uint16_t MINPC_CRC16_INIT = 0xFFFF;
const uint16_t CRC16_INIT = 0xffff;
static const uint16_t W_CRC_TABLE[256] = 
{
  0x0000, 0x1189, 0x2312, 0x329b, 0x4624, 0x57ad, 0x6536, 0x74bf, 0x8c48, 0x9dc1, 0xaf5a, 0xbed3,
  0xca6c, 0xdbe5, 0xe97e, 0xf8f7, 0x1081, 0x0108, 0x3393, 0x221a, 0x56a5, 0x472c, 0x75b7, 0x643e,
  0x9cc9, 0x8d40, 0xbfdb, 0xae52, 0xdaed, 0xcb64, 0xf9ff, 0xe876, 0x2102, 0x308b, 0x0210, 0x1399,
  0x6726, 0x76af, 0x4434, 0x55bd, 0xad4a, 0xbcc3, 0x8e58, 0x9fd1, 0xeb6e, 0xfae7, 0xc87c, 0xd9f5,
  0x3183, 0x200a, 0x1291, 0x0318, 0x77a7, 0x662e, 0x54b5, 0x453c, 0xbdcb, 0xac42, 0x9ed9, 0x8f50,
  0xfbef, 0xea66, 0xd8fd, 0xc974, 0x4204, 0x538d, 0x6116, 0x709f, 0x0420, 0x15a9, 0x2732, 0x36bb,
  0xce4c, 0xdfc5, 0xed5e, 0xfcd7, 0x8868, 0x99e1, 0xab7a, 0xbaf3, 0x5285, 0x430c, 0x7197, 0x601e,
  0x14a1, 0x0528, 0x37b3, 0x263a, 0xdecd, 0xcf44, 0xfddf, 0xec56, 0x98e9, 0x8960, 0xbbfb, 0xaa72,
  0x6306, 0x728f, 0x4014, 0x519d, 0x2522, 0x34ab, 0x0630, 0x17b9, 0xef4e, 0xfec7, 0xcc5c, 0xddd5,
  0xa96a, 0xb8e3, 0x8a78, 0x9bf1, 0x7387, 0x620e, 0x5095, 0x411c, 0x35a3, 0x242a, 0x16b1, 0x0738,
  0xffcf, 0xee46, 0xdcdd, 0xcd54, 0xb9eb, 0xa862, 0x9af9, 0x8b70, 0x8408, 0x9581, 0xa71a, 0xb693,
  0xc22c, 0xd3a5, 0xe13e, 0xf0b7, 0x0840, 0x19c9, 0x2b52, 0x3adb, 0x4e64, 0x5fed, 0x6d76, 0x7cff,
  0x9489, 0x8500, 0xb79b, 0xa612, 0xd2ad, 0xc324, 0xf1bf, 0xe036, 0x18c1, 0x0948, 0x3bd3, 0x2a5a,
  0x5ee5, 0x4f6c, 0x7df7, 0x6c7e, 0xa50a, 0xb483, 0x8618, 0x9791, 0xe32e, 0xf2a7, 0xc03c, 0xd1b5,
  0x2942, 0x38cb, 0x0a50, 0x1bd9, 0x6f66, 0x7eef, 0x4c74, 0x5dfd, 0xb58b, 0xa402, 0x9699, 0x8710,
  0xf3af, 0xe226, 0xd0bd, 0xc134, 0x39c3, 0x284a, 0x1ad1, 0x0b58, 0x7fe7, 0x6e6e, 0x5cf5, 0x4d7c,
  0xc60c, 0xd785, 0xe51e, 0xf497, 0x8028, 0x91a1, 0xa33a, 0xb2b3, 0x4a44, 0x5bcd, 0x6956, 0x78df,
  0x0c60, 0x1de9, 0x2f72, 0x3efb, 0xd68d, 0xc704, 0xf59f, 0xe416, 0x90a9, 0x8120, 0xb3bb, 0xa232,
  0x5ac5, 0x4b4c, 0x79d7, 0x685e, 0x1ce1, 0x0d68, 0x3ff3, 0x2e7a, 0xe70e, 0xf687, 0xc41c, 0xd595,
  0xa12a, 0xb0a3, 0x8238, 0x93b1, 0x6b46, 0x7acf, 0x4854, 0x59dd, 0x2d62, 0x3ceb, 0x0e70, 0x1ff9,
  0xf78f, 0xe606, 0xd49d, 0xc514, 0xb1ab, 0xa022, 0x92b9, 0x8330, 0x7bc7, 0x6a4e, 0x58d5, 0x495c,
  0x3de3, 0x2c6a, 0x1ef1, 0x0f78
};

enum Enum_MiniPc_Status
{
		MiniPc_Status_DISABLE = 0,
		MiniPc_Status_ENABLE,
};

struct Struct_MiniPc_Data
{
    uint8_t header = 0xA5;
    float linear_x;
    float linear_y;
    float angular_yaw;
    int pixel_dx;
    int pixel_dy;
    uint8_t flags;           // 标志位
    uint16_t crc16 = 0xFFFF; // crc16校验
} __attribute__((packed));
struct Struct_Send_Data
{
    uint8_t header = 0x5A;
    
    uint16_t crc16 = 0xFFFF; // crc16校验
} __attribute__((packed));
class Class_Minipc 
{
	public:
		void Init(UART_HandleTypeDef *huart);
		inline float Get_Pixel_difference_X();
		inline float Get_Pixel_difference_Y();
		inline float Get_Velocity_X();
		inline float Get_Velocity_Y();
		inline float Get_Velocity_Z();
		inline uint8_t  Get_Lock_Flag();
		inline uint8_t  Get_Win_Flag();
		void TIM1msMod50_Alive_PeriodElapsedCallback();
		Struct_MiniPc_Data MiniPc_Data;
		Struct_MiniPc_Data Pre_MiniPc_Data;
		Struct_Send_Data Send_Data;
		//当前时刻的minipc接收flag
    uint32_t MiniPc_Flag = 0;
    //前一时刻的minipc接收flag
    uint32_t Pre_MiniPc_Flag = 0;
		Enum_MiniPc_Status MiniPc_Status;
		void MiniPc_Data_Process();
	  void MiniPc_UART_RxCpltCallback(uint8_t *Rx_Data);
		void UART_Tx_UI();
	  void Append_CRC16_Check_Sum(uint8_t * pchMessage, uint32_t dwLength);
    bool Verify_CRC16_Check_Sum(const uint8_t * pchMessage, uint32_t dwLength);
    uint16_t Get_CRC16_Check_Sum(const uint8_t * pchMessage, uint32_t dwLength, uint16_t wCRC);
	protected:
    //初始化相关常量

    //绑定的UART
    Struct_UART_Manage_Object *UART_Manage_Object;
	
//		void Navigation_Data_Process(uint8_t *Rx_Data);
//		void Selfaiming_Data_Process(uint8_t *Rx_Data);
//		Struct_Navigation_Data Navigation_Data;
//		Struct_Selfaiming_Data Selfaiming_Data;
//		Enum_Navigation_Status Navigation_Status;
//		Enum_Selfaiming_Status Selfaiming_Status;
//	 //当前时刻的导航接收flag
//    uint32_t Navigation_Flag = 0;
//    //前一时刻的导航接收flag
//    uint32_t Pre_Navigation_Flag = 0;
//    //当前时刻的自瞄接收flag
//    uint32_t Aim_Flag = 0;
//    //前一时刻的自瞄接收flag
//    uint32_t Pre_Aim_Flag = 0;

};
float Class_Minipc::Get_Pixel_difference_X()
{
return MiniPc_Data.pixel_dx;
}
float Class_Minipc::Get_Pixel_difference_Y()
{
return MiniPc_Data.pixel_dy;
}
uint8_t Class_Minipc::Get_Lock_Flag()
{
return MiniPc_Data.flags&0x01;
}
uint8_t Class_Minipc::Get_Win_Flag()
{
return MiniPc_Data.flags&0x02;
}

float Class_Minipc::Get_Velocity_X()
{
return MiniPc_Data.linear_x;
}
float Class_Minipc::Get_Velocity_Y()
{
return MiniPc_Data.linear_y;
}
float Class_Minipc::Get_Velocity_Z()
{
return MiniPc_Data.angular_yaw;
}
//float Class_Minipc::Get_Pixel_difference_X()
//{
//return Selfaiming_Data.Pixel_difference_X;
//}
//float Class_Minipc::Get_Pixel_difference_Y()
//{
//return Selfaiming_Data.Pixel_difference_Y;
//}
//uint8_t Class_Minipc::Get_Lock_Flag()
//{
//return Selfaiming_Data.Lock_Flag;
//}
//uint8_t Class_Minipc::Get_Win_Flag()
//{
//return Selfaiming_Data.Win_Flag;
//}

//float Class_Minipc::Get_Velocity_X()
//{
//return Navigation_Data.Velocity_X;
//}
//float Class_Minipc::Get_Velocity_Y()
//{
//return Navigation_Data.Velocity_Y;
//}
//float Class_Minipc::Get_Velocity_Z()
//{
//return Navigation_Data.Velocity_Z;
//}