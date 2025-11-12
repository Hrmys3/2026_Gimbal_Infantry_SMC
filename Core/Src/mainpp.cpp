
#include "BSP.h"
//#include "usart_new.h"

/**
  * @brief  The application entry point.
  * @retval int
  */

extern CAN_HandleTypeDef hcan1;
extern CAN_HandleTypeDef hcan2;

void User_Init()
{
	//打印用串口使能
	DEBUGC_UartInit();
	TIM5_IT_Init();

	//遥控器串口使能
	REMOTEC_Init();

	can.All_Init();

	//IMU用
	delay_init();
	DWT_Init(168);

	Servo_StartPWM();

	usart_printf("Program start!\r\n");
	osKernelStart();
}

int main(void)
{
	BSP_Init();
	User_Init();
	/* USER CODE BEGIN WHILE */
	while (1)
	{
		/* USER CODE END WHILE */
		//matc();
		/* USER CODE BEGIN 3 */
	}
	/* USER CODE END 3 */
}