//
// Created by mjw on 2022/9/23.
//

#include "remotec.h"
#include "usart.h"
#include "debugc.h"
#include "stdlib.h"
#include "cmath"
#include "cstring"
#include "cstdio"
#include "INS_task.h"
#include "gimbalc.h"
#include "normal_pid.h"
#include "shootc.h"
#include "drv_can.h"
#include "servo.h"
#include "packet.hpp"
#include "crc.h"

remotec MyRemote;
//receive data, 21bytes one frame, but set 36 bytes
//接收原始数据，为21个字节，给了36个字节长度，防止DMA传输越界
static uint8_t vt13_rx_buf[2][VT13_RX_BUF_NUM];

/**
  * @brief          remote control init
  * @param[in]      none
  * @retval         none
  */
/**
  * @brief          遥控器初始化
  * @param[in]      none
  * @retval         nonehh
  */
void REMOTEC_Init(void)
{
	REMOTEIO_init(vt13_rx_buf[0], vt13_rx_buf[1], VT13_RX_BUF_NUM);
}

void REMOTEC_UartIrqHandler(void)
{
	if (huart6.Instance->SR & UART_FLAG_RXNE)//接收到数据
	{
		__HAL_UART_CLEAR_PEFLAG(&huart6);
	}
	else if (USART6->SR & UART_FLAG_IDLE)
	{
		static uint16_t this_time_rx_len = 0;

		__HAL_UART_CLEAR_PEFLAG(&huart6);

		if ((hdma_usart6_rx.Instance->CR & DMA_SxCR_CT) == RESET)
		{
			/* Current memory buffer used is Memory 0 */

			//disable DMA
			//失效DMA
			__HAL_DMA_DISABLE(&hdma_usart6_rx);

			//get receive data length, length = set_data_length - remain_length
			//获取接收数据长度,长度 = 设定长度 - 剩余长度
			this_time_rx_len = VT13_RX_BUF_NUM - hdma_usart6_rx.Instance->NDTR;

			//reset set_data_lenght
			//重新设定数据长度
			hdma_usart6_rx.Instance->NDTR = VT13_RX_BUF_NUM;

			//set memory buffer 1
			//设定缓冲区1
			hdma_usart6_rx.Instance->CR |= DMA_SxCR_CT;

			//enable DMA
			//使能DMA
			__HAL_DMA_ENABLE(&hdma_usart6_rx);
			if (this_time_rx_len == RC_FRAME_LENGTH)
			{
				MyRemote.vt13_to_rc(vt13_rx_buf[0]);
			}
		}
		else
		{
			/* Current memory buffer used is Memory 1 */
			//disable DMA
			//失效DMA
			__HAL_DMA_DISABLE(&hdma_usart6_rx);

			//get receive data length, length = set_data_length - remain_length
			//获取接收数据长度,长度 = 设定长度 - 剩余长度
			this_time_rx_len = VT13_RX_BUF_NUM - hdma_usart6_rx.Instance->NDTR;

			//reset set_data_lenght
			//重新设定数据长度
			hdma_usart6_rx.Instance->NDTR = VT13_RX_BUF_NUM;

			//set memory buffer 0
			//设定缓冲区0
			hdma_usart6_rx.Instance->CR &= ~(DMA_SxCR_CT);

			//enable DMA
			//使能DMA
			__HAL_DMA_ENABLE(&hdma_usart6_rx);

			if (this_time_rx_len == RC_FRAME_LENGTH)
			{
				//处理遥控器数据
				MyRemote.vt13_to_rc(vt13_rx_buf[1]);
			}
		}
		MyRemote.RC_DataHandle();
	}
}


/**
  * @brief          remote control protocol resolution
  * @param[in]      sbus_buf: raw data point
  * @param[out]     rc_ctrl: remote control data struct point
  * @retval         none
  */
/**
  * @brief          遥控器协议解析
  * @param[in]      vt13_buf: 原生数据指针
  * @param[out]     rc_ctrl: 遥控器数据指针
  * @retval         none
  */
void remotec::vt13_to_rc( const uint8_t* VT13_buf)
{
	 // 检查输入指针是否为空
    if (VT13_buf == NULL ||  &rc_ctrl == NULL)
    {
        return;
    }

    // 检查VT13_buf的前两个字节是否为特定值，并验证CRC16校验和
	if(VT13_buf[0] == 0xa9 && VT13_buf[1] == 0x53 && Verify_CRC16_Check_Sum(VT13_buf, 21))
	{
	// 从VT13_buf中解析通道0的值
		rc_ctrl.rc.ch[0] = (VT13_buf[2] | (VT13_buf[3] << 8)) & 0x07ff;        //!< Channel 0
	// 从VT13_buf中解析通道1的值
		rc_ctrl.rc.ch[1] = ((VT13_buf[3] >> 3) | (VT13_buf[4] << 5)) & 0x07ff; //!< Channel 1
	// 从VT13_buf中解析通道2的值
		rc_ctrl.rc.ch[2] = ((VT13_buf[4] >> 6) | (VT13_buf[5] << 2) |          //!< Channel 2
													(VT13_buf[6] << 10)) & 0x07ff;
	// 从VT13_buf中解析通道3的值
		rc_ctrl.rc.ch[3] = ((VT13_buf[6] >> 1) | (VT13_buf[7] << 7)) & 0x07ff; //!< Channel 3
	// 从VT13_buf中解析模式开关的值
		rc_ctrl.rc.mode_sw = ((VT13_buf[7] >> 4) & 0x0003);
	// 从VT13_buf中解析停止按钮的值
		rc_ctrl.rc.stop = ((VT13_buf[7] >> 6) & 0x01);
	// 从VT13_buf中解析左按钮的值
		rc_ctrl.rc.left_button = ((VT13_buf[7] >> 7) & 0x01);//fn
	// 从VT13_buf中解析右按钮的值
		rc_ctrl.rc.right_button = ((VT13_buf[8] >> 0) & 0x01);
	// 从VT13_buf中解析滚轮的值
		rc_ctrl.rc.wheel = ((VT13_buf[8] >> 1) | (VT13_buf[9] << 7)) & 0x07FF;
	// 从VT13_buf中解析（扳机）的值
		rc_ctrl.rc.shutter = (VT13_buf[9] >> 4) & 0x01;//扳机

	// 从VT13_buf中解析鼠标X轴的值
		rc_ctrl.mouse.x = (VT13_buf[10] | (VT13_buf[11] << 8));
	// 从VT13_buf中解析鼠标Y轴的值
		rc_ctrl.mouse.y = (VT13_buf[12] | (VT13_buf[13] << 8));
	// 从VT13_buf中解析鼠标Z轴的值
		rc_ctrl.mouse.z = (VT13_buf[14] | (VT13_buf[15] << 8));

	// 从VT13_buf中解析鼠标左键的状态
		rc_ctrl.mouse.press_l.Now_State = (VT13_buf[16] >> 0) & 0x03;
	// 从VT13_buf中解析鼠标右键的状态
		rc_ctrl.mouse.press_r.Now_State = (VT13_buf[16] >> 2) & 0x03;
	// 从VT13_buf中解析鼠标中键的状态
		rc_ctrl.mouse.middle.Now_State = (VT13_buf[16] >> 4) & 0x03;

	// 从VT13_buf中解析键值
		rc_ctrl.key.value = (VT13_buf[17] | (VT13_buf[18] << 8));
		rc_ctrl.key.W = (rc_ctrl.key.value & 0x01);
		rc_ctrl.key.S = (rc_ctrl.key.value & 0x02) >> 1;
		rc_ctrl.key.A = (rc_ctrl.key.value & 0x04) >> 2;
		rc_ctrl.key.D = (rc_ctrl.key.value & 0x08) >> 3;
		rc_ctrl.key.SHIFT.Now_State = (rc_ctrl.key.value & 0x10) >> 4;
		rc_ctrl.key.CONTRL.Now_State = (rc_ctrl.key.value & 0x20) >> 5;
		rc_ctrl.key.Q.Now_State = (rc_ctrl.key.value & 0x40) >> 6;
		rc_ctrl.key.E.Now_State = (rc_ctrl.key.value & 0x80) >> 7;
		rc_ctrl.key.R.Now_State = (rc_ctrl.key.value & 0x100) >> 8;
		rc_ctrl.key.F.Now_State = (rc_ctrl.key.value & 0x200) >> 9;
		rc_ctrl.key.G.Now_State = (rc_ctrl.key.value & 0x400) >> 10;
		rc_ctrl.key.Z.Now_State = (rc_ctrl.key.value & 0x800) >> 11;
		rc_ctrl.key.X.Now_State = (rc_ctrl.key.value & 0x1000) >> 12;
		rc_ctrl.key.C.Now_State = (rc_ctrl.key.value & 0x2000) >> 13;
		rc_ctrl.key.V.Now_State = (rc_ctrl.key.value & 0x4000) >> 14;
		rc_ctrl.key.B.Now_State = (rc_ctrl.key.value & 0x8000) >> 15;
	// 从VT13_buf中解析CRC16校验和
		rc_ctrl.crc16 = (VT13_buf[19] | (VT13_buf[20] << 8));

	// 对解析出的通道值和滚轮值进行偏移调整
		rc_ctrl.rc.ch[0] -= RC_CH_VALUE_OFFSET;
		rc_ctrl.rc.ch[1] -= RC_CH_VALUE_OFFSET;
		rc_ctrl.rc.ch[2] -= RC_CH_VALUE_OFFSET;
		rc_ctrl.rc.ch[3] -= RC_CH_VALUE_OFFSET;
		rc_ctrl.rc.wheel -= RC_CH_VALUE_OFFSET;

		RC_GetNewData = 0;
	}
}

void remotec::RC_DataHandle()
{
	if (abs(rc_ctrl.rc.ch[0]) < 5)rc_ctrl.rc.ch[0] = 0;
	if (abs(rc_ctrl.rc.ch[1]) < 5)rc_ctrl.rc.ch[1] = 0;
	if (abs(rc_ctrl.rc.ch[2]) < 5)rc_ctrl.rc.ch[2] = 0;
	if (abs(rc_ctrl.rc.ch[3]) < 5)rc_ctrl.rc.ch[3] = 0;
	if (abs(rc_ctrl.rc.ch[0]) > 670 || abs(rc_ctrl.rc.ch[3]) > 670)
	{
		memset(&rc_ctrl, 0, sizeof(RC_ctrl_t)); //异常值
	}
}

/*
	* @name   portHandle
	* @brief  非连续键值处理 即上一次是0，本次是1，判断为按了一次，用于处理状态切换等不适用于连续用手按的按键
	* @param  port 键值状态结构体
  	* @retval None
*/
void remotec::portHandle(Key_State* port)
{
	if (port->Now_State == 1 && port->Last_State == 0) port->Is_Click_Once = 1;
	else port->Is_Click_Once = 0;
	port->Last_State = port->Now_State;
}

void remotec::portSetYawSpeed(void)
{
	switch (ControlMode)
	{
	case KEY_MODE:
		if (abs(rc_ctrl.mouse.x) < 50)
			yaw_speed = -rc_ctrl.mouse.x * 0.5; //这儿后期加等级分档位
		else
			yaw_speed = -rc_ctrl.mouse.x * 2.0; //这儿后期加等级分档位
		break;
	case RC_MODE:
		yaw_speed = -rc_ctrl.rc.ch[0] * 360 / 660.0f;
		break;
	}
//	return yaw_speed;
}

void remotec::portSetPihSpeed(void)
{
	switch (ControlMode)
	{
	case KEY_MODE:
		if (abs(rc_ctrl.mouse.y) < 50)
			pih_speed = rc_ctrl.mouse.y * 0.5;
		else
			pih_speed = rc_ctrl.mouse.y * 1.5;
		break;
	case RC_MODE:
		pih_speed = rc_ctrl.rc.ch[1] * 360 / 660.0f;
		break;
	}
}

void remotec::portSetVx(void)
{
	switch (ControlMode)
	{
	case KEY_MODE:
		vx = (rc_ctrl.key.D - rc_ctrl.key.A) * 80.0; //这儿后期加等级分档位
		if (rc_ctrl.key.SHIFT.Now_State == 1)
			vx *= 2.4;
		break;
	case RC_MODE:
		vx = rc_ctrl.rc.ch[3] * 200.0f / 660.0f;
		break;
	}
}

void remotec::portSetVy(void)
{
	switch (ControlMode)
	{
	case KEY_MODE:
		vy = (rc_ctrl.key.W - rc_ctrl.key.S) * 80.0;
		if (rc_ctrl.key.SHIFT.Now_State == 1)
			vy *= 2.4;
		break;
	case RC_MODE:
		vy = rc_ctrl.rc.ch[2] * 200.0f / 660.0f;
		break;
	}
}

void remotec::portSetCarMode(void)
{
	portHandle(&rc_ctrl.key.V);
	portHandle(&rc_ctrl.key.F);
	switch (ControlMode)
	{
	case KEY_MODE:
		if (rc_ctrl.key.V.Is_Click_Once && Last_MotionMode == SUIDONG)
		{
			MotionMode = SPIN;
		}
		else if (rc_ctrl.key.V.Is_Click_Once && Last_MotionMode == SPIN)
		{
			MotionMode = SUIDONG;
		}
		else MotionMode = Last_MotionMode;

		break;

	case RC_MODE:
		switch (rc_ctrl.rc.mode_sw)
		{
		case 0:
			MotionMode = SPIN;
			break;
		case 1:
			MotionMode = SUIDONG;
			break;
		case 2:
			MotionMode = STOP;
			break;
		}
		break;
	}
	Last_MotionMode = MotionMode;
}

void remotec::portSetProtect(void)
{
	portHandle(&rc_ctrl.key.G);
	portHandle(&rc_ctrl.key.F);
	switch (ControlMode)
	{
	case KEY_MODE:
		if (rc_ctrl.key.G.Is_Click_Once && Last_FricMode != OPENFRIC)
		{
			FricMode = OPENFRIC;
		}
		else if (rc_ctrl.key.G.Is_Click_Once && Last_FricMode != CLOSEFRIC)
		{
			FricMode = CLOSEFRIC;
		}
		else FricMode = Last_FricMode;

		break;
	case RC_MODE:
		if(rc_ctrl.rc.shutter)
			FricMode = OPENFRIC;
		else
			FricMode = CLOSEFRIC;
		break;
	}
	Last_FricMode = FricMode;
}

uint8_t remotec::portIsZimiao(void)
{
	switch (ControlMode)
	{
		case RC_MODE:
		{
			if (rc_ctrl.rc.right_button == 1)
				return 1;
			else
				return 0;
		}
		case KEY_MODE:
		{
			if (rc_ctrl.mouse.press_r.Now_State)
				return 0x01;  //带数字的自瞄
			else
				return 0;
		}
	}
}

uint8_t remotec::portIsRedrawing(void)
{
	portHandle(&rc_ctrl.key.B);
	return rc_ctrl.key.B.Now_State;
}

void remotec::portSetHead(void)
{
	int16_t Target = Get_ChassisTarget();
	portHandle(&rc_ctrl.key.X);
	portHandle(&rc_ctrl.key.Q);
	portHandle(&rc_ctrl.key.E);
	if (rc_ctrl.key.Q.Is_Click_Once)
	{
		Set_ChassisTarget(Target + 45);
	}

	if (rc_ctrl.key.E.Is_Click_Once)
	{
		Set_ChassisTarget(Target - 45);
	}

	if (rc_ctrl.key.X.Is_Click_Once)
	{
		Set_YawTarget(180);
		Set_ChassisTarget(Target + 180);
	}
}

void remotec::Swich_ControlMode(void)
{
	switch (rc_ctrl.rc.mode_sw)
	{
	case TOKEY:
		//Control_Mode = KEY_MODE;
		break;
	default:
		ControlMode = RC_MODE;
		break;
	}
}

void remotec::update()
{
	// usart_printf("%d\r\n",RC_GetNewData);
	RC_GetNewData++;
	if (RC_GetNewData > 50)
	{
		is_online = 0;
	}
	else is_online = 1;

	if (RC_GetNewData == 1000) {
		RC_GetNewData = 51;
	}

	Swich_ControlMode();
	portSetProtect();
	portSetCarMode();
	portSetVx();
	portSetVy();
	portSetPihSpeed();
	portSetYawSpeed();
	portSetHead();
}