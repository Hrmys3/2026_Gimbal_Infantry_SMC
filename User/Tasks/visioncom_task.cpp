//
// Updated by 2b-superman on 2025/10/04.
//

#include "packet.hpp"
#include "crc.h"
#include "visioncom_task.h"
#include "FreeRTOS.h"
#include "task.h"
#include "QuaternionEKF.h"
#include "cmsis_os.h"
#include "remotec.h"
#include "INS_task.h"
#include "drv_can.h"
#include "debugc.h"
#include "main.h"
#include "usart.h"  // 串口头文件，用于串口通信
#include "usartio.h"
#include "usbd_cdc_if.h"
#include "usbd_cdc.h"

// 声明接收数据包结构体变量
extern ReceivePacket vision_packet;

// 定义发送数据包结构体变量，用于存储要发送给上位机的数据
SendPacket send_packet;
uint8_t Buf[sizeof(SendPacket)];
uint8_t New_Buf[sizeof(Buf)];

/**
  * @brief 视觉通信循环函数
  * @param mode: 当前工作模式，决定发送数据类型
  * @return None
  * @note 此函数负责填充发送数据包并通过串口发送给上位机
  */
void VisionChattingLoop(uint8_t mode)
{
    // 设置数据包头，用于标识数据包开始
    send_packet.head[0] = 'E';
    send_packet.head[1] = 'G';

    // 发送四元数数据（用于姿态传输）
    // send_packet.data_type = 0;  // 数据类型标识：0表示四元数数据

    // 从IMU获取四元数并转换为int16类型（放大1000倍以保留小数精度）
    send_packet.quaternion_data.qw = (int16_t)(QEKF_INS.q[0] * 1000);
    send_packet.quaternion_data.qx = (int16_t)(QEKF_INS.q[1] * 1000);
    send_packet.quaternion_data.qy = (int16_t)(QEKF_INS.q[2] * 1000);
    send_packet.quaternion_data.qz = (int16_t)(QEKF_INS.q[3] * 1000);

    // 发送状态数据（用于状态传输）
    // send_packet.data_type = 1;  // 数据类型标识：1表示状态数据

    // 填充状态数据（子弹速度放大100倍以保留小数精度）
    send_packet.status_data.bullet_speed = (int16_t)(judge.shoot_spd_now * 100);
    //if (send_packet.status_data.bullet_speed == 0) send_packet.status_data.bullet_speed = (int16_t)3000; //通信测试用
    // 当前工作模式：是否自瞄
    send_packet.status_data.mode = MyRemote.portIsZimiao();
    // 射击模式（如单发、连发等）
    send_packet.status_data.shoot_mode = 1;
    // 云台pitch角度（放大100倍以保留小数精度）
    send_packet.status_data.ft_angle = (int16_t)(IMU_NaiveAngle().pitch * 100);

    // 计算CRC校验值并填充到数据包末尾（排除CRC字段本身）
    Append_CRC16_Check_Sum((uint8_t*)&send_packet, sizeof(SendPacket));

    //测试数据包末尾是否是crc校验值
    // uint16_t calculated_crc = 0;
    // uint8_t* packet_end = (uint8_t*)&send_packet + sizeof(SendPacket) - 2;
    // calculated_crc = *(uint16_t*)packet_end;
    // usart_printf("%02x\r\n", calculated_crc);

    std::copy(reinterpret_cast<const uint8_t*>(&send_packet),
       reinterpret_cast<const uint8_t*>(&send_packet) + sizeof (SendPacket),Buf);

    // 将数据包复制到发送缓冲区
    memcpy(New_Buf, Buf, sizeof(SendPacket));

    //print_packet_debug(New_Buf, sizeof(New_Buf)); //调试用打印

    //通过USB向上位机发送数据
    CDC_Transmit_FS(New_Buf, sizeof(Buf));
}

/**
  * @brief 视觉通信任务函数
  * @param argument: FreeRTOS任务参数
  * @return None
  * @note 此函数是FreeRTOS任务，负责周期性地处理视觉通信
  */
void VisionComTask(void const* argument)
{
    /* USER CODE BEGIN VisionComTask */
    portTickType CurrentTime;

    //static uint32_t print_time_debug = 0;   //调试用

    /* Infinite loop */
    for (;;)
    {
        CurrentTime = xTaskGetTickCount();
        VisionChattingLoop(MyRemote.portIsZimiao());

        //调试用打印
        // if (CurrentTime - print_time_debug > 100) // 100ms间隔
        // {
        //     print_time_debug = CurrentTime;
        //
        //     print_packet_debug((uint8_t*)&vision_packet, sizeof(vision_packet));
        // }

        vTaskDelayUntil(&CurrentTime, 10 / portTICK_RATE_MS);
    }
    /* USER CODE END VisionComTask */
}

//新增数据包打印函数（调试用）
/*
void print_packet_debug(const uint8_t* packet_data, size_t packet_size) {
    char debug_msg[100];
    char hex_buf[4];

    //打印数据包大小
    sprintf(debug_msg, "Size: %d\r\n", packet_size);
    HAL_UART_Transmit(&huart1, (uint8_t*)debug_msg, strlen(debug_msg), 100);

    //打印数据包内容（十六进制格式）
    for (size_t i = 0; i < packet_size; i++) {
        sprintf(hex_buf, "%02X ", packet_data[i]);
        HAL_UART_Transmit(&huart1, (uint8_t*)hex_buf, 3, 100);
    }
    HAL_UART_Transmit(&huart1, (uint8_t*)"\r\n", 4, 100);
}
*/
/*
//使用方法：
#include "visioncom_task.h"

print_packet_debug((uint8_t*)&packet, sizeof(packet));
*/