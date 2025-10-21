// Copyright (c) 2022 ChenJun
// Licensed under the MIT License.

#ifndef RM_SERIAL_DRIVER__PACKET_HPP_
#define RM_SERIAL_DRIVER__PACKET_HPP_

#include <algorithm>
#include <cstdint>
#include <vector>

// 从上位机接收的数据包结构（VisionToBoard）
// 用于接收来自视觉上位机的控制指令
struct __attribute__((packed)) ReceivePacket {
	uint8_t head[2] = {'E', 'G'};        // 数据包头，用于标识数据包开始
	uint8_t control;                     // 控制指令（如自瞄开关、小陀螺模式等）
	uint8_t shoot;                       // 射击指令（0-不射击，1-射击）
	float offset_yaw;                  // 目标yaw角度（单位：0.01度）
	float offset_pitch;                // 目标pitch角度（单位：0.01度）
	float horizon_distance;            // 目标水平距离（单位：cm）
	uint8_t id;                        //packet_id，自增，用来标记小电脑发来的数据
	uint16_t crc16;                      // CRC16校验值，用于验证数据完整性
};

// 发送给上位机的数据包结构（BoardToVision）
// 用于向视觉上位机发送云台状态信息
struct __attribute__((packed)) SendPacket {
	uint8_t head[2] = {'E', 'G'};        // 数据包头，用于标识数据包开始
	// uint8_t data_type;                   // 数据类型标识（0-四元数，1-状态信息）
	struct __attribute__((packed)) {
		int16_t qw, qx, qy, qz;      // 四元数数据（w,x,y,z分量），用于表示云台姿态
	} quaternion_data;
	struct __attribute__((packed)) {
		int16_t bullet_speed;        // 子弹速度（单位：0.01m/s）
		uint8_t mode;                // 当前模式（如自瞄、小陀螺等）
		uint8_t shoot_mode;          // 射击模式（如单发、连发等）
		int16_t ft_angle;            // 云台pitch角度（单位：0.01度）
	} status_data;
	uint16_t crc16;                      // CRC16校验值，用于验证数据完整性
};

inline ReceivePacket fromVector(const uint8_t* data)
{
	ReceivePacket packet;
	std::copy(data, data + sizeof(ReceivePacket), reinterpret_cast<uint8_t*>(&packet));
	return packet;
}

inline uint8_t* toVector(const SendPacket& data)
{
	uint8_t packet[sizeof(SendPacket)];
	std::copy(
			reinterpret_cast<const uint8_t*>(&data),
			reinterpret_cast<const uint8_t*>(&data) + sizeof(SendPacket), packet);
	return packet;
}

#endif  // RM_SERIAL_DRIVER__PACKET_HPP_