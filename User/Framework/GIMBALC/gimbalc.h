//
// Created by mjw on 2022/9/19.
//

#ifndef ROBO_TEST_GIMBALC_H
#define ROBO_TEST_GIMBALC_H

#ifdef __cplusplus
extern "C" {
#endif
#include "stm32f4xx_hal.h"
#include "PIDC.h"
#include "drv_can.h"
#include "slidingmodec.h"
#include "normal_pid.h"
////兵种选择
//#define MECANUM1 1 //老麦轮 √ 已退休
//#define MECANUM2 2 //新麦轮 √
//#define OMNI 3 //全向轮
//#define HERO 4 //英雄云台 √
//#define BALANCE 5//平衡步兵
//
//#define GIMBAL OMNI

//电机反馈选择
#define ECD_MODE 0 //ChassisYaw、fric和ram均采用编码器模式
#define GYR_MODE 1 //yaw和pitch均采用陀螺仪模式

//选择是哪个角
#define YAW_ANGLE 1
#define PIH_ANGLE 2
#define RAM_ANGLE 3

//选择操作模式
#define KEY_MODE 1
#define RC_MODE 2
#define AUTOAIM_MODE 3

//算法选择
#define NORMAL 1 //普通PID
#define MATLAB 2 //MATLAB PID
#define SLIDE 3 //滑模控制

//模式赋值
// #define ZIYOU  1
#define STOP 1 //急停
#define SUIDONG  3
#define TUOLUO  2
#define SPIN 4
#define CAR_PROTECT 5
#define RC_OFFLINE 6

//保护模式赋值
#define PROTECT 4
#define TOKEY 2
#define OPENFRIC 3
#define CLOSEFRIC 1
#define OFFLINE 0
#define ONLINE 1

//摩擦轮状态
#define CLOSERAMMER 0
#define OPENRAMMER 1
#define CRAZYRAMMER 2

//舵机状态
#define SERVO_OFF 0
#define SERVO_ON 1

//是否允许发弹
#define FORBID 0
#define PERMIT 1

//实际摩擦轮状态
#define FRIC_OFF 0
#define FRIC_ON 1

extern ExtU rtU;
extern ExtY rtY;

#define Pid_In rtU
#define Pid_Out rtY //matlab生成的PID

int16_t Get_ChassisTarget(void);
void Set_ChassisTarget(int16_t Target);
void Set_YawTarget(float Target);

class gimbalc
{
public:
	float Chassis_DifGain = 15, ChassisYawTarget = 104.0f, YawBias, vz;
	float Pih_EcdUpLimit = 170, Pih_EcdLowLimit = 120, Pih_GyrUpLimit = 24, Pih_GyrLowLimit = -24;
	float YawTarget,PihTarget;
	int8_t fric_ram_status, last_id,AutoAim;

	void Printf_Test(void);
	void ControlLoop(void);

	Motor motors[2]
	{
		{GYR_MODE,SLIDE,&hcan2,1.0,8192.0f,1,CAN_YAW_RCV_ID},
		{GYR_MODE,MATLAB,&hcan1,1.0,8192.0f,-1,CAN_PIH_RCV_ID},
	};
//	NormalPID
	NormalPID pos_pid[2]
	{
		{1,1,1,1},
		{1,1,1,1}
	};
	NormalPID speed_pid[2]
	{
		{1,1,1,1},
		{1,1,1,1}
	};
	NormalPID ChassisYawPid{0.6,0.0,0.2,1000};
// 	SMC
	SMC YawSMC{20, 150, 0, 0.01, 21, 27, 16384, 0.7, 1};
	gimbalc();
private:
	bool CAN2_Status;
	int8_t warning,Last_Warning,SportMode,Last_SportMode,is_online,Protect_flag,Last_ProtectFlag;
	int8_t last_ID,lost_timer;

	void Protect_Mode();
	void ParamChoose(int8_t mode);
	void ChassisComLoop();
	void AlgorithmCompute();
	void SetWithRC(void);
	void TargetInit(void);
	void CurrentCompute();
	void Pih_Limit();
	void Yaw_EcdClean(void);
};
extern gimbalc omni;
#ifdef __cplusplus
}
#endif
#endif //ROBO_TEST_GIMBALC_H
