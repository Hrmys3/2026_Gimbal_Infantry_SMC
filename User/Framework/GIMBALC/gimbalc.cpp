//
// Created by mjw on 2022/9/19.
//
#include "gimbalc.h"
#include "drv_can.h"
#include "normal_pid.h"
#include "debugc.h"
#include "remotec.h"
#include "INS_task.h"
#include <cmath>
#include "shootc.h"
#include "kalman.h"
#include "iwdgc.h"
#include "packet.hpp"
#include "servo.h"
#include "slidingmodec.h"
#include <complex.h>

gimbalc omni; //全向
extern ReceivePacket vision_packet;

gimbalc::gimbalc()
{
	//matlab pid
	PID_initialize();

	YawBias = ChassisYawTarget;

	Pid_In.PihP_MO = 300;
	Pid_In.PihS_MO = 20192;
	Pid_In.YawP_MO = 300;
	Pid_In.YawS_MO = 30192;
	Pid_In.YawP_LO = -Pid_In.YawP_MO;
	Pid_In.YawS_LO = -Pid_In.YawS_MO;
	Pid_In.PihP_LO = -Pid_In.PihP_MO;
	Pid_In.PihS_LO = -Pid_In.PihS_MO;

	pos_pid[2].OutStep = 5000;
	pos_pid[2].OutMax = 300;
	pos_pid[2].ErrAllMax = 30;

	speed_pid[2].OutMax = 30192;
	speed_pid[2].ErrAllMax = 7192;

}

//操作模式参数选择：键盘模式、遥控器模式、自瞄模式采用不同参数
void gimbalc::ParamChoose(int8_t mode)
{
	switch (mode)
	{
		case KEY_MODE: //键盘模式
		{
			YawSMC.C = 25;
			YawSMC.K = 140;

			Pid_In.PihP_P = 1.8;
			Pid_In.PihP_I = 0;
			Pid_In.PihP_D = 0.15;
			Pid_In.PihP_N = 75;
			Pid_In.Pih_Dif_Gain = 0.12; //前馈一阶差分增益

			Pid_In.PihS_P = 1000;
			Pid_In.PihS_I = 1250;
			Pid_In.PihS_D = 0.001;
			Pid_In.PihS_N = 20;

			break;
		}

		case RC_MODE: //遥控器模式
		{
			YawSMC.C = 25; //23
			YawSMC.K = 170; //160

			Pid_In.PihP_P = 2.5;
			Pid_In.PihP_I = 0;
			Pid_In.PihP_D = 0.1;
			Pid_In.PihP_N = 75;
			Pid_In.Pih_Dif_Gain = 0.12; //前馈一阶差分增益

			Pid_In.PihS_P = 1000;
			Pid_In.PihS_I = 1000;
			Pid_In.PihS_D = 0.001;
			Pid_In.PihS_N = 20;

			break;
		}

		case AUTOAIM_MODE: //自瞄模式
		{
			YawSMC.C = 12;
			YawSMC.K = 120;

			Pid_In.PihP_P = 1.0;
			Pid_In.PihP_I = 0;
			Pid_In.PihP_D = 0.1;
			Pid_In.PihP_N = 75;
			Pid_In.Pih_Dif_Gain = 0.01; //前馈一阶差分增益

			Pid_In.PihS_P = 800;
			Pid_In.PihS_I = 500;
			Pid_In.PihS_D = 0.000;
			Pid_In.PihS_N = 20;

			break;
		}
	}
}

void gimbalc::AlgorithmCompute()
{

	motors[0].update_angle();
	motors[1].update_angle();

	//普通pid
	ChassisYawPid.Update(motors[0].Angle_Ecd);
	pos_pid[0].Update(motors[0].Motor_Angle);
	pos_pid[1].Update(motors[1].Motor_Angle);

	ChassisYawPid.GetOutput();
	pos_pid[0].GetOutput();
	pos_pid[1].GetOutput();

	speed_pid[0].Target = pos_pid[0].Out;
	speed_pid[1].Target = pos_pid[1].Out;

	speed_pid[0].Update(motors[0].Motor_Speed);
	speed_pid[1].Update(motors[1].Motor_Speed);

	speed_pid[0].GetOutput();
	speed_pid[1].GetOutput();

	//滑模控制
	YawSMC.SMC_Tick(motors[0].Motor_Angle, motors[0].Motor_Speed * 5.99f);

	//Matlab的PID
	Pid_In.YawAngle_Now = motors[0].Motor_Angle;
	Pid_In.YawSpeed_Now = motors[0].Motor_Speed;
	Pid_In.PihAngle_Now = motors[1].Motor_Angle;
	Pid_In.PihSpeed_Now = motors[1].Motor_Speed;
	PID_step(1);

	//底盘随动前馈
	ChassisYawPid.Out = ChassisYawPid.Out - Chassis_DifGain * (pos_pid[0].Target - pos_pid[0].lastTarget);
	pos_pid[0].lastTarget = pos_pid[0].Target;

}

void gimbalc::Pih_Limit() //pitch限位
{
	if ( motors[1].Feedback_Mode == ECD_MODE ) //编码器模式
	{
		if (PihTarget > Pih_EcdUpLimit)
		{
			PihTarget = Pih_EcdUpLimit;
		}
		if (PihTarget  < Pih_EcdLowLimit)
		{
			PihTarget  = Pih_EcdLowLimit;
		}
	}
	else //陀螺仪模式
	{
		if (PihTarget > Pih_GyrUpLimit)
		{
			PihTarget = Pih_GyrUpLimit;
		}
		if (PihTarget  < Pih_GyrLowLimit)
		{
			PihTarget  = Pih_GyrLowLimit;
		}
	}
}

void gimbalc::Yaw_EcdClean(void) //无问题
{
	motors[0].MotorAngle_ALL = 0;
	motors[0].NowAngle = 0;
	motors[0].Angle_Ecd = motors[0].Angle * 360.0f / (8192.0f);
	ChassisYawPid.Err_all = 0;
} //ecd重置

void gimbalc::Protect_Mode()
{
	if(!motors[0].is_online || !motors[1].is_online || IS_IMU_OK == 0 || !MyRemote.is_online || SportMode == STOP)
	{
		Protect_flag = OFFLINE;
	}
	else
	{
		Protect_flag = ONLINE;
	}

	if (Protect_flag == ONLINE)
	{
		if(Last_Warning != TOKEY && warning == TOKEY)
		{
			fric_ram_status = OPENRAMMER;
			warning = OPENFRIC;
			MyRemote.Last_ProtectMode = OPENFRIC;
		}

		switch (warning)
		{
		case CLOSEFRIC:
		{
			fric_ram_status = CLOSERAMMER;
			shoot.speed_pids[0].Out = 0;
			shoot.FricSpeedClean();
			break;
		}

		case OPENFRIC:
		{
			if (Last_Warning != OPENFRIC || Last_ProtectFlag == OFFLINE) //不能在断电的时候由连发切单发，且还是有一定幅度的反转 大概10°
			{
				//rammer电机重置
				shoot.speed_pids[0].Err_all = 0;
				shoot.ram_pos_pid.Target = 0;
				shoot.ram_pos_pid.Err_all = 0;
				shoot.rammer_flag = 0;
				shoot.SetRammer();//通电时默认连发
				shoot.motors[0].clear();
			}
			shoot.FricSpeedReset();
			fric_ram_status = OPENRAMMER;
			break;
		}
		}
	}
	else if (Protect_flag == OFFLINE)
	{
		Pid_Out.YawCurrent = 0;
		Pid_Out.PihCurrent = 0;
		speed_pid[0].Target = 0;
		speed_pid[1].Target = 0;
		YawSMC.u = 0;

		shoot.FricSpeedClean();
		shoot.RammerSpeedClean();
		shoot.speed_pids[0].Out = 0;

		TargetInit();
		can.YawSendCurrent(0);
		can.PitchSendCurrent(0);

		fric_ram_status = CLOSERAMMER;
		SportMode = SUIDONG;
		Last_SportMode = SUIDONG;
		ChassisYawPid.Err_all = 0;
	}
	Last_Warning = warning;
	Last_ProtectFlag = Protect_flag;
}

void gimbalc::ChassisComLoop()
{
	is_online = 0xff;
	if (Protect_flag == OFFLINE)
	{
		MyRemote.vx = 0;
		MyRemote.vy = 0;
		vz = 0;
		is_online = 0;
	}

	if (MyRemote.rc_ctrl.key.R.Now_State)fric_ram_status  = CRAZYRAMMER;
	switch (CAN2_Status) //0 1 错开发送，避免挤占can通道
	{
	case 0:
		can.ChassisSendCmd(MyRemote.vx*2,MyRemote.vy*2, vz*0.8, 0,is_online ); //错开发送
		CAN2_Status = 1;
		break;
	case 1:
		can.ChassisSendGimbalStatus(-(motors[0].Angle_Ecd - YawBias), 0, Servo_GetStatus(), shoot.GetFricStatus() & shoot.motors[1].is_online & shoot.motors[2].is_online, fric_ram_status & shoot.permit, MyRemote.portIsRedrawing());//累计误差消除  portIsToSentry()机间通信没用
		CAN2_Status = 0;
		break;
	}
}

void gimbalc::SetWithRC(void)
{
	MyRemote.update();

	SportMode =  MyRemote.SportMode;

	warning = MyRemote.ProtectMode;
	AutoAim = MyRemote.portIsZimiao(); //暫時不改

	vz = -ChassisYawPid.Out;

	if (Protect_flag == ONLINE) //非保护模式
	{
		if (AutoAim == 0) //非自瞄模式，使用键鼠或者遥控器控制
		{
			YawTarget += MyRemote.yaw_speed * 5 / 1000;
			PihTarget += MyRemote.pih_speed * 5 / 1000;
			vision_packet.shoot = 0;
		}
		else //开启自瞄模式
		{
			if (vision_packet.control == 1)
			{
				//在当前姿态值上加减
				lost_timer = 0;

				if (vision_packet.id != last_id){ //上位机发来的ID更新
					// //更正上位机1发来的偏移量
					// if(vision_packet.offset_yaw>284) vision_packet.offset_yaw -= 360;
					// if(vision_packet.offset_yaw<-284) vision_packet.offset_yaw += 360;

					//上位机给出角度偏移过大时，视为发生错误，不执行旋转
					if (abs(motors[1].Motor_Angle - vision_packet.auto_pitch_target ) > 20 || abs(motors[0].Motor_Angle - vision_packet.auto_yaw_target) > 50)
					{

					}

					else //上位机给出数据正确
					{
						//目标角度设置为 当前角度+上位机给的偏移量
						YawTarget = vision_packet.auto_yaw_target;
						PihTarget = vision_packet.auto_pitch_target;
					}
					last_id = vision_packet.id ;
				}

				else { //id未更新，不执行旋转

				}

			}
			else //按了自瞄，但上位机没有看到目标，控制指令control=0
			{

				lost_timer++;
			}
			if (lost_timer >= 7)
			{
				vision_packet.shoot = 0;
				lost_timer = 0;
			}
		}

		//YawTarget限幅，避免疯转
		if (YawTarget - motors[0].Motor_Angle > 80) YawTarget = motors[0].Motor_Angle + 80;
		if (YawTarget - motors[0].Motor_Angle < -80) YawTarget = motors[0].Motor_Angle - 80;
	}

	//不同操作模式的参数选择
	if (AutoAim == 0)
	{
		if(MyRemote.Control_Mode == KEY_MODE) ParamChoose(KEY_MODE);
		else ParamChoose(RC_MODE);
	}
	else ParamChoose(AUTOAIM_MODE);

	//自瞄同时小陀螺时，调整控制逻辑和参数
	// if(CarMode == TUOLUO && Zimiao == 1) YawMotorAllAngel.Algorithml = NOMEL;
	if(SportMode == TUOLUO && AutoAim == 1)
	{
//		YawMotorAllAngel.Algorithml = SLIDE;
		YawSMC.J =0.74;
		YawSMC.C = 10;
		if (vision_packet.control == 0) { //上位机没有检测到装甲板

		}
		else {
			// YawTarget = vision_packet.auto_yaw_target + 2.0f;
			YawTarget = vision_packet.auto_yaw_target + omni.motors[0].Speed_Ecd * 0.025;
		}
	}
	else
	{
//		YawMotorAllAngel.Algorithml = SLIDE;
		YawSMC.J =0.75;
	}

	//切换不同的运动模式
	switch (SportMode)
	{
	default://随动
	{
		if (Last_SportMode != SUIDONG)
		{
			Yaw_EcdClean();
		}
		if (ChassisYawTarget - motors[0].Angle_Ecd > 180)ChassisYawTarget -= 360; //加减2π
		if (motors[0].Angle_Ecd - ChassisYawTarget > 180)ChassisYawTarget += 360;//优弧劣弧处理
		ChassisYawPid.Target = ChassisYawTarget; //YAW_Bias
		break;
	}
	case TUOLUO: //小陀螺模式
	{
		Yaw_EcdClean();
		vz = -50.0f; //恒速小陀螺
		break;
	}
	}

	Pih_Limit();
	{
		Pid_In.YawAngle_set = YawTarget;
		Pid_In.PihAngle_set = PihTarget;

		pos_pid[0].Target = YawTarget;
		pos_pid[1].Target = PihTarget;

		YawSMC.target = YawTarget;
	}
	Last_SportMode = SportMode;
	if (MyRemote.rc_ctrl.key.SHIFT.Now_State && vz != -ChassisYawPid.Out) vz = vz * 2.5	; //是否需要分段？ 还是按键定模式
}

void gimbalc::TargetInit(void)
{
	YawTarget = motors[0].Motor_Angle;
	PihTarget = motors[1].Motor_Angle; //PIH好像有点搜索不到
}

void gimbalc::CurrentCompute()
{
	//选择yaw的算法
	switch (motors[0].Algorithml)
	{
	case MATLAB_PID:
		can.YawSendCurrent(motors[0].pole * Pid_Out.YawCurrent);
		break;
	case SLIDE: //一般使用该模式
		can.YawSendCurrent(motors[0].pole * YawSMC.u);
		break;
	case NORMAL:
		can.YawSendCurrent(motors[0].pole * speed_pid[0].Out);
		break;
	}

	//选择pitch的算法
	switch (motors[1].Algorithml)
	{
	case MATLAB_PID: //一般使用该模式
		can.PitchSendCurrent(motors[1].pole * Pid_Out.PihCurrent);
		break;
	case NORMAL:
		can.PitchSendCurrent(motors[1].pole * speed_pid[1].Out);
		break;
	}

	can.ShootSendCurrent(shoot.speed_pids[1].Out,shoot.speed_pids[2].Out,shoot.speed_pids[0].Out,0);
}

//matlab生成控制器 先用一个大循环，后期移植到task
void gimbalc::ControlLoop()
{
	FeedDog(); //喂狗 √
	Printf_Test(); //√
	AlgorithmCompute();
	SetWithRC();
	Protect_Mode();
	ChassisComLoop();
	CurrentCompute();
}

void gimbalc::Printf_Test(void)
{
	//if (MyRemote.portIsZimiao() == 1) usart_printf("111\r\n");
	//usart_printf("%.2f, %.2f, %.2f\r\n", vision_packet.offset_yaw, YawTarget, motors[0].Motor_Angle);
	// usart_printf("%.2f\r\n",omni.motors[0].Speed_Ecd);
	//usart_printf("%.2f\r\n",motors[1].Angle_Ecd);
	//usart_printf("%d \r\n", MyRemote.rc_ctrl.rc.mode_sw);
	//usart_printf("%d\r\n",MyRemote.rc_ctrl.rc.wheel);
	// usart_printf("%d\r\n",MyRemote.rc_ctrl.rc.ch[0]);
	// usart_printf("%f\r\n",Pid_Out.PihCurrent);
	//usart_printf("%.2f %.2f %.2f %.2f\r\n",motors[0].Angle_Ecd, motors[0].Angle_Imu, INS.Yaw, INS.YawTotalAngle);
	usart_printf("%f,%f,%f,%f,%d,%d\r\n",motors[0].Angle_Imu,motors[1].Angle_Imu,motors[0].Motor_Speed,motors[1].Motor_Speed,motors[0].is_online,motors[1].is_online);
} //放一些常用的打印

//以下是改变随动方向的函数接口
int16_t Get_ChassisTarget(void)
{
	return omni.ChassisYawTarget;
}

void Set_ChassisTarget(int16_t Target)
{
	omni.ChassisYawTarget = Target;
}

void Set_YawTarget(float Target)
{
	omni.YawTarget += Target;
}