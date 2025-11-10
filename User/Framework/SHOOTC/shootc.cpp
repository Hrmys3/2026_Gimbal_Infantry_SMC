//
// Created by mjw on 2022/10/7.
//
//shootc 增加模式
#include "shootc.h"
#include "drv_can.h"
#include "normal_pid.h"
#include "debugc.h"
#include "remotec.h"
#include "INS_task.h"
#include "math.h"
#include "gimbalc.h"

shootc shoot;
extern ReceivePacket vision_packet;

int8_t shootc::GetFricStatus(void)
{
	if (abs(speed_pids[FRIC_L].Input) > 5500 && abs(speed_pids[FRIC_R].Input) > 5500)
	{
		return FRIC_ON;
	}
	else return FRIC_OFF;
}

shootc::shootc()
{
	ram_pos_pid.ComputeType = PositionPID_e;

	speed_pids[RAM].WorkType = Ramp_e;
	speed_pids[RAM].RampTargetTime = 1;
	speed_pids[RAM].RampTargetStep = 120;
	speed_pids[RAM].ComputeType = IncrementPID_e;

	speed_pids[FRIC_L].WorkType = Normal_e;
	speed_pids[FRIC_L].RampTargetTime = 3;
	speed_pids[FRIC_L].RampTargetStep = 100;
	speed_pids[FRIC_L].ComputeType = IncrementPID_e;

	speed_pids[FRIC_R].WorkType = Normal_e;
	speed_pids[FRIC_R].RampTargetTime = 3;
	speed_pids[FRIC_R].RampTargetStep = 100;
	speed_pids[FRIC_R].ComputeType = IncrementPID_e;

	Channel_StopTimeMax = 50;
}

void shootc::FricControl(void)
{
	motors[FRIC_L].update_angle();
	motors[FRIC_R].update_angle();

	speed_pids[FRIC_L].Update(motors[FRIC_L].Motor_Speed);
	speed_pids[FRIC_R].Update(motors[FRIC_R].Motor_Speed);

	speed_pids[FRIC_L].GetOutput();
	speed_pids[FRIC_R].GetOutput();
}

void shootc::FricSpeedClean(void) //clean不用斜坡
{
	speed_pids[FRIC_L].Target = 0;
	speed_pids[FRIC_R].Target = 0;

	if (abs(speed_pids[FRIC_L].Input) < 1000 || abs(speed_pids[FRIC_R].Input) < 1000) //清除漏电流，防止下载疯转
	{
		speed_pids[FRIC_L].Out = 0;
		speed_pids[FRIC_R].Out = 0;
	}
}

void shootc::RammerSpeedClean(void)
{
	speed_pids[RAM].WorkType = Normal_e;
	speed_pids[RAM].Target = 0;
}

void shootc::FricSpeedReset()
{
	speed_pids[FRIC_L].WorkType = Ramp_e;
	speed_pids[FRIC_L].Target = -SHOOT_SPEED;

	speed_pids[FRIC_R].WorkType = Ramp_e;
	speed_pids[FRIC_R].Target = SHOOT_SPEED;
}


void shootc::RamSpeedTarget(float Ram_Speed, int8_t mode) //bug:有时候目标值弹跳 已解决
{
	FricSpeedReset();
	if (mode == CONTINUOUS) //连发模式，拨弹轮以一定速度旋转
	{
		speed_pids[RAM].WorkType = Ramp_e;
		speed_pids[RAM].Target = Ram_Speed;
		rammer_flag = 0;
	}
	else if (mode == SINGLE) //单发模式
	{
		if (rammer_flag == 0) //掉电初始时还是出现大幅度反转
		{
			//这些东西先都置零
			ram_pos_pid.Err_all = 0;
			ram_pos_pid.LastInput = 0;
			speed_pids[RAM].Err_all = 0;
			motors[RAM].clear(); //这些东西先都置零
		}
		rammer_flag++; //flag记1，单发模式
		ram_pos_pid.Target = 0 + rammer_flag * 45; //只转45度
	}
}

void shootc::Stuck_Check(void)
{
	int16_t Current = motors[RAM].Torque;
	// usart_printf("%d,%d\r\n", Current, stack_time);
	if (Current > 5000) //堵转
	{
		stack_time++;
	}
	if (stack_time >= 100) //堵转超过500ms
	{
		ram_pos_pid.Err_all = 0;
		ram_pos_pid.Err_all = 0;
		RamSpeedTarget(-20, CONTINUOUS); //卡弹时以100rpm速度反转1s
		reverse_time++;
	}
	if (reverse_time >= reverse_time_max)
	{
		reverse_time = 0;
		stack_time = 0;
	}
}//堵转检测

//计算当前枪口热量
void shootc::Heat_Calculate() //这个函数重构后放哪里？先放shoot里，因为可能同时用fric&ram检测
{
	static uint8_t shootspd_drop = 0;
	if (ram_pos_pid.Input > ram_pos_pid.LastInput + 40) //开摩擦轮检测到掉速？？？
	{
		shootspd_drop = 1;
		ram_pos_pid.LastInput = ram_pos_pid.Input;
	}
	if (shootspd_drop == 1)
	{
		Heat_Cal += 10;  //10是一发小弹丸热量
		shootspd_drop = 0;
	}
	Heat_Cal -= (float) judge.cool_rate / 200.0f;  //周期是5ms
	if (Heat_Cal < 0)
	{
		Heat_Cal = 0;
	}
}

void shootc::SetRammer(void)
{
	// motors[0].update_angle();
	speed_pids[RAM].Update(motors[RAM].Motor_Speed);
	speed_pids[RAM].GetOutput();

	MyRemote.portHandle(&MyRemote.rc_ctrl.key.Q);
	MyRemote.portHandle(&MyRemote.rc_ctrl.mouse.press_l);
	int16_t RammerSpeed = 10;
	Channel_Max = 120.0f; //最大拨弹速度-遥控器

	switch (MyRemote.ControlMode)
	{
	case KEY_MODE:
	{
		if (isPermitted())
		{
			speed_pids[RAM].WorkType = Ramp_e;
			speed_pids[RAM].Target = ram_pos_pid.Out;
			if (rammer_flag == 0)
			{
				speed_pids[RAM].WorkType = Ramp_e;
				speed_pids[RAM].Target = 0;
				ram_pos_pid.Err_all = 0; //积分项清除
				ram_pos_pid.Target = 0;
			}

			Channel_Now = MyRemote.rc_ctrl.mouse.press_l.Now_State;
			if (MyRemote.rc_ctrl.mouse.press_l.Is_Click_Once)
			{
				RamSpeedTarget(RammerSpeed, 2);
			}

			motors[RAM].update_angle();
			ram_pos_pid.Update(motors[RAM].Motor_Angle);
			ram_pos_pid.GetOutput();

			if (Channel_Now == Channel_Last && Channel_Now != 0)
			{
				if (Channel_StopTime++ >= Channel_StopTimeMax) //速度环正常
				{
					RammerSpeed = 50;
					if (MyRemote.rc_ctrl.key.R.Now_State == 1) RammerSpeed = 120;//改成与当前热量相关
					RamSpeedTarget(RammerSpeed, 1);
				}
			}
			else Channel_StopTime = 0;
			Channel_Last = Channel_Now;
		}
		else
		{
			RammerSpeedClean();
			RamSpeedTarget(0, 1);
			shoot.speed_pids[RAM].Out = 0;
		}
		break;
	}
	case RC_MODE:
	{
		if (isPermitted())
		{
			speed_pids[RAM].WorkType = Ramp_e;
			speed_pids[RAM].Target = ram_pos_pid.Out;
			if (rammer_flag == 0)
			{
				speed_pids[RAM].WorkType = Ramp_e;
				speed_pids[RAM].Target = 0;
				ram_pos_pid.Target = 0;
				ram_pos_pid.Err_all = 0; //积分项清除
			}

			Channel_Now = abs(MyRemote.rc_ctrl.rc.wheel * Channel_Max / 660.0f);

			if (Channel_Now == Channel_Max && Channel_Last != Channel_Max)
			{
				// ShootSpeedTarget(SHOOT_SPEED, -RammerSpeed, 2);
				RamSpeedTarget(RammerSpeed, SINGLE);
			}

			motors[RAM].update_angle();
			ram_pos_pid.Update(motors[RAM].Motor_Angle);
			ram_pos_pid.GetOutput();

			if (Channel_Now == Channel_Last && Channel_Now != 0)
			{
				if (Channel_StopTime++ >= Channel_StopTimeMax) //速度环正常 英雄：1000000  步兵：50
				{
					RammerSpeed = Channel_Now;
					RamSpeedTarget(RammerSpeed, CONTINUOUS);
				}
			}
			else Channel_StopTime = 0;
			Channel_Last = Channel_Now;
		}
		else
		{
			RammerSpeedClean();
			RamSpeedTarget(0, CONTINUOUS);
			shoot.speed_pids[RAM].Out = 0;
		}
		break;
	}
	}
	if(!motors[1].is_online && !motors[2].is_online)
	{
		ram_pos_pid.Target = 0;
		ram_pos_pid.Err_all = 0;
		speed_pids[RAM].Err_all = 0;
		rammer_flag = 0;
		motors[RAM].clear();
	}
}

void shootc::Heat_Protect(void)
{
	Heat_Calculate();
	if (judge.heat_now1 > Heat_Cal) //默认是Heat_Now1吗
	{
		Heat_Cal = judge.heat_now1 + 20;
	}
	if(judge.heat_now1 < Heat_Cal - 20)
	{
		zerobullettimer++;
		// Heat_Cal = Heat_Now1;
	}
	else zerobullettimer = 0;

	if (zerobullettimer > 200)
	{
		Heat_Cal = judge.heat_now1;
		zerobullettimer = 0;
	}

	if (Heat_Cal > judge.cool_limit - 30)
	{
		heat_permit = FORBID; //即将超热量，不允许发弹
	}
	else heat_permit = PERMIT;
}

bool shootc::isPermitted() {
	if (heat_permit && GetFricStatus() && motors[1].is_online && motors[2].is_online && omni.fric_ram_status == OPENRAMMER) {
		//自瞄模式下，需要上位机发来shoot=1的允许指令
		if (omni.AutoAim) {
			if (vision_packet.shoot) return PERMIT;
		}

		//非自瞄模式
		else return PERMIT;
	}
	else return FORBID;
}


void shootc::ControlLoop(void) //发弹主循环：模式切换和速率测试
{
	if (judge.cool_limit == 0)
	{
		judge.cool_limit = 3000;
		judge.cool_rate = 20;
	}
	FricControl();
	Heat_Protect(); //热量超限保护 - 可选？
	if (omni.AutoAim == 0 || omni.AutoAim == 1 && vision_packet.shoot ==1){ //自瞄模式下需要上位机发送射击指令
		SetRammer(); //拨弹轮速度设置
	}
	Stuck_Check(); //卡弹检测
	// usart_printf("%d\r\n",motors[0].Torque);
	// usart_printf("%f,%f,%f,%f,%d\r\n",ram_pos_pid.Target,ram_pos_pid.Input,speed_pids[0].Target,speed_pids[0].Input,MyRemote.is_online);
}
