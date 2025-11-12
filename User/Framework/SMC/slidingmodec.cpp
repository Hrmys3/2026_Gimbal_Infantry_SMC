#include"slidingmodec.h"
#include<cmath>

#include"gimbalc.h"
#include"remotec.h" //小陀螺+自瞄模式下加入补偿量 很乱，之后要再改
#include "usartio.h"

//SMC YawSMC(20, 120, 0, 0.001, 21, 27, 25000, 0.8, 0.5);

void SMC::SMC_Tick(float angle_now,float angle_vel)
{
	angle = angle_now;
	ang_vel = angle_vel;
	float e_qp;
	float qp = q/p;

	// static float last_speed = 0;
	// float current_speed = omni.motors[0].Speed_Ecd;
	float spin_bias = omni.motors[0].Speed_Ecd * 2	; //原本直接加在target上可行的值
	// float spin_bias_dot = (current_speed - last_speed) * 0.025;

	error = angle - target;
	float error_dot = ang_vel - target_dot;
	target_ddot = (target - target_last) - target_dot;
	target_dot = (target - target_last);
	// if (MyRemote.ControlMode == AUTOAIM_MODE && MyRemote.MotionMode == TUOLUO) { //类似一种前馈补偿？
	if (MyRemote.Control_Mode == AUTOAIM_MODE) {
		// error = angle  - target - spin_bias; //只在自瞄模式下使用前馈
		// error_dot = ang_vel - target_dot + spin_bias;

	}
	usart_printf("%.3f, %.3f\r\n", ang_vel, spin_bias);

	if (fabs(error) < error_eps)
	{
		u = 0;
		return;
	}

	if (error < 0)
		e_qp = -pow(abs(error), qp);
	else
		e_qp = pow(abs(error), qp);

	s = error_dot + C * e_qp;
	ds = -epsilon * Sat(s) - K * s;
	u = J * (target_ddot + ds - C * qp * error_dot * e_qp / abs(error));

	if (abs(error) < 1)
	{
		error = angle  - target;

		s = C * error + error_dot;//smc surface
		u = J * (target_ddot - C * error_dot - epsilon * Sat(s) - K * s);
	}

	// if (MyRemote.Control_Mode == AUTOAIM_MODE && MyRemote.SportMode == TUOLUO) { //类似前馈补偿？
	// 	// u += ; //由spin_bias计算出最后输出的补偿量
	// }

	if (u > u_max)
		u = u_max;
	if (u < -u_max)
		u = -u_max;

	target_last = target;
	// last_speed = current_speed;
}