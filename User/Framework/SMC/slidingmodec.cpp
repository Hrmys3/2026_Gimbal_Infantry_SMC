//
// Created by Hrmys3 on 2025/10/23.
//

/*
更新了滑模控制，采用非奇异快速终端滑模（NFTSMC），并加入了速度修正量（Velocity Compensation）
*/

#include "slidingmodec.h"
#include<cmath>

#include"gimbalc.h"
#include"remotec.h" //小陀螺+自瞄模式下加入补偿量


// 示例初始化：alpha=10, beta(C)=0.5, gamma(K_gamma)=0.1, K=120, p=27, q=21
//SMC YawSMC(10.0, 0.5, 0.1, 120, 0, 0.001, 27, 21, 25000, 0.8, 0.5);
//原SMC YawSMC(20, 120, 0, 0.001, 21, 27, 25000, 0.8, 0.5);
//各参数含义：SMC(float alpha, float C,float K_gamma,float K,float ref,float error_eps,uint16_t p,uint16_t q,float u_max,float J,float epsilon):

//使用非奇异快速终端滑模（NFTSMC）+速度修正量
//注意：要求？qp范围？
// void SMC::SMC_Tick(float angle_now, float angle_vel) {
//     angle = angle_now;
//     ang_vel = angle_vel;
//
//     // 1. 计算误差和参考导数
//     error = angle - ref;
//     float e_dot = ang_vel - dref; //error的微分（e上一点）
//     float e_dot_qp; //error微分的(q/p)次幂
//
//     dref = (ref - refl);
//     ddref = (ref - refl) - dref;
//
//     refl = ref;
//
//     // 2. 检查死区
//     if (fabs(error) < error_eps)
//     {
//         u = 0;
//         return;
//     }
//
//     // 3. 计算 NFTSMC + 速度修正 滑模面 s
//     // s = alpha * e + C * e_dot^(p/q) + gamma * e_dot
//     float qp = (float)q / (float)p;
//
//     // e_dot_pq计算：分情况讨论
//     if (error < 0) {
//         e_dot_qp = -pow(abs(e_dot), qp);
//     }
//     else {
//         e_dot_qp = pow(abs(e_dot), qp);
//     }
//
//     // 滑模面s
//     s = alpha * error + beta * e_dot_qp + gamma * e_dot;
//
//     // 4. 计算控制律 u
//     // 趋近律
//     float s_dot = -K * s - epsilon * Sat(s); //s_dot的另一种表示
//
//     // 以下部分由计算得到
//     // 增益 G = C * (p/q) * |e_dot|^((q/p)-1) + gamma
//     //float qp_minus_1 = qp - 1.0f; // (p/q) - 1，正数
//     //float gain_term = beta * qp * powf(fabs(e_dot), qp_minus_1) + gamma;
//     float gain_term = beta * qp * e_dot_qp / fabs(e_dot) + gamma;
//
//     // u_eq 部分的分子
//     //float numerator = s_dot - alpha * e_dot;
//
//     // *** 无需处理奇异点 ***
//     // G_new 始终 > 0 (因为 gamma > 0)
//     u = J * (ddref + (s_dot - alpha * e_dot) / gain_term);
//
//     // 5. 输出限幅
//     if (u > u_max) {
//         u = u_max;
//     }
//     if (u < -u_max) {
//         u = -u_max;
//     }
// }


void SMC::SMC_Tick(float angle_now, float angle_vel) {
    angle = angle_now;
    ang_vel = angle_vel;

    // --- 1. 计算误差、误差微分、目标角度、目标角度微分 ---
    e = angle - ref;
    ref_ddot = (ref - ref_last) - ref_dot;
    ref_dot = (ref - ref_last);
    float e_dot = ang_vel - ref_dot;

    //小陀螺+自瞄模式下的需要一定补偿量
    float spin_bias;
    float last_yaw_ecd;
    float spin_bias_dot;

    if (MyRemote.ControlMode == AUTOAIM_MODE && MyRemote.MotionMode == SPIN) {
        //小陀螺+自瞄模式下的补偿量
    }

    // --- 2. 检查死区 ---
    if (abs(e) < error_eps && abs(e_dot) < error_eps) {
        u = 0;
        return;
    }

    // --- 3. 计算 NFTSMC + 速度修正 滑模面 s ---
    // s = alpha * e + C * e_dot^(p/q) + K_gamma * e_dot
    float pq = (float)p / (float)q;

    // e_dot_pq = |e_dot|^(p/q) * sign(e_dot)
    //float e_dot_pq = powf(fabs(e_dot), pq) * Signal(e_dot);
    e_dot_pq = powf(abs(e_dot), pq) * Signal(e_dot);

    s = alpha * e + beta * e_dot_pq + gamma * e_dot;
    //误差较小时，调整参数并把非线性项e_dot_pq改为线性项e_dot
    if (abs(e) < 1) {
        alpha = alpha * 0.5;
        //beta = beta * 0.5;
        e_dot_pq = e_dot;
    }

    // --- 4. 计算控制律 u ---
    // 趋近律
    float s_dot_des = -K * s - epsilon * Sat(s);
    if (abs(e) < 1) {
        K = K * 0.7;
    }

    // 计算增益 G = C * (p/q) * |e_dot|^((p/q)-1) + gamma
    float pq_minus_1 = pq - 1.0f; // (p/q) - 1，正数，并非！！！啊
    float gain_term = beta * pq * powf(fabs(e_dot), pq_minus_1) + gamma;

    // 计算 u_eq 部分的分子
    float numerator = s_dot_des - alpha * e_dot;

    // *** 无需处理奇异点 ***
    // G_new 始终 > 0 (因为 gamma > 0)
    if (abs(e) < 1) {
        J = J * 2.0;
        ref_ddot = ref_ddot * 0.5;
    } //处理e的二阶微分引起的抖振问题，是这样处理吗？？？
    u = J * (ref_ddot + numerator / gain_term);

    // --- 5. 输出限幅 ---
    if (u > u_max) u = u_max;
    if (u < -u_max) u = -u_max;

    // 6. 更新
    ref_last = ref;
    //last_yaw_ecd = ...
}