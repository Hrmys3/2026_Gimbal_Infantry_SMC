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

// 构造函数
SMC::SMC(float alpha, float beta, float gamma, float K, float ref, float error_eps,
         uint16_t p, uint16_t q, float u_max, float J, float epsilon_base,
         float dt, float alpha_dot, float phi, float beta_eps)
    : alpha(alpha), beta(beta), gamma(gamma), K(K), ref(ref), error_eps(error_eps),
      p(p), q(q), u_max(u_max), J(J), epsilon_base(epsilon_base),
      dt(dt), alpha_dot(alpha_dot), phi(phi), coefficient_eps(beta_eps) {

    // 初始化状态
    Reset();
}

// 重置状态函数
void SMC::Reset() {
    u = 0.0f;
    e = 0.0f;
    e_dot = 0.0f;
    e_dot_last = 0.0f;
    ref_last = ref; // 假设初始目标为ref
    ref_dot = 0.0f;
    ref_dot_last = 0.0f;
    s = 0.0f;
}

// 主计算函数
void SMC::SMC_Tick(float angle_now, float angle_vel, float feedforward) {

    // --- 1. 计算误差、目标导数 ---
    e = angle_now - ref;

    // 优化点 1: 使用 dt 计算微分
    // if (dt <= 0.0f) return; // 防止除以0
    ref_dot = (ref - ref_last) / dt;
    ref_dot = (1.0f - 0.5f) * ref_dot_last + 0.5f * ref_dot; // (可选)给ref_dot也加个滤波
    float ref_ddot = (ref_dot - ref_dot_last) / dt;

    // --- 2. 计算 e_dot 并滤波 ---
    float e_dot_raw = angle_vel - ref_dot;
    // 优化点 2: e_dot 一阶低通滤波
    e_dot = (1.0f - alpha_dot) * e_dot_last + alpha_dot * e_dot_raw; //之后可以取消掉alpha_dot这个参数，设置为定值

    // --- 3. 检查死区 ---
    if (fabsf(e) < error_eps && fabsf(e_dot) < error_eps) {
        // 在死区内，可以考虑重置状态或保持输出为0
        // u = 0; // 保持上次输出或置零，这里置零
        // return;
        // 保持计算可能更平滑，这里不直接返回
    }

    // --- 4. 增益调度 (Gain Scheduling) ---
    // (这是对原代码 if(abs(e)<1) 逻辑的更安全实现)
    float alpha_eff = alpha;
    float K_eff = K;
    float J_eff = J;
    float ref_ddot_eff = ref_ddot;
    float beta_eff = beta;

    float dead_band = 1.0f; // 原代码的 (abs(e) < 1)
    if (fabsf(e) < dead_band) {
        // // 进入小误差区，调整参数（原代码逻辑）
        // alpha_eff *= 0.5f;
        // K_eff *= 0.7f;
        // J_eff *= 2.0f;
        // ref_ddot_eff *= 0.5f;
        // // beta_eff = 0; // 线性化滑模面
    }

    // --- 5. 计算 NFTSMC 滑模面 s ---
    // s = alpha*e + beta*e_dot_pq + gamma*e_dot
    float pq = (float)p / (float)q;
    float e_dot_pq;

    if (fabsf(e) < dead_band) {
        // 原代码逻辑：小误差时，将非线性项 e_dot_pq 改为线性项 e_dot
        // s = alpha_eff * e + beta_eff * e_dot + gamma * e_dot
        e_dot_pq = e_dot;
        // beta_eff = 0, gamma_eff = beta + gamma (等效)
    } else {
        // 大误差时，使用非线性项
        e_dot_pq = powf(fabsf(e_dot), pq) * Signal(e_dot);
    }

    // 最终滑模面
    s = alpha_eff * e + beta_eff * e_dot_pq + gamma * e_dot;


    // --- 6. 计算控制律 u ---

    // 优化点 5: 计算自适应 epsilon
    float epsilon_eff = epsilon_base / (1.0f + coefficient_eps * fabsf(e));

    // 期望的趋近律 s_dot_des = -K*s - epsilon*sat(s)
    // 优化点 4: Sat() 内部已替换为 tanhf
    float s_dot_des = -K_eff * s - epsilon_eff * Sat(s);

    // 计算增益 G = C * (p/q) * |e_dot|^((p/q)-1) + gamma
    float pq_minus_1 = pq - 1.0f;
    float gain_term;

    // G 的计算需要特别注意 e_dot 接近 0 的情况
    if (fabsf(e_dot) < 1e-4f) {
        // e_dot 极小时，gain_term 主要由 gamma 决定
        gain_term = gamma;
    } else {
        gain_term = beta_eff * pq * powf(fabsf(e_dot), pq_minus_1) + gamma;
    }

    // 避免 G 过小导致控制律爆炸
    if (fabsf(gain_term) < 1e-3f) {
        gain_term = 1e-3f * Signal(gain_term);
    }

    // u = J * (ref_ddot + (s_dot_des - alpha*e_dot) / G)
    float numerator = s_dot_des - alpha_eff * e_dot;

    // 优化点 3: 加入前馈补偿
    u = J_eff * (ref_ddot_eff + numerator / gain_term) - feedforward;

    // --- 7. 输出限幅 ---
    if (u > u_max) u = u_max;
    if (u < -u_max) u = -u_max;

    // --- 8. 更新状态变量 ---
    ref_last = ref;
    ref_dot_last = ref_dot;
    e_dot_last = e_dot;
}