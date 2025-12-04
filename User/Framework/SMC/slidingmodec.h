//
// Created by Hrmys3 on 2025/10/23.
//

#include "stm32f4xx_hal.h"
#include "math.h"
#ifndef SMC_H
#define SMC_H

class SMC {
public:
    // --- 原始参数 (NFTSMC) ---
    float alpha; // s = alpha*e + ...
    float beta;  // s = ... + beta*e_dot_pq + ...
    float gamma; // s = ... + gamma*e_dot (速度修正量)
    float K;     // 趋近律 s_dot = -K*s - ...
    float ref;   // 目标值
    float error_eps; // 误差死区
    uint16_t p;
    uint16_t q;
    float u_max; // 最大输出
    float J;     // 系统惯量估计

    // --- 优化点 1: dt ---
    float dt; // 控制周期 (s)

    // --- 优化点 2: e_dot 滤波器 ---
    float alpha_dot; // e_dot 的一阶低通滤波系数

    // --- 优化点 4: Tanh 饱和函数 ---
    float phi; // tanh 边界层厚度

    // --- 优化点 5: 自适应 Epsilon ---
    float epsilon_base; // 基础鲁棒增益
    float coefficient_eps;     // Epsilon 自适应系数

    // --- 成员变量 (状态) ---
    float u; // 控制器输出

    // 构造函数
    SMC(float alpha, float beta, float gamma, float K, float ref, float error_eps,
        uint16_t p, uint16_t q, float u_max, float J, float epsilon_base,
        float dt, float alpha_dot = 0.8f, float phi = 0.05f, float beta_eps = 2.0f);

    // 主计算函数
    // 优化点 3: 加入 feedforward
    void SMC_Tick(float angle_now, float angle_vel, float feedforward);

    // 重置状态
    void Reset();

    float get_e() {
        return e;
    }
    float get_e_dot() {
        return e_dot;
    }
    float get_s() {
        return s;
    }

private:
    // --- 状态变量 ---
    float e;
    float e_dot;
    float e_dot_last;
    float ref_last;
    float ref_dot; // 目标角速度
    float ref_dot_last; // 上一次的目标角速度
    float s;

    // --- 辅助函数 ---
    int8_t Signal(float y) {
        if (y > 0) return 1;
        if (y < 0) return -1;
        return 0;
    }

    // 优化点 4: Sat(y) 优化为 tanhf
    float Sat(float y) {
        return tanhf(y / phi);
    }
};

#endif //SMC_H
