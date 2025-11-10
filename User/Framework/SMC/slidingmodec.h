//
// Created by Hrmys3 on 2025/10/23.
//

#include "stm32f4xx_hal.h"
#include "math.h"
#ifndef SMC_H
#define SMC_H

class SMC{
public:
    float alpha; //新增NFTSMC的线性项系数alpha，这里简写为a
    float beta;
    float gamma; //新增，作为速度修正量 gamma (e_dot 项)
    float K;
    float ref;
    float error_eps;
    uint16_t p;
    uint16_t q;
    float u_max;
    float J;
    float angle;
    float ang_vel;
    float epsilon;

    float u;
    SMC(float alpha, float beta,float gamma,float K,float ref,float error_eps,uint16_t p,uint16_t q,float u_max,float J,float epsilon):
    alpha(alpha),beta(beta),gamma(gamma),K(K),ref(ref),error_eps(error_eps),p(p),q(q),u_max(u_max),J(J),epsilon(epsilon){};
    //构造函数中增加alpha 和 K_gamma，必须确保1<p/q<2

    void SMC_Tick(float angle_now,float angle_vel);

//private: //为了测试，先改成public
    float e;
    float error_last;
    float e_dot_pq;
    float ref_dot;
    float ref_ddot;
    float ref_last;
    //const float delta = 0.001f; //NFTSMC新增，用于处理e_dot=0奇异点，加入速度修正量之后不需要

    float s;
    float ds;

    float Sat(float y)
    {
        if (fabs(y) < 1 || fabs(y) == 1)
            return y;
        else
            return Signal(y);
    }

    int8_t Signal(float y)
    {
        if (y > 0)
            return 1;
        else if (y == 0)
            return 0;
        else
            return -1;
    }
};
//extern SMC YawSMC;

#endif //SMC_H
