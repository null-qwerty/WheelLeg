#pragma once

#include "BaseControl/Controller/pidController.hpp"
#include "BaseControl/Motor/Motor.hpp"

#include "Math/Vector.hpp"

#include "Utils/Status.hpp"

/**
 * @brief 五连杆并联腿机构
 *
 * @details
 * 五连杆并联腿机构，由两个关节和一个轮子组成，关节分别为前关节和后关节，轮子为腿的末端。类内带有
 * VMC 模型解算，将输入的力矩（切向和径向）转换为关节力矩。
 * 模型构建见[文档](https://blog.null-qwerty.work/2025/01/24/%E4%BA%94%E8%BF%9E%E6%9D%86%E6%AD%A3%E8%BF%90%E5%8A%A8%E5%AD%A6%E8%A7%A3%E7%AE%97/)。
 */
class Leg {
public:
    struct State {
        float theta; ///< 摆杆角度
        float theta_dot;
        float x; ///< 位移
        float x_dot;
        float phi; ///< 机体角度
        float phi_dot;
        float l; ///< 腿长
        Status status;
    };
    Leg(Joint &frontJoint, Joint &backJoint, Wheel &wheel, float dt);
    Leg &init();

    Leg &setLegLenth(float l1, float l2, float l3, float l4, float l5);
    Leg &setLinearzationParam(Matrix<12, 6> p);
    Leg &setLegLenthController(pidController controller);
    Leg &setRollCompensation(float f);
    Leg &setYawCompensation(float t);
    Leg &setThetaCompensation(float t);

    State &getCurrentState();
    State &getTargetState();

    Leg &calculateTotalTorque();

private:
    /* 执行器 */
    Joint &frontJoint, &backJoint; ///< 关节电机
    Wheel &wheel; ///< 轮子电机

    float dt; ///< 控制周期，单位为秒

    /* VMC 解算参数 */
    float l1, l2, l3, l4, l5; ///< 五连杆长度
    float phi1, phi2, phi3, phi4; ///< 五连杆角度
    Vector2f A, B, C, D, E; ///< 五连杆顶点，A, E 为前后关节，C 为轮子顶点
    Vector2f F; ///< 摆杆力矩 [F, T_p]，分别为沿腿方向和沿摆杆方向的力矩
    Vector2f T; ///< 关节输出力矩 [T1, T2]，分别为前关节和后关节的力矩

    /* 控制平衡状态的 LQR 控制器参数 */
    State current_state, target_state; ///< 当前状态和目标状态
    Vector6f X; ///< 状态向量 [theta, theta_dot, x, x_dot, phi, phi_dot]
    Vector6f X_d; ///< 期望状态向量 [0, 0, x_d, 0, 0, 0]
    Matrix<2, 6> K; ///< LQR 控制器增益矩阵
    Matrix<12, 6> pk; ///< K 关于腿长 l 的线性化拟合参数，最多五次项拟合
    Vector2f u; ///< 控制向量 [T, T_p]，分别为轮的力矩和摆杆力矩
    Vector2f u_balance; ///< 平衡控制向量

    /* 腿长控制 */
    pidController leg_length_controller; ///< 为保证减震效果，K_p 应较小
    float leg_length_max;
    float leg_length_min;
    float F_leg_length; ///< 腿长控制力矩
    float F_roll_compensation; ///< 横滚角补偿力矩，补偿控制器误差

    /* 转向控制 */
    float T_yaw; ///< 转向动力轮力矩

    /* 双腿协调 */
    float T_p_theta; ///< 补偿摆动力矩使双腿摆动角度相同

    State calculateState(float angle_front, float angle_back);
    Vector2f transformToJointTorque(Vector2f &u);
    Matrix<2, 6> &calculateK();
};