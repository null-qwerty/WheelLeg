#pragma once

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
        float theta;
        float theta_dot;
        float x;
        float x_dot;
        float phi;
        float phi_dot;
        float l;
        Status status;
    };
    Leg(Joint &frontJoint, Joint &backJoint, Wheel &wheel);
    Leg &init();

    Leg &setLegLenth(float l1, float l2, float l3, float l4, float l5);
    Leg &setLinearzationParam(Matrix<12, 6> p);
    Leg &setJointLimit(float maxAngle, float minAngle);

    State &getCurrentState();
    State &getTargetState();

    Leg &calculateTotalTorque();

private:
    Joint &frontJoint, &backJoint;
    Wheel &wheel;

    float l1, l2, l3, l4, l5;
    float phi1, phi2, phi3, phi4;
    Vector2f A, B, C, D, E;

    State current_state, target_state;
    Vector6f X;
    Matrix<2, 6> K;
    Matrix<12, 6> pk;
    Vector2f u;
    Vector2f u_balance;

    float angleLimitMax, angleLimitMin;
    float frontJointHorizonAngle, backJointHorizonAngle;

    Leg &calculateCurrentState();
    Leg &transformToJointTorque(Vector2f &u);
    Matrix<2, 6> &calculateK();
};