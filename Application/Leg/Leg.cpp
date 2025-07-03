#include "Leg/Leg.hpp"

#include "Math/Matrix.hpp"
#include "Math/Trigonometric.hpp"
#include "Math/Vector.hpp"

Leg::Leg(Joint &frontJoint, Joint &backJoint, Wheel &wheel)
    : frontJoint(frontJoint)
    , backJoint(backJoint)
    , wheel(wheel)
{
}

Leg &Leg::init()
{
    // TODO 初始化腿部关节电机
    //  1. 撞击限位，设置软零点
    //  2. 将关节电机设置到水平位置

    return *this;
}

Leg &Leg::setLegLenth(float l1, float l2, float l3, float l4, float l5)
{
    this->l1 = l1;
    this->l2 = l2;
    this->l3 = l3;
    this->l4 = l4;
    this->l5 = l5;

    A[0] = -l5 / 2;
    A[1] = 0;
    E[0] = l5 / 2;
    E[1] = 0;

    return *this;
}

Leg &Leg::setLinearzationParam(Matrix<12, 6> p)
{
    pk = p;

    return *this;
}

Leg &Leg::setJointLimit(float maxAngle, float minAngle)
{
    angleLimitMax = maxAngle;
    angleLimitMin = minAngle;

    return *this;
}

Leg::State &Leg::getCurrentState()
{
    return current_state;
}

Leg::State &Leg::getTargetState()
{
    return target_state;
}

Leg &Leg::calculateTotalTorque()
{
    X = Vector6f((float *)(&target_state)) -
        Vector6f((float *)(&current_state)); // 计算当前状态与目标状态的差值向量
    u_balance = calculateK() * X; // 计算为了保持平衡状态的虚拟力矩
    // u_balance[0]: 动力轮力矩
    // u_balance[1]: 摆杆力矩

    // TODO: 添加控制腿长的力矩计算；留接口用于外部传入防止劈叉的控制力矩
    // 合成力矩
    u[0] = 0; // 径向力矩
    u[1] = u_balance[1]; // 切向力矩
    transformToJointTorque(u); // 将合成力矩转换为关节力矩

    frontJoint.getTargetState().toreque = u[0]; // 设置前关节的目标力矩
    backJoint.getTargetState().toreque = u[1];

    return *this;
}

Leg &Leg::transformToJointTorque(Vector2f &u)
{
    calculateCurrentState();

    auto &phi0 = current_state.phi;
    auto &l0 = current_state.l;

    Matrix2x2f T;
    // clang-format off
    T << l1 * Math::sin(phi1 - phi3) * Math::sin(phi0 - phi2) / Math::sin(phi3 - phi2),
         l1 * Math::cos(phi1 - phi3) * Math::sin(phi0 - phi2) / (l0 * Math::sin(phi3 - phi2)),
         l4 * Math::sin(phi0 - phi2) * Math::sin(phi3 - phi4) / Math::sin(phi3 - phi2),
         l4 * Math::cos(phi0 - phi2) * Math::sin(phi3 - phi4) / (l0 * Math::sin(phi3 - phi2));
    // clang-format on
    u = T * u;

    return *this;
}

Leg &Leg::calculateCurrentState()
{
    phi1 = PI - frontJoint.getState().position;
    phi4 = backJoint.getState().position;

    B[0] = A[0] + l1 * Math::cos(phi1);
    B[1] = A[1] + l1 * Math::sin(phi1);
    D[0] = E[0] + l4 * Math::cos(phi4);
    D[1] = E[1] + l4 * Math::sin(phi4);

    auto vBD = B - D;
    auto lBD2 = vBD.dot(vBD);
    auto m = 2 * l2 * vBD[0];
    auto n = 2 * l2 * vBD[1];
    auto p = l3 * l3 - l2 * l2 - lBD2;

    phi2 = 2 * Math::atan2(n + sqrt(m * m + n * n - p * p), m + p);
    phi3 = 2 * Math::atan2(n - sqrt(m * m + n * n - p * p), m + p);

    C[0] = B[0] + l2 * Math::cos(phi2);
    C[1] = B[1] + l2 * Math::sin(phi2);

    current_state.phi = Math::atan2(C[1], C[0]);
    current_state.l = C.magnitude();

    return *this;
}

Matrix<2, 6> &Leg::calculateK()
{
    auto &l = current_state.l;
    Vector6f l0;
    l0 << l * l * l * l * l, l * l * l * l, l * l * l, l * l, l, 1;
    auto tempK = pk * l0; // 线性拟合腿长为 l 的增益矩阵
    K = (float *)(tempK);

    return this->K;
}