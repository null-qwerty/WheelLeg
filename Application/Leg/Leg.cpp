#include "Leg/Leg.hpp"

#include "Math/Matrix.hpp"
#include "Math/Trigonometric.hpp"
#include "Math/Vector.hpp"
#include "Utils/Status.hpp"

Leg::Leg(Joint &frontJoint, Joint &backJoint, Wheel &wheel, float dt)
    : frontJoint(frontJoint)
    , backJoint(backJoint)
    , wheel(wheel)
    , dt(dt)
{
}

Leg &Leg::init()
{
    current_state.status = STATUS_INITUALIZING;

    target_state.theta = current_state.theta = 0.0f;
    target_state.theta_dot = current_state.theta_dot = 0.0f;
    target_state.x = current_state.x = 0.0f;
    target_state.x_dot = current_state.x_dot = 0.0f;
    target_state.phi = current_state.phi = 0.0f;
    target_state.phi_dot = current_state.phi_dot = 0.0f;

    auto max = frontJoint.getOptionData().soft_limit_max;
    auto min = frontJoint.getOptionData().soft_limit_min;

    leg_length_max = calculateState(max, max).l;
    leg_length_min = calculateState(min, min).l;

    // target_state.l = (leg_length_max + leg_length_min) / 2.0f;
    target_state.l = leg_length_min;

    calculateCurrentState();
    current_state.status = STATUS_INITUALIZED;

    return *this;
}

Leg &Leg::deInit()
{
    current_state.status = STATUS_DEINITUALIZING;
    frontJoint.deInit();
    backJoint.deInit();
    wheel.deInit();
    current_state.status = STATUS_DEINITUALIZED;

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

Leg &Leg::setLegLenthController(pidController controller)
{
    leg_length_controller = controller;
    return *this;
}

Leg &Leg::setRollCompensation(float f)
{
    F_roll_compensation = f;
    return *this;
}

Leg &Leg::setYawCompensation(float t)
{
    T_yaw = t;
    return *this;
}

Leg &Leg::setThetaCompensation(float t)
{
    T_p_theta = t;
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

float Leg::getTheta()
{
    return current_state.theta;
}

float Leg::getHalfLegLengthRange()
{
    return (leg_length_max - leg_length_min) / 2.0f;
}

Leg &Leg::calculateTotalTorque()
{
    // if (current_state.status != STATUS_INITUALIZED ||
    //     frontJoint.getState().status != STATUS_INITUALIZED ||
    //     backJoint.getState().status != STATUS_INITUALIZED ||
    //     wheel.getState().status != STATUS_INITUALIZED) {
    //     wheel.getTargetState().toreque = 0;
    //     frontJoint.getTargetState().toreque = 0;
    //     backJoint.getTargetState().toreque = 0;
    //     return *this;
    // }

    //     计算当前状态与目标状态的差值向量
    X = Vector6f((float *)(&current_state));
    X_d = Vector6f((float *)(&target_state));
    u_balance = K * (X_d - X); // 计算为了保持平衡状态的虚拟力矩
    // u_balance = Vector2f::Zero();
    F_leg_length = leg_length_controller.calculate(
        target_state.l, current_state.l); // 计算腿长控制力矩

    u[0] = u_balance[0] + T_yaw;
    u[1] = u_balance[1] + T_p_theta;

    F[0] = F_leg_length + F_roll_compensation;
    F[1] = u[1];

    T = transformToJointTorque(F); // 将合成力矩转换为关节力矩

    wheel.getTargetState().toreque = u[0]; // 设置轮子的目标力矩
    frontJoint.getTargetState().toreque = T[0]; // 设置前关节的目标力矩
    backJoint.getTargetState().toreque = T[1]; // 设置后关节的目标力矩
    // wheel.getTargetState().toreque = 0;
    // frontJoint.getTargetState().toreque = 0;
    // backJoint.getTargetState().toreque = 0;

    return *this;
}

Leg &Leg::calculateCurrentState()
{
    estimated_state = calculateState(frontJoint.getState().position,
                                     backJoint.getState().position);
    current_state.theta_dot =
        (estimated_state.theta - current_state.theta) / dt; // 摆杆角速度
    current_state.theta = estimated_state.theta;
    current_state.l = estimated_state.l;
    current_state.x_dot = wheel.getState().velocity * PI / 60.0f * 0.10640f;

    calculateK(); // 计算增益矩阵 K

    float rate = fabsf(K(0, 4) / K(0, 3));
    if (rate > 10)
        rate = 10;
    if (fabsf(current_state.x_dot) > fabsf(current_state.phi) * rate) {
        current_state.x_dot = 0;
    }

    return *this;
}

Vector2f Leg::transformToJointTorque(Vector2f &u)
{
    auto &phi0 = current_state.phi;
    auto &l0 = current_state.l;

    Matrix2x2f T;
    Vector2f F;

    auto sigma = Math::sin(phi3 - phi2);
    // clang-format off
    T << l1 * Math::sin(phi0 - phi3) * Math::sin(phi1 - phi2) / sigma,
         l1 * Math::cos(phi0 - phi3) * Math::sin(phi1 - phi2) / (l0 * sigma),
         l4 * Math::sin(phi0 - phi2) * Math::sin(phi3 - phi4) / sigma,
         l4 * Math::cos(phi0 - phi2) * Math::sin(phi3 - phi4) / (l0 * sigma);
    // clang-format on
    F = T * u;

    return F;
}

Leg::State Leg::calculateState(float angle_front, float angle_back)
{
    State estimated_state = current_state;
    phi4 = angle_front;
    phi1 = PI - angle_back;

    B[0] = A[0] + l1 * Math::cos(phi1);
    B[1] = A[1] + l1 * Math::sin(phi1);
    D[0] = E[0] + l4 * Math::cos(phi4);
    D[1] = E[1] + l4 * Math::sin(phi4);

    auto vBD = D - B;
    auto lBD2 = vBD.dot(vBD);
    auto m = 2 * l2 * vBD[0];
    auto n = 2 * l2 * vBD[1];
    auto p = l3 * l3 - l2 * l2 + lBD2;
    phi2 = 2 * Math::atan2(n + sqrt(m * m + n * n - p * p), m + p);

    auto vDB = B - D;
    auto lDB2 = lBD2;
    m = 2 * l3 * vDB[0];
    n = 2 * l3 * vDB[1];
    p = l2 * l2 - l3 * l3 + lDB2;
    phi3 = 2 * PI - 2 * Math::atan2(n + sqrt(m * m + n * n - p * p), m + p);

    C[0] = B[0] + l2 * Math::cos(phi2);
    C[1] = B[1] + l2 * Math::sin(phi2);

    estimated_state.theta = PI / 2 - Math::atan2(C[1], C[0]);
    estimated_state.l = C.magnitude();

    return estimated_state;
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