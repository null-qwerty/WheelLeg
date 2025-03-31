#include "Leg.hpp"

#include "Math/Matrix.hpp"
#include "Math/Trigonometric.hpp"

Leg::Leg(Joint &frontJoint, Joint &backJoint, Wheel &wheel)
    : frontJoint(frontJoint)
    , backJoint(backJoint)
    , wheel(wheel)
{
}

Leg &Leg::init()
{
    /**
     * TODO 初始化腿部关节电机
     *      1. 撞击限位，设置软零点
     *      2. 将关节电机设置到水平位置
     */

    return *this;
}

Leg &Leg::setParam(float l1, float l2, float l3, float l4, float l5)
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

Leg &Leg::setJointLimit(float maxAngle, float minAngle)
{
    angleLimitMax = maxAngle;
    angleLimitMin = minAngle;

    return *this;
}

Leg &Leg::setVirtualTorque(float vertical, float horizontal)
{
    calculateCurrentState();

    auto &phi0 = state.phi0;
    auto &l0 = state.l0;
    // \begin{align}
    // T(t) &= \begin{bmatrix}
    // \frac{l_1\Math::sin(\phi_0-\phi_3)\Math::sin(\phi_1-\phi_2)}{\Math::sin(\phi_3-\phi_2)}
    // &
    // \frac{l_1\Math::cos(\phi_0-\phi_3)\Math::sin(\phi_1-\phi_2)}{l_0\Math::sin(\phi_3-\phi_2)}
    // \newline
    // \frac{l_4\Math::sin(\phi_0-\phi_2)\Math::sin(\phi_3-\phi_4)}{\Math::sin(\phi_3-\phi_2)}
    // &
    // \frac{l_4\Math::cos(\phi_0-\phi_2)\Math::sin(\phi_3-\phi_4)}{l_0\Math::sin(\phi_3-\phi_2)}
    // \end{bmatrix}
    // \end{align}
    Matrix2x2f T;
    // clang-format off
    T << l1 * Math::sin(phi1 - phi3) * Math::sin(phi0 - phi2) / Math::sin(phi3 - phi2),
         l1 * Math::cos(phi1 - phi3) * Math::sin(phi0 - phi2) / (l0 * Math::sin(phi3 - phi2)),
         l4 * Math::sin(phi0 - phi2) * Math::sin(phi3 - phi4) / Math::sin(phi3 - phi2),
         l4 * Math::cos(phi0 - phi2) * Math::sin(phi3 - phi4) / (l0 * Math::sin(phi3 - phi2));
    // clang-format on
    Vector2f q, t;
    q << vertical, horizontal;
    t = T * q;

    frontJoint.getTargetState().toreque = t[0];
    backJoint.getTargetState().toreque = t[1];

    return *this;
}

Leg &Leg::setWheelTorque(float torque)
{
    wheel.getTargetState().toreque = torque;

    return *this;
}

Leg::State &Leg::getState()
{
    calculateCurrentState();

    return state;
}

Leg &Leg::calculateCurrentState()
{
    phi1 = PI - frontJoint.getState().position;
    phi4 = backJoint.getState().position;
    // x_B = x_A + l_1 * Math::cos(phi_1);
    // y_B = y_A + l_1 * Math::sin(phi_1);
    // x_D = x_E + l_4 * Math::cos(phi_4);
    // y_D = y_E + l_4 * Math::sin(phi_4);
    B[0] = A[0] + l1 * Math::cos(phi1);
    B[1] = A[1] + l1 * Math::sin(phi1);
    D[0] = E[0] + l4 * Math::cos(phi4);
    D[1] = E[1] + l4 * Math::sin(phi4);
    // l_BD_2 = (x_B - x_D) * (x_B - x_D) + (y_B - y_D) * (y_B - y_D);
    // m = 2l_2(x_B - x_D)
    // n = 2l_2(y_B - y_D)
    // p = l_3_2 - l_2_2 - l_BD_2
    // phi2 = 2 * atan2(n + sqrt(m*m + n*n - p*p), m + p);
    // phi3 = 2 * atan2(n - sqrt(m*m + n*n - p*p), m + p);
    auto vBD = B - D;
    auto lBD2 = vBD.dot(vBD);
    auto m = 2 * l2 * vBD[0];
    auto n = 2 * l2 * vBD[1];
    auto p = l3 * l3 - l2 * l2 - lBD2;
    phi2 = 2 * Math::atan2(n + sqrt(m * m + n * n - p * p), m + p);
    phi3 = 2 * Math::atan2(n - sqrt(m * m + n * n - p * p), m + p);
    // x_dot_C = diff(x_C, t);
    // y_dot_C = diff(y_C, t);
    C[0] = B[0] + l2 * Math::cos(phi2);
    C[1] = B[1] + l2 * Math::sin(phi2);
    state.phi0 = Math::atan2(C[1], C[0]);
    state.l0 = C.magnitude();

    return *this;
}