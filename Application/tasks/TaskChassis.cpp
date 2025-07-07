#include "BaseControl/Controller/pidController.hpp"
#include "FreeRTOS.h"
#include "Leg/Leg.hpp"

#include "Math/Vector.hpp"
#include "Utils/Status.hpp"
#include "cmsis_os.h"
#include "cmsis_os2.h"
#include "portmacro.h"
#include "projdefs.h"
#include "tasks.hpp"

xTaskHandle legInitHandle;
xTaskHandle legControllHandle;
xTaskHandle legDeinitTaskHandle;
xTaskHandle legStateUpdateHandle;

Leg left_leg(lfJoint, lbJoint, leftWheel, 0.002);
Leg right_leg(rfJoint, rbJoint, rightWheel, 0.002);

extern Vector3f chassis_eular_angle;
extern Vector3f chassis_gyro, chassis_accel;
float &yaw = chassis_eular_angle[2];
float &pitch = chassis_eular_angle[1];
float &roll = chassis_eular_angle[0];

Matrix<12, 6> p;

void vTaskChassisInit(void *pvParameters)
{
    left_leg.setLegLenth(0.14f, 0.28f, 0.28f, 0.14f, 0.13f);
    right_leg.setLegLenth(0.14f, 0.28f, 0.28f, 0.14f, 0.13f);
    // clang-format off
    p <<
    0.092421127435565,  -0.123637434003352,   0.063828883120901,  -0.014569739115445,   0.000522186366159,  -0.000031959576001,
    0.007653860987913,  -0.011477587164183,   0.006930293566631,  -0.002039331171159,   0.000125397037978,  -0.000005724502031,
    0.002048847639963,  -0.002465234365771,   0.001092496308452,  -0.000193457004102,   0.000001527765074,   0.000000569304292,
    0.010026752238566,  -0.012089908545735,   0.005378096645159,  -0.000961140155001,   0.000009240386030,   0.000002499805670,
    0.021745057079129,  -0.034620149902096,   0.021619020636883,  -0.006343299854552,   0.000691706420529,   0.000045719545229,
   -0.001744659739039,   0.001052895739356,   0.000292368002955,  -0.000337845214681,   0.000062529316459,   0.000007349343419,
   -0.349838501685620,   0.180525289830054,   0.117700581181001,  -0.110515895810080,   0.026771576723557,  -0.000454169834805,
   -0.143354776532384,   0.165599499260478,  -0.066158292913877,   0.007740252697635,   0.001097778856079,   0.000000872833619,
    0.013752781643839,  -0.021895705325150,   0.013673069198836,  -0.004011855084337,   0.000437473552205,   0.000028915579302,
    0.063985427944676,  -0.102881749386318,   0.064807456167882,  -0.019167488567388,   0.002107017154984,   0.000140702401135,
   -1.295805024200559,   1.559151112405602,  -0.690955334013425,   0.122352952456635,  -0.000966243472657,  -0.000360059648857,
   -0.165878083161510,   0.210108864810327,  -0.100708201126330,   0.020844678544707,  -0.000982184172355,  -0.000143870304864;
    p = p * 1e5;
    // clang-format on
    left_leg.setLinearzationParam(p);
    right_leg.setLinearzationParam(p);
    left_leg.setLegLenthController(pidController(0.0, 0., 0., 160, -160));
    right_leg.setLegLenthController(pidController(0.0, 0., 0., 160, -160));

    while (1) {
        ulTaskNotifyTake(pdTRUE, portMAX_DELAY);
        osDelay(3000); // 等待关节电机初始化完成

        left_leg.init();
        right_leg.init();

        xTaskNotifyGive(legStateUpdateHandle);
        xTaskNotifyGive(legControllHandle);
    }

    vTaskDelete(legInitHandle);
}

void vTaskChassisDeinit(void *pvParameters)
{
    while (1) {
        ulTaskNotifyTake(pdTRUE, portMAX_DELAY);
        left_leg.deInit();
        right_leg.deInit();
    }

    vTaskDelete(legDeinitTaskHandle);
}

void vTaskChassisStateUpdate(void *pvParameters)
{
    ulTaskNotifyTake(pdTRUE, portMAX_DELAY);

    float chassis_pitch, chassis_pitch_dot;

    TickType_t xLastWakeTime = xTaskGetTickCount();
    TickType_t xFrequency = pdMS_TO_TICKS(2);
    while (1) {
        chassis_pitch_dot = chassis_gyro[1];
        chassis_pitch = -chassis_eular_angle[1];

        left_leg.calculateCurrentState();
        right_leg.calculateCurrentState();
        auto half_speed = (left_leg.getCurrentState().x_dot +
                           right_leg.getCurrentState().x_dot) /
                          2.0f;
        left_leg.getCurrentState().x_dot = half_speed;
        right_leg.getCurrentState().x_dot = half_speed;

        left_leg.getCurrentState().phi = chassis_pitch;
        left_leg.getCurrentState().phi_dot = chassis_pitch_dot;
        right_leg.getCurrentState().phi = chassis_pitch;
        right_leg.getCurrentState().phi_dot = chassis_pitch_dot;

        vTaskDelayUntil(&xLastWakeTime, xFrequency);
    }
}

pidController theta_controller(0.0, 0.0, 0.0, 30, -30);
pidController yaw_compensation(1, 0.0, 0.01, 10, -10);

float chassis_omega, chassis_target_x;

void vTaskChassisControl(void *pvParameters)
{
    ulTaskNotifyTake(pdTRUE, portMAX_DELAY);

    float kroll = 0.0f;
    float chassis_roll, chassis_yaw_dot = 0;

    TickType_t xLastWakeTime = xTaskGetTickCount();
    TickType_t xFrequency = pdMS_TO_TICKS(4);
    while (1) {
        chassis_yaw_dot = 0.6 * chassis_gyro[2] + 0.4 * chassis_yaw_dot;
        chassis_roll = chassis_eular_angle[0];

        float omega = 0, x = 0;
        x = chassis_target_x;
        omega = -chassis_omega;

        // auto utheta = theta_controller.calculate(
        //     0.0f, right_leg.getCurrentState().theta -
        //               left_leg.getCurrentState().theta);
        auto uyaw = yaw_compensation.calculate(omega, chassis_yaw_dot);
        auto utheta = 0.0f;
        // auto uyaw = 0.0f;

        left_leg.setRollCompensation(kroll * chassis_roll)
            .setThetaCompensation(utheta)
            .setYawCompensation(-uyaw / 2);
        right_leg.setRollCompensation(-kroll * chassis_roll)
            .setThetaCompensation(-utheta)
            .setYawCompensation(uyaw / 2);

        left_leg.getTargetState().x = x;
        right_leg.getTargetState().x = x;
        left_leg.getTargetState().phi = -0.035f;
        right_leg.getTargetState().phi = -0.035f;

        left_leg.calculateTotalTorque();
        right_leg.calculateTotalTorque();

        vTaskDelayUntil(&xLastWakeTime, xFrequency);
    }

    vTaskDelete(legControllHandle);
}