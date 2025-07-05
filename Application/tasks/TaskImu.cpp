#include "BaseControl/Controller/pidController.hpp"
#include "BaseControl/Sensor/BMI088/BMI088.hpp"

#include "Filter/Mahony.hpp"
#include "Math/Quaternion.hpp"

#include "tasks.hpp"

Vector3f chassis_eular_angle;
Vector3f chassis_gyro, chassis_accel;

xTaskHandle imuTaskHandle;
xTaskHandle imuTempHoldHandle;

Mahony mahony(1000.0f);
SPI imuConnectivity(&hspi2, SPI::dmaOption::RX);
BMI088 imu(imuConnectivity);

BMI088::Data_t *imu_data;
float imu_target_temprature = 45;

void vTaskImu(void *pvParameters)
{
    TickType_t xLastWakeTime = xTaskGetTickCount();
    TickType_t xFrequency = pdMS_TO_TICKS(1);

    imu.init();

    xTaskNotifyGive(imuTempHoldHandle);

    while (1) {
        imu_data = (BMI088::Data_t *)imu.getData();
        if (fabsf(imu_data->temperature - imu_target_temprature) < 1.0f) {
            break;
        }
        vTaskDelayUntil(&xLastWakeTime, xFrequency);
    }

    mahony.init(1.0f);
    imu.reset();

    xLastWakeTime = xTaskGetTickCount();

    while (1) {
        imu_data = (BMI088::Data_t *)imu.getData();

        chassis_gyro[0] = imu_data->gyro.roll;
        chassis_gyro[1] = imu_data->gyro.pitch;
        chassis_gyro[2] = imu_data->gyro.yaw;
        chassis_accel[0] = imu_data->accel.x;
        chassis_accel[1] = imu_data->accel.y;
        chassis_accel[2] = imu_data->accel.z;

        chassis_eular_angle =
            mahony.update(chassis_gyro, chassis_accel).toEulerAngles();

        mahony.yawZeroDriftOffset(5.32213e-7);

        vTaskDelayUntil(&xLastWakeTime, xFrequency);
    }
}

float pwm;
pidController temp_hold_controller(100, 0., 0., 1000, -1000);

void vTaskImuTempHold(void *pvParameters)
{
    HAL_TIM_Base_Start(&htim3);
    HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_4);

    ulTaskNotifyTake(pdTRUE, portMAX_DELAY);

    TickType_t xLastWakeTime = xTaskGetTickCount();
    TickType_t xFrequency = pdMS_TO_TICKS(2);

    while (1) {
        pwm = temp_hold_controller.calculate(imu_target_temprature,
                                             imu_data->temperature);
        if (pwm < 0)
            pwm = 0;
        __HAL_TIM_SetCompare(&htim3, TIM_CHANNEL_4, pwm);

        vTaskDelayUntil(&xLastWakeTime, xFrequency);
    }
}