#include "cmsis_os.h"
#include "fast_math_functions.h"
#include "tasks.hpp"

void vTaskReadDbus(void *pvParameters)
{
    dbus.init();

    bool jointInited = false;

    TickType_t xLastWakeTime;
    TickType_t xFrequency = pdMS_TO_TICKS(1);
    xLastWakeTime = xTaskGetTickCount();
    while (1) {
        // 读取遥控器数据
        if (dbus.receiveMessage() == HAL_OK) {
            dbus.decodeDBUSMessage();
        }
        // 控制轮毂电机速度
        if (xSemaphoreTake(wheelControlMutex, 1)) {
            // // 前进
            // leftWheel.getTargetState().velocity =
            //     (dbus.getDBUSData().rc.ch1 - 1024) / 660.0 * 360;
            // rightWheel.getTargetState().velocity =
            //     leftWheel.getTargetState().velocity;
            // // 转向
            // leftWheel.getTargetState().velocity +=
            //     (dbus.getDBUSData().rc.ch0 - 1024) / 660.0 * 80;
            // rightWheel.getTargetState().velocity -=
            //     (dbus.getDBUSData().rc.ch0 - 1024) / 660.0 * 80;
            chassis_target_x =
                1.0f * (dbus.getDBUSData().rc.ch1 - 1024) / 660.0f * 0.001;
            chassis_omega =
                1.0f * (dbus.getDBUSData().rc.ch0 - 1024) / 660.0f * PI / 10.0f;
            xSemaphoreGive(wheelControlMutex);
        }

        // 初始化关节电机
        if (!jointInited && dbus.getDBUSData().rc.s2 == 3) {
            jointInited = true;
            xTaskNotifyGive(jointInitTaskHandle);
        }
        if (jointInited && dbus.getDBUSData().rc.s2 == 1) {
            jointInited = false;
            xTaskNotifyGive(legDeinitTaskHandle);
        }

        vTaskDelayUntil(&xLastWakeTime, xFrequency);
    }

    vTaskDelete(readDbusTaskHandle);
}

xTaskHandle readDbusTaskHandle;

SemaphoreHandle_t wheelControlMutex;

DBUS dbus(&huart5, DBUS::dmaOption::RX);
