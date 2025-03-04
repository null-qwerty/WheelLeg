#include "tasks.hpp"

void vTaskReadDbus(void *pvParameters)
{
    dbus.init();
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
            // 前进
            leftWheel.getTargetState().velocity =
                (dbus.getDBUSData().rc.ch1 - 1024) / 660.0 * 3600;
            rightWheel.getTargetState().velocity =
                leftWheel.getTargetState().velocity;
            // 转向
            leftWheel.getTargetState().velocity +=
                (dbus.getDBUSData().rc.ch0 - 1024) / 660.0 * 800;
            rightWheel.getTargetState().velocity -=
                (dbus.getDBUSData().rc.ch0 - 1024) / 660.0 * 800;

            xSemaphoreGive(wheelControlMutex);
        }

        vTaskDelayUntil(&xLastWakeTime, xFrequency);
    }

    vTaskDelete(readDbusTaskHandle);
}

xTaskHandle readDbusTaskHandle;

SemaphoreHandle_t wheelControlMutex;

DBUS dbus(&huart5, DBUS::dmaOption::RX);
