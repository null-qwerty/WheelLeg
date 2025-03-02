#include "tasks.hpp"

void vTaskReadDbus(void *pvParameters)
{
    dbus.init();
    TickType_t xLastWakeTime;
    TickType_t xFrequency = pdMS_TO_TICKS(2);
    xLastWakeTime = xTaskGetTickCount();
    while (1) {
        // 读取遥控器数据
        if (dbus.receiveMessage() == HAL_OK) {
            dbus.decodeDBUSMessage();
        }

        vTaskDelayUntil(&xLastWakeTime, xFrequency);
    }

    vTaskDelete(readDbusTaskHandle);
}

xTaskHandle readDbusTaskHandle;

SemaphoreHandle_t wheelControlMutex;

DBUS dbus(&huart5, DBUS::dmaOption::RX);
