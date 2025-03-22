#include "cmsis_os2.h"

#include "tasks.hpp"

void WheelLegTasksInit(void)
{
    /* 创建任务 */
    xTaskCreate(vTaskLed, "ledTask", 128 * 2, NULL, osPriorityNormal,
                &ledTaskHandle);
    xTaskCreate(vTaskReadDbus, "readDbusTask", 128 * 2, NULL,
                osPriorityAboveNormal, &readDbusTaskHandle);
    xTaskCreate(vTaskWheelReceive, "wheelReceiveTask", 128 * 2, NULL,
                osPriorityAboveNormal, &wheelReceiveTaskHandle);
    xTaskCreate(vTaskWheelControl, "wheelControlTask", 128 * 2, NULL,
                osPriorityAboveNormal, &wheelControlTaskHandle);
    xTaskCreate(vTaskJointInit, "jointInitTask", 128 * 2, NULL,
                osPriorityAboveNormal, &jointInitTaskHandle);
    xTaskCreate(vTaskLeftJointTransmit, "jointTransmitTask", 128 * 2, NULL,
                osPriorityAboveNormal, &lJointTransmitTaskHandle);
    xTaskCreate(vTaskRightJointTransmit, "jointTransmitTask", 128 * 2, NULL,
                osPriorityAboveNormal, &rJointTransmitTaskHandle);
    return;
}