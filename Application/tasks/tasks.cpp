#include "cmsis_os2.h"

#include "tasks.hpp"

void WheelLegTasksInit(void)
{
    /* 创建任务 */
    xTaskCreate(vTaskReadDbus, "readDbusTask", 128 * 2, NULL,
                osPriorityAboveNormal, &readDbusTaskHandle);
    xTaskCreate(vTaskWheelReceive, "wheelReceiveTask", 128 * 2, NULL,
                osPriorityAboveNormal, &wheelReceiveTaskHandle);
    xTaskCreate(vTaskWheelControl, "wheelControlTask", 128 * 2, NULL,
                osPriorityAboveNormal, &wheelControlTaskHandle);
    return;
}