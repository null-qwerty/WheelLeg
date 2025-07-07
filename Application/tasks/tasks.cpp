#include "cmsis_os2.h"

#include "tasks.hpp"

void WheelLegTasksInit(void)
{
    /* 创建任务 */
    xTaskCreate(vTaskLed, "led", 128 * 2, NULL, osPriorityNormal,
                &ledTaskHandle);
    xTaskCreate(vTaskReadDbus, "readDbus", 128 * 2, NULL, osPriorityAboveNormal,
                &readDbusTaskHandle);
    xTaskCreate(vTaskWheelReceive, "wheelReceive", 128 * 2, NULL,
                osPriorityHigh, &wheelReceiveTaskHandle);
    xTaskCreate(vTaskWheelControl, "wheelControl", 128 * 2, NULL,
                osPriorityHigh, &wheelControlTaskHandle);
    xTaskCreate(vTaskJointInit, "jointInit", 128 * 2, NULL,
                osPriorityAboveNormal, &jointInitTaskHandle);
    xTaskCreate(vTaskLeftJointTransmit, "ljointTransmit", 128 * 2, NULL,
                osPriorityHigh, &lJointTransmitTaskHandle);
    xTaskCreate(vTaskRightJointTransmit, "rjointTransmit", 128 * 2, NULL,
                osPriorityHigh, &rJointTransmitTaskHandle);
    xTaskCreate(vTaskLeftJointEncode, "lJointEncode", 128 * 2, NULL,
                osPriorityHigh, &lJointEncodeHandle);
    xTaskCreate(vTaskRightJointEncode, "rJointEncode", 128 * 2, NULL,
                osPriorityHigh, &rJointEncodeHandle);
    xTaskCreate(vTaskChassisInit, "ChassisInit", 128 * 4, NULL,
                osPriorityAboveNormal, &legInitHandle);
    xTaskCreate(vTaskChassisControl, "ChassisControl", 128 * 2, NULL,
                osPriorityHigh, &legControllHandle);
    xTaskCreate(vTaskChassisDeinit, "ChassisDeinit", 128, NULL,
                osPriorityAboveNormal, &legDeinitTaskHandle);
    xTaskCreate(vTaskChassisStateUpdate, "ChassisStateUpdate", 128 * 2, NULL,
                osPriorityHigh, &legStateUpdateHandle);
    xTaskCreate(vTaskImu, "Imu", 128 * 2, NULL, osPriorityHigh, &imuTaskHandle);
    xTaskCreate(vTaskImuTempHold, "imuTempHold", 128, NULL,
                osPriorityAboveNormal, &imuTempHoldHandle);
    return;
}