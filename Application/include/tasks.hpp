#pragma once

#include "main.h"
#include "FreeRTOS.h"
#include "cmsis_os2.h"
#include "task.h"
#include "semphr.h"

#include "BaseControl/Connectivity/UART/DBUS.hpp"

void WheelLegTasksInit(void);

/**
 * @brief 读取遥控器的数据
 *
 * @param pvParameters 任务参数，未使用
 *
 * @note 以 200 Hz 的频率读取 DBUS 数据 (osPriorityAboveNormal)
 */
__weak void vTaskReadDbus(void *pvParameters);

extern xTaskHandle readDbusTaskHandle;

extern SemaphoreHandle_t wheelControlMutex;

extern DBUS dbus;