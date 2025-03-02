#pragma once

#include "main.h"
#include "semphr.h"

#include "BaseControl/Connectivity/UART/DBUS.hpp"
#include "BaseControl/Motor/RM3508.hpp"

void WheelLegTasksInit(void);

/**
 * @brief 读取遥控器的数据
 *
 * @param pvParameters 任务参数，未使用
 *
 * @note 以 200 Hz 的频率读取 DBUS 数据 (osPriorityAboveNormal)
 */
__weak void vTaskReadDbus(void *pvParameters);

/**
 * @brief 接收轮毂的电机的反馈信号
 *
 * @param pvParameters 任务参数，未使用
 *
 * @note 以 1000Hz 的频率接收电机的反馈信号，将反馈值存入全局变量中
 * (osPriorityAboveNormal)
 */
__weak void vTaskWheelReceive(void *pvParameters);

/**
 * @brief 向轮毂的电机发送控制信号
 *
 * @param pvParameters 任务参数，未使用
 *
 * @note 通过 notify 执行 (osPriorityAboveNormal)
 */
__weak void vTaskWheelControl(void *pvParameters);

extern xTaskHandle readDbusTaskHandle;

extern xTaskHandle wheelReceiveTaskHandle;
extern xTaskHandle wheelControlTaskHandle;

extern SemaphoreHandle_t wheelControlMutex;

extern DBUS dbus;

extern FDCAN_FilterTypeDef wheelFdcanFilter;
extern FDCAN wheelConnectivity;
extern RM3508 leftWheel, rightWheel;
