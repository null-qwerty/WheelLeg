#pragma once

#include "main.h"
#include "semphr.h"

#include "BaseControl/Connectivity/UART/DBUS.hpp"
#include "BaseControl/Motor/Motor.hpp"

void WheelLegTasksInit(void);

/**
 * @brief led 任务
 *
 * @param pvParameters 任务参数，未使用
 *
 * @note 闪烁 LED 表示系统正常运行 (osPriorityNormal)
 */
__weak void vTaskLed(void *pvParameters);

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

/**
 * @brief 初始化关节电机
 *
 * @param pvParameters 任务参数，未使用
 *
 * @note 通过 notify 执行，执行完毕后启动通信任务 (osPriorityAboveNormal)
 */
__weak void vTaskJointInit(void *pvParameters);

/**
 * @brief 给左边关节电机发送控制信号
 *
 * @param pvParameters 任务参数，未使用
 *
 * @note 被初始化任务唤醒后定频率执行 (osPriorityAboveNormal)
 */
__weak void vTaskLeftJointTransmit(void *pvParameters);

/**
 * @brief 给右边关节电机发送控制信号
 *
 * @param pvParameters 任务参数，未使用
 *
 * @note 被初始化任务唤醒后定频率执行 (osPriorityAboveNormal)
 */
__weak void vTaskRightJointTransmit(void *pvParameters);

extern xTaskHandle ledTaskHandle;

extern xTaskHandle readDbusTaskHandle;

extern xTaskHandle wheelReceiveTaskHandle;
extern xTaskHandle wheelControlTaskHandle;

extern xTaskHandle jointInitTaskHandle;
extern xTaskHandle lJointTransmitTaskHandle;
extern xTaskHandle rJointTransmitTaskHandle;

extern SemaphoreHandle_t wheelControlMutex;

extern DBUS dbus;

extern FDCAN_FilterTypeDef wheelFdcanFilter;
extern FDCAN wheelConnectivity;

extern RM3508 leftWheel, rightWheel;

extern UART lLegConnectivity, rLegConnectivity;

extern UnitreeA1 lfJoint, lbJoint, rfJoint, rbJoint;