#include "BaseControl/Connectivity/Connectivity.hpp"
#include "BaseControl/Motor/Motor.hpp"
#include "BaseControl/Motor/UnitreeA1protocol.hpp"

#include "Math/Trigonometric.hpp"
#include "cmsis_os2.h"
#include "projdefs.h"
#include "tasks.hpp"
#include "usart.h"

void vTaskJointInit(void *pvParameters)
{
    // 使用 FreeRTOS 函数来动态分配内存

    auto initfunction = [&](Connectivity &connectivity) {
        auto sendframe = (UART::xUARTFrame_t *)(connectivity.getSendFrame());
        auto receiveframe =
            (UART::xUARTFrame_t *)(connectivity.getReceiveFrame());
        sendframe->length = sizeof(sendData);
        receiveframe->length = sizeof(receiveData);

        sendframe->data[0] = (uint8_t *)pvPortMalloc(sendframe->length);
        sendframe->data[1] = (uint8_t *)pvPortMalloc(sendframe->length);
        receiveframe->data[0] = (uint8_t *)pvPortMalloc(receiveframe->length);
        receiveframe->data[1] = (uint8_t *)pvPortMalloc(receiveframe->length);

        memset(sendframe->data[0], 0, sendframe->length);
        memset(sendframe->data[1], 0, sendframe->length);
        memset(receiveframe->data[0], 0, receiveframe->length);
        memset(receiveframe->data[1], 0, receiveframe->length);

        connectivity.receiveMessage();
    };

    initfunction(lLegConnectivity);
    initfunction(rLegConnectivity);

    while (true) {
        ulTaskNotifyTake(pdTRUE, portMAX_DELAY);
        lfJoint.init();
        // lLegConnectivity.sendMessage();
        osDelay(5);
        lbJoint.init();
        // lLegConnectivity.sendMessage();
        osDelay(5);
        rfJoint.init();
        // rLegConnectivity.sendMessage();
        osDelay(5);
        rbJoint.init();
        // rLegConnectivity.sendMessage();
        osDelay(5);

        xTaskNotifyGive(lJointTransmitTaskHandle);
        xTaskNotifyGive(rJointTransmitTaskHandle);
    }
    // 释放内存
    auto deinitfunction = [&](Connectivity &connectivity) {
        auto sendframe = (UART::xUARTFrame_t *)(connectivity.getSendFrame());
        auto receiveframe =
            (UART::xUARTFrame_t *)(connectivity.getReceiveFrame());
        vPortFree(sendframe->data[0]);
        vPortFree(sendframe->data[1]);
        vPortFree(receiveframe->data[0]);
        vPortFree(receiveframe->data[1]);
    };
    deinitfunction(lLegConnectivity);
    deinitfunction(rLegConnectivity);
    vTaskDelete(jointInitTaskHandle);
}

void vTaskLeftJointTransmit(void *pvParameters)
{
    ulTaskNotifyTake(pdTRUE, portMAX_DELAY);

    TickType_t xLastWakeTime = xTaskGetTickCount();
    TickType_t xFrequency = pdMS_TO_TICKS(4);

    while (true) {
        lfJoint.encodeControlMessage();
        lLegConnectivity.sendMessage();
        // 由于 A1 电机是一问一答的，所以这里需要等待反馈数据
        ulTaskNotifyTake(pdTRUE, 1);

        lbJoint.encodeControlMessage();
        lLegConnectivity.sendMessage();
        ulTaskNotifyTake(pdTRUE, 1);
        // 任务定频执行
        //! 不一定能够定频，因为与中断有关。这里应该给一个宽松的时间，保证长于上面等待信号量的时间。
        //* 4.8M 波特率，一次发送 34 字节，接收 78 字节，1 停止位无校验位：
        //* 1 / 4800000 * 10 * (78 + 34) = 0.000233s = 0.233ms
        //* 再考虑其他函数的执行时间，2ms 应该够用。
        //* 右侧电机的发送任务也是一样的。
        vTaskDelayUntil(&xLastWakeTime, xFrequency);
    }
    vTaskDelete(lJointTransmitTaskHandle);
}

void vTaskRightJointTransmit(void *pvParameters)
{
    ulTaskNotifyTake(pdTRUE, portMAX_DELAY);

    TickType_t xLastWakeTime = xTaskGetTickCount();
    TickType_t xFrequency = pdMS_TO_TICKS(4);

    while (true) {
        rfJoint.encodeControlMessage();
        rLegConnectivity.sendMessage();
        ulTaskNotifyTake(pdTRUE, 1);

        rbJoint.encodeControlMessage();
        rLegConnectivity.sendMessage();
        ulTaskNotifyTake(pdTRUE, 1);

        vTaskDelayUntil(&xLastWakeTime, xFrequency);
    }
    vTaskDelete(rJointTransmitTaskHandle);
}

void HAL_UARTEx_RxEventCallback(UART_HandleTypeDef *huart, uint16_t Size)
{
    if (huart == &huart2 && Size == sizeof(receiveData)) {
        if (huart->hdmarx->Init.Mode == DMA_CIRCULAR) {
            HAL_UART_DMAStop(huart);
        }
        // 解析反馈数据，函数内部会进行筛选，所以都调用一次
        lfJoint.decodeFeedbackMessage();
        lbJoint.decodeFeedbackMessage();

        BaseType_t xHigherPriorityTaskWoken = pdTRUE;
        // 解析完后通知任务，发送下一个电机的控制数据
        vTaskNotifyGiveFromISR(lJointTransmitTaskHandle,
                               &xHigherPriorityTaskWoken);
        // 开启接收下一次数据
        lLegConnectivity.receiveMessage();
    } else if (huart == &huart3 && Size == sizeof(receiveData)) {
        if (huart->hdmarx->Init.Mode == DMA_CIRCULAR) {
            HAL_UART_DMAStop(huart);
        }

        rfJoint.decodeFeedbackMessage();
        rbJoint.decodeFeedbackMessage();

        BaseType_t xHigherPriorityTaskWoken = pdTRUE;

        vTaskNotifyGiveFromISR(rJointTransmitTaskHandle,
                               &xHigherPriorityTaskWoken);

        rLegConnectivity.receiveMessage();
    }
    // 切换上下文，如果有更高优先级的任务需要执行，就立即执行
    portYIELD_FROM_ISR(pdTRUE);
}

xTaskHandle jointInitTaskHandle;
xTaskHandle lJointTransmitTaskHandle;
xTaskHandle rJointTransmitTaskHandle;

UART lLegConnectivity(&huart2, UART::dmaOption::RX);
UART rLegConnectivity(&huart3, UART::dmaOption::RX);

uint8_t jointOption = Motor::MotorOption::MOTOR_SOFT_LIMIT |
                      Motor::MotorOption::MOTOR_SOFT_ZERO;
Motor::MotorOptionData jointOptionData = {
    .soft_limit_min = DEGREE_TO_RAND(-15.0f),
    .soft_limit_max = DEGREE_TO_RAND(45.0f),
    .soft_zero = 0.0f,
};

UnitreeA1 lfJoint(lLegConnectivity, 0, 0,
                  LEFT_MOTOR_CLOCKWISE *FRONT_MOTOR_CLOCKWISE, 9.1f,
                  jointOption, jointOptionData);
UnitreeA1 lbJoint(lLegConnectivity, 1, 1,
                  LEFT_MOTOR_CLOCKWISE *BACK_MOTOR_CLOCKWISE, 9.1f, jointOption,
                  jointOptionData);
UnitreeA1 rfJoint(rLegConnectivity, 0, 0,
                  RIGHT_MOTOR_CLOCKWISE *FRONT_MOTOR_CLOCKWISE, 9.1f,
                  jointOption, jointOptionData);
UnitreeA1 rbJoint(rLegConnectivity, 1, 1,
                  RIGHT_MOTOR_CLOCKWISE *BACK_MOTOR_CLOCKWISE, 9.1f,
                  jointOption, jointOptionData);