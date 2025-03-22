#include "BaseControl/Motor/UnitreeA1protocol.hpp"

#include "tasks.hpp"

void vTaskJointInit(void *pvParameters)
{
    ((UART::xUARTFrame_t *)(lLegConnectivity.getSendFrame()))->length =
        sizeof(sendData);
    ((UART::xUARTFrame_t *)(lLegConnectivity.getReceiveFrame()))->length =
        sizeof(receiveData);
    ((UART::xUARTFrame_t *)(rLegConnectivity.getSendFrame()))->length =
        sizeof(sendData);
    ((UART::xUARTFrame_t *)(rLegConnectivity.getReceiveFrame()))->length =
        sizeof(receiveData);

    lLegConnectivity.init();
    rLegConnectivity.init();

    while (true) {
        ulTaskNotifyTake(pdTRUE, portMAX_DELAY);
        HAL_GPIO_WritePin(USART2_DE_GPIO_Port, USART2_DE_Pin, GPIO_PIN_SET);
        lfJoint.init();
        lLegConnectivity.sendMessage();
        osDelay(1);
        lbJoint.init();
        lLegConnectivity.sendMessage();
        HAL_GPIO_WritePin(USART2_DE_GPIO_Port, USART2_DE_Pin, GPIO_PIN_RESET);

        HAL_GPIO_WritePin(USART3_DE_GPIO_Port, USART3_DE_Pin, GPIO_PIN_SET);
        rfJoint.init();
        rLegConnectivity.sendMessage();
        osDelay(1);
        rbJoint.init();
        rLegConnectivity.sendMessage();
        HAL_GPIO_WritePin(USART3_DE_GPIO_Port, USART3_DE_Pin, GPIO_PIN_RESET);

        xTaskNotifyGive(lJointTransmitTaskHandle);
        xTaskNotifyGive(rJointTransmitTaskHandle);
    }
    vTaskDelete(jointInitTaskHandle);
}

void vTaskLeftJointTransmit(void *pvParameters)
{
    ulTaskNotifyTake(pdTRUE, portMAX_DELAY);

    TickType_t xLastWakeTime = xTaskGetTickCount();
    TickType_t xFrequency = pdMS_TO_TICKS(1);

    while (true) {
        lfJoint.encodeControlMessage();
        HAL_GPIO_WritePin(USART2_DE_GPIO_Port, USART2_DE_Pin, GPIO_PIN_SET);
        lLegConnectivity.sendMessage();
        HAL_GPIO_WritePin(USART2_DE_GPIO_Port, USART2_DE_Pin, GPIO_PIN_RESET);
        lLegConnectivity.receiveMessage();
        lfJoint.decodeFeedbackMessage();

        vTaskDelayUntil(&xLastWakeTime, xFrequency);

        lbJoint.encodeControlMessage();
        HAL_GPIO_WritePin(USART2_DE_GPIO_Port, USART2_DE_Pin, GPIO_PIN_SET);
        lLegConnectivity.sendMessage();
        HAL_GPIO_WritePin(USART2_DE_GPIO_Port, USART2_DE_Pin, GPIO_PIN_RESET);
        lLegConnectivity.receiveMessage();
        lbJoint.decodeFeedbackMessage();

        vTaskDelayUntil(&xLastWakeTime, xFrequency);
    }
    vTaskDelete(lJointTransmitTaskHandle);
}

void vTaskRightJointTransmit(void *pvParameters)
{
    ulTaskNotifyTake(pdTRUE, portMAX_DELAY);

    TickType_t xLastWakeTime = xTaskGetTickCount();
    TickType_t xFrequency = pdMS_TO_TICKS(1);

    while (true) {
        rfJoint.encodeControlMessage();
        HAL_GPIO_WritePin(USART3_DE_GPIO_Port, USART3_DE_Pin, GPIO_PIN_SET);
        rLegConnectivity.sendMessage();
        HAL_GPIO_WritePin(USART3_DE_GPIO_Port, USART3_DE_Pin, GPIO_PIN_RESET);
        rLegConnectivity.receiveMessage();
        rfJoint.decodeFeedbackMessage();

        vTaskDelayUntil(&xLastWakeTime, xFrequency);

        rbJoint.encodeControlMessage();
        HAL_GPIO_WritePin(USART3_DE_GPIO_Port, USART3_DE_Pin, GPIO_PIN_SET);
        rLegConnectivity.sendMessage();
        HAL_GPIO_WritePin(USART3_DE_GPIO_Port, USART3_DE_Pin, GPIO_PIN_RESET);
        rLegConnectivity.receiveMessage();
        rbJoint.decodeFeedbackMessage();

        vTaskDelayUntil(&xLastWakeTime, xFrequency);
    }
    vTaskDelete(rJointTransmitTaskHandle);
}

xTaskHandle jointInitTaskHandle;
xTaskHandle lJointTransmitTaskHandle;
xTaskHandle rJointTransmitTaskHandle;

UART lLegConnectivity(&huart2, UART::dmaOption::DISABLE);
UART rLegConnectivity(&huart3, UART::dmaOption::DISABLE);

UnitreeA1 lfJoint(lLegConnectivity, 0, 0,
                  LEFT_MOTOR_CLOCKWISE *FRONT_MOTOR_CLOCKWISE);
UnitreeA1 lbJoint(lLegConnectivity, 1, 1,
                  LEFT_MOTOR_CLOCKWISE *BACK_MOTOR_CLOCKWISE);
UnitreeA1 rfJoint(rLegConnectivity, 0, 0,
                  RIGHT_MOTOR_CLOCKWISE *FRONT_MOTOR_CLOCKWISE);
UnitreeA1 rbJoint(rLegConnectivity, 1, 1,
                  RIGHT_MOTOR_CLOCKWISE *BACK_MOTOR_CLOCKWISE);