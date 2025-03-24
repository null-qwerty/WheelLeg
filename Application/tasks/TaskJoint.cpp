#include "BaseControl/Motor/UnitreeA1protocol.hpp"

#include "tasks.hpp"

void vTaskJointInit(void *pvParameters)
{
    auto r_sendframe = (UART::xUARTFrame_t *)(rLegConnectivity.getSendFrame());
    r_sendframe->length = sizeof(sendData);
    r_sendframe->data[0] = (uint8_t *)pvPortMalloc(r_sendframe->length);
    r_sendframe->data[1] = (uint8_t *)pvPortMalloc(r_sendframe->length);
    auto r_receiveframe =
        (UART::xUARTFrame_t *)(rLegConnectivity.getReceiveFrame());
    r_receiveframe->length = sizeof(receiveData);
    r_receiveframe->data[0] = (uint8_t *)pvPortMalloc(r_receiveframe->length);
    r_receiveframe->data[1] = (uint8_t *)pvPortMalloc(r_receiveframe->length);

    auto l_sendframe = (UART::xUARTFrame_t *)(lLegConnectivity.getSendFrame());
    l_sendframe->length = sizeof(sendData);
    l_sendframe->data[0] = (uint8_t *)pvPortMalloc(l_sendframe->length);
    l_sendframe->data[1] = (uint8_t *)pvPortMalloc(l_sendframe->length);
    auto l_receiveframe =
        (UART::xUARTFrame_t *)(lLegConnectivity.getReceiveFrame());
    l_receiveframe->length = sizeof(receiveData);
    l_receiveframe->data[0] = (uint8_t *)pvPortMalloc(l_receiveframe->length);
    l_receiveframe->data[1] = (uint8_t *)pvPortMalloc(l_receiveframe->length);

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

        __HAL_UART_ENABLE_IT(&huart2, UART_IT_IDLE);
        __HAL_UART_ENABLE_IT(&huart3, UART_IT_IDLE);

        xTaskNotifyGive(lJointTransmitTaskHandle);
        xTaskNotifyGive(rJointTransmitTaskHandle);
    }
    vPortFree(r_sendframe->data[0]);
    vPortFree(r_sendframe->data[1]);
    vPortFree(r_receiveframe->data[0]);
    vPortFree(r_receiveframe->data[1]);
    vPortFree(l_sendframe->data[0]);
    vPortFree(l_sendframe->data[1]);
    vPortFree(l_receiveframe->data[0]);
    vPortFree(l_receiveframe->data[1]);
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
        // lLegConnectivity.receiveMessage();
        ulTaskNotifyTake(pdTRUE, 2);
        lfJoint.decodeFeedbackMessage();

        // vTaskDelayUntil(&xLastWakeTime, xFrequency);

        lbJoint.encodeControlMessage();
        HAL_GPIO_WritePin(USART2_DE_GPIO_Port, USART2_DE_Pin, GPIO_PIN_SET);
        lLegConnectivity.sendMessage();
        HAL_GPIO_WritePin(USART2_DE_GPIO_Port, USART2_DE_Pin, GPIO_PIN_RESET);
        // lLegConnectivity.receiveMessage();
        ulTaskNotifyTake(pdTRUE, 2);
        lbJoint.decodeFeedbackMessage();

        // vTaskDelayUntil(&xLastWakeTime, xFrequency);
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
        // rLegConnectivity.receiveMessage();
        ulTaskNotifyTake(pdTRUE, 2);
        rfJoint.decodeFeedbackMessage();

        // vTaskDelayUntil(&xLastWakeTime, xFrequency);

        rbJoint.encodeControlMessage();
        HAL_GPIO_WritePin(USART3_DE_GPIO_Port, USART3_DE_Pin, GPIO_PIN_SET);
        rLegConnectivity.sendMessage();
        HAL_GPIO_WritePin(USART3_DE_GPIO_Port, USART3_DE_Pin, GPIO_PIN_RESET);
        // rLegConnectivity.receiveMessage();
        ulTaskNotifyTake(pdTRUE, 2);
        rbJoint.decodeFeedbackMessage();

        // vTaskDelayUntil(&xLastWakeTime, xFrequency);
    }
    vTaskDelete(rJointTransmitTaskHandle);
}

#include "stm32h7xx_it.h"

/**
 * @brief This function handles USART2 global interrupt.
 */
void USART2_IRQHandler(void)
{
    /* USER CODE BEGIN USART2_IRQn 0 */
    if (__HAL_UART_GET_FLAG(&huart2, UART_FLAG_IDLE) != RESET) {
        __HAL_UART_CLEAR_IDLEFLAG(&huart2);
        HAL_UART_DMAStop(&huart2);

        auto length =
            sizeof(receiveData) - __HAL_DMA_GET_COUNTER(huart2.hdmarx);

        if (length == sizeof(receiveData)) {
            BaseType_t xHigherPriorityTaskWoken = pdFALSE;

            ((UART::xUARTFrame_t *)(rLegConnectivity.getReceiveFrame()))
                ->length = sizeof(receiveData);

            vTaskNotifyGiveFromISR(lJointTransmitTaskHandle,
                                   &xHigherPriorityTaskWoken);
        } else {
            ((UART::xUARTFrame_t *)(rLegConnectivity.getReceiveFrame()))
                ->length = sizeof(receiveData) - length;
        }
        rLegConnectivity.receiveMessage();
    }
    /* USER CODE END USART2_IRQn 0 */
    HAL_UART_IRQHandler(&huart2);
    /* USER CODE BEGIN USART2_IRQn 1 */
    portYIELD_FROM_ISR(pdFALSE);
    /* USER CODE END USART2_IRQn 1 */
}

/**
 * @brief This function handles USART3 global interrupt.
 */
void USART3_IRQHandler(void)
{
    /* USER CODE BEGIN USART3_IRQn 0 */
    if (__HAL_UART_GET_FLAG(&huart3, UART_FLAG_IDLE) != RESET) {
        __HAL_UART_CLEAR_IDLEFLAG(&huart3);
        HAL_UART_DMAStop(&huart3);

        auto length =
            sizeof(receiveData) - __HAL_DMA_GET_COUNTER(huart3.hdmarx);

        if (length == sizeof(receiveData)) {
            BaseType_t xHigherPriorityTaskWoken = pdFALSE;

            ((UART::xUARTFrame_t *)(lLegConnectivity.getReceiveFrame()))
                ->length = sizeof(receiveData);

            vTaskNotifyGiveFromISR(lJointTransmitTaskHandle,
                                   &xHigherPriorityTaskWoken);
        } else {
            ((UART::xUARTFrame_t *)(lLegConnectivity.getReceiveFrame()))
                ->length = sizeof(receiveData) - length;
        }
        lLegConnectivity.receiveMessage();
    }
    /* USER CODE END USART3_IRQn 0 */
    HAL_UART_IRQHandler(&huart3);
    /* USER CODE BEGIN USART3_IRQn 1 */
    portYIELD_FROM_ISR(pdFALSE);
    /* USER CODE END USART3_IRQn 1 */
}

xTaskHandle jointInitTaskHandle;
xTaskHandle lJointTransmitTaskHandle;
xTaskHandle rJointTransmitTaskHandle;

UART lLegConnectivity(&huart2, UART::dmaOption::RX);
UART rLegConnectivity(&huart3, UART::dmaOption::RX);

UnitreeA1 lfJoint(lLegConnectivity, 0, 0,
                  LEFT_MOTOR_CLOCKWISE *FRONT_MOTOR_CLOCKWISE);
UnitreeA1 lbJoint(lLegConnectivity, 1, 1,
                  LEFT_MOTOR_CLOCKWISE *BACK_MOTOR_CLOCKWISE);
UnitreeA1 rfJoint(rLegConnectivity, 0, 0,
                  RIGHT_MOTOR_CLOCKWISE *FRONT_MOTOR_CLOCKWISE);
UnitreeA1 rbJoint(rLegConnectivity, 1, 1,
                  RIGHT_MOTOR_CLOCKWISE *BACK_MOTOR_CLOCKWISE);