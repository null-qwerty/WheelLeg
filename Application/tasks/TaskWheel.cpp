#include "tasks.hpp"

#include "BaseControl/Motor/RM3508.hpp"
#include "BaseControl/Controller/pidController.hpp"

void vTaskWheelReceive(void *pvParameters)
{
    wheelConnectivity.init();
    while (1) {
        // 接收 CAN2 中断发出的信号量
        // xClearCountOnExit 清空信号量数量
        // xTicksToWait 超时等待
        ulTaskNotifyTake(pdTRUE, portMAX_DELAY);
        // 解析 CAN 消息帧
        // clang-format off
        if (((FDCAN::xReceptionFrame_t*)wheelConnectivity.getReceiveFrame())->header.Identifier == leftWheel.getReceiveId()) {
            leftWheel.decodeFeedbackMessage();
        } else if (((FDCAN::xReceptionFrame_t*)wheelConnectivity.getReceiveFrame())->header.Identifier == rightWheel.getReceiveId()) {
            rightWheel.decodeFeedbackMessage();
        }
        // 向轮毂控制任务发出信号量，任务之间使用 xTaskNotifyGive()
        xTaskNotifyGive(wheelControlTaskHandle);
        // clang-format on
    }

    vTaskDelete(wheelReceiveTaskHandle);
}

pidController lwslc(20., 0.05, 0.7, 14000, -14000);
pidController rwslc(20., 0.05, 0.7, 14000, -14000);

void vTaskWheelControl(void *pvParameters)
{
    leftWheel.init();
    rightWheel.init();
    leftWheel.getSpeedLoopController() = &lwslc;
    rightWheel.getSpeedLoopController() = &rwslc;

    while (1) {
        // 接收从其他任务发出的信号量
        ulTaskNotifyTake(pdTRUE, portMAX_DELAY);

        if (xSemaphoreTake(wheelControlMutex, 1)) {
            leftWheel.encodeControlMessage();
            rightWheel.encodeControlMessage();
            xSemaphoreGive(wheelControlMutex);
        }

        // 发送控制信息，若发送失败则重新初始化 CAN 线
        if (wheelConnectivity.sendMessage() != HAL_OK)
            wheelConnectivity.init();
    }

    vTaskDelete(wheelControlTaskHandle);
}

void HAL_FDCAN_RxFifo0Callback(FDCAN_HandleTypeDef *hfdcan, uint32_t RxFifo0ITs)
{
    // * 需要在中断中调用 HAL_CAN_GetRxMessage()，实测如果把
    // * 调用放到 notify 的任务中无法正常接收消息，还需调试
    wheelConnectivity.receiveMessage();
    // 判断接收的消息是否合法
    // clang-format off
    if (((FDCAN::xReceptionFrame_t*)(wheelConnectivity.getReceiveFrame()))->header.IdType != FDCAN_STANDARD_ID ||
        ((FDCAN::xReceptionFrame_t*)(wheelConnectivity.getReceiveFrame()))->header.DataLength != 8) {
        return;
    }
    // clang-format on
    // 高优先级优先，上下文切换时优先执行高优先级任务
    BaseType_t xHigherPriorityTaskWoken = pdTRUE;
    // 发送二进制信号量通知 wheelReceiveTask
    // 中断中需要调用 vTaskNotifyGiveFromISR()
    vTaskNotifyGiveFromISR(wheelReceiveTaskHandle, &xHigherPriorityTaskWoken);
    // 切换上下文
    portYIELD_FROM_ISR(xHigherPriorityTaskWoken);

    return;
}

xTaskHandle wheelReceiveTaskHandle;
xTaskHandle wheelControlTaskHandle;

FDCAN_FilterTypeDef wheelFdcanFilter = { .IdType = FDCAN_STANDARD_ID,
                                         .FilterIndex = 0,
                                         .FilterType = FDCAN_FILTER_RANGE,
                                         .FilterConfig =
                                             FDCAN_FILTER_TO_RXFIFO0,
                                         .FilterID1 = 0x000,
                                         .FilterID2 = 0x7ff,
                                         .RxBufferIndex = 0,
                                         .IsCalibrationMsg = 0 };

FDCAN wheelConnectivity = FDCAN(&hfdcan1, wheelFdcanFilter, 0);

// pid 调参, 使用的 TI 的开源 pid，计算方式见 pidController::Calculate()
// 算法的微分部分似乎会造成高频震荡且微分输出值存在爆炸的风险，需查阅资料（解决，参考pidController::Calculate()）
RM3508 leftWheel = RM3508(wheelConnectivity, 4, 0x204, LEFT_MOTOR_CLOCKWISE);
RM3508 rightWheel = RM3508(wheelConnectivity, 1, 0x201, RIGHT_MOTOR_CLOCKWISE);