#include "tasks.hpp"

#include "BaseControl/Connectivity/SPI/SPI.hpp"

// 1us 发送 6bit，为了居中右移一位。
#define CODE_0 0x60 // 0110 0000，高低电平 1:2
#define CODE_1 0x78 // 0111 1000，高低电平 2:1

xTaskHandle ledTaskHandle;

SPI ledConnectivity(&hspi6, SPI::dmaOption::DISABLE);

uint8_t color[24];

void generateColor(uint8_t red, uint8_t green, uint8_t blue)
{
    uint8_t redIndex = 1, greenIndex = 0, blueIndex = 2;

    for (int i = 0; i < 8; i++) {
        color[8 * redIndex + 7 - i] = ((red >> i) & 1) ? CODE_1 : CODE_0;
        color[8 * greenIndex + 7 - i] = ((green >> i) & 1) ? CODE_1 : CODE_0;
        color[8 * blueIndex + 7 - i] = ((blue >> i) & 1) ? CODE_1 : CODE_0;
    }
}

void sendColor()
{
    static uint8_t reset = 0;

    ((SPI::xSPIFrame_t *)(ledConnectivity.getSendFrame()))->data = &reset;
    ((SPI::xSPIFrame_t *)(ledConnectivity.getSendFrame()))->length = 1;
    ledConnectivity.sendMessage();

    ((SPI::xSPIFrame_t *)(ledConnectivity.getSendFrame()))->data = color;
    ((SPI::xSPIFrame_t *)(ledConnectivity.getSendFrame()))->length = 24;
    ledConnectivity.sendMessage();

    ((SPI::xSPIFrame_t *)(ledConnectivity.getSendFrame()))->data = &reset;
    ((SPI::xSPIFrame_t *)(ledConnectivity.getSendFrame()))->length = 1;
    for (int i = 0; i < 100; i++)
        ledConnectivity.sendMessage();
}

void vTaskLed(void *pvParameters)
{
    ledConnectivity.init();

    TickType_t xLastWakeTime;
    TickType_t xFrequency = pdMS_TO_TICKS(5);

    uint8_t red = 0xff, green = 0x00, blue = 0x00;

    generateColor(red, green, blue);
    sendColor();

    xLastWakeTime = xTaskGetTickCount();
    while (true) {
        for (green = 0; green < 0xff; green++) {
            generateColor(red, green, blue);
            sendColor();
            vTaskDelayUntil(&xLastWakeTime, xFrequency);
        }
        for (red = 0xff; red > 0x00; red--) {
            generateColor(red, green, blue);
            sendColor();
            vTaskDelayUntil(&xLastWakeTime, xFrequency);
        }
        for (blue = 0x00; blue < 0xff; blue++) {
            generateColor(red, green, blue);
            sendColor();
            vTaskDelayUntil(&xLastWakeTime, xFrequency);
        }
        for (green = 0xff; green > 0x00; green--) {
            generateColor(red, green, blue);
            sendColor();
            vTaskDelayUntil(&xLastWakeTime, xFrequency);
        }
        for (red = 0x00; red < 0xff; red++) {
            generateColor(red, green, blue);
            sendColor();
            vTaskDelayUntil(&xLastWakeTime, xFrequency);
        }
        for (blue = 0xff; blue > 0x00; blue--) {
            generateColor(red, green, blue);
            sendColor();
            vTaskDelayUntil(&xLastWakeTime, xFrequency);
        }
    }
    vTaskDelete(ledTaskHandle);
}