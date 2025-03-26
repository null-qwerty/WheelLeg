/* USER CODE BEGIN Header */
/**
 ******************************************************************************
 * @file           : main.c
 * @brief          : Main program body
 ******************************************************************************
 * @attention
 *
 * Copyright (c) 2025 STMicroelectronics.
 * All rights reserved.
 *
 * This software is licensed under terms that can be found in the LICENSE file
 * in the root directory of this software component.
 * If no LICENSE file comes with this software, it is provided AS-IS.
 *
 ******************************************************************************
 */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "tasks.hpp"
#include "Utils/buzzer.hpp"
#include "Utils/note.hpp"
#include "Utils/music.hpp"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
uint16_t delayUnit = 150;
// windows XP startup music
// Note note[] = { { H3b, 2 * delayUnit }, { M3b, 1 * delayUnit },
//                 { M7b, 4 * delayUnit }, { M6b, 6 * delayUnit },
//                 { H3b, 2 * delayUnit }, { M7b, 10 * delayUnit } };
// default startup music
Note note[] = { { H1, 1 * delayUnit }, { Z0, 1 * delayUnit },
                { H1, 1 * delayUnit }, { Z0, 2 * delayUnit },
                { H1, 1 * delayUnit }, { H2, 1 * delayUnit },
                { H3, 1 * delayUnit } };
// a moon filled sky. (from ef - a fairy tale of the two.)
// Note note[] = { { L6, 1 * delayUnit },  { M4s, 2 * delayUnit },
//                 { M5, 1 * delayUnit },  { M4s, 2 * delayUnit },
//                 { M3, 1 * delayUnit },  { M2, 4 * delayUnit },
//                 { Z0, 1 * delayUnit },  { M4s, 1 * delayUnit },
//                 { H2, 2 * delayUnit },  { H2, 1 * delayUnit },
//                 { H3, 1 * delayUnit },  { H2, 1 * delayUnit },
//                 { H1s, 1 * delayUnit }, { H2, 4 * delayUnit } };
// you - GALA
// Note note[] = {
//     { M5, 4 * delayUnit },   { M3b, 4 * delayUnit },  { M7b, 4 * delayUnit },
//     { M6b, 4 * delayUnit },  { M5, 4 * delayUnit },   { M3b, 4 * delayUnit },
//     { M7b, 4 * delayUnit },  { M6b, 4 * delayUnit },  { M5, 4 * delayUnit },
//     { M3b, 4 * delayUnit },  { M7b, 4 * delayUnit },  { M6b, 4 * delayUnit },
//     { M5, 4 * delayUnit },   { M3b, 4 * delayUnit },  { M7b, 4 * delayUnit },
//     { M6b, 2 * delayUnit },  { L7b, 2 * delayUnit },  { M1, 6 * delayUnit },
//     { L7b, 2 * delayUnit },  { M1, 2 * delayUnit },   { L7b, 2 * delayUnit },
//     { M1, 2 * delayUnit },   { M3b, 4 * delayUnit },  { M6b, 4 * delayUnit },
//     { M5, 4 * delayUnit },   { M1, 4 * delayUnit },   { L7b, 2 * delayUnit },
//     { M1, 6 * delayUnit },   { L7b, 2 * delayUnit },  { M1, 2 * delayUnit },
//     { L7b, 2 * delayUnit },  { M1, 2 * delayUnit },   { M6b, 8 * delayUnit },
//     { M5, 2 * delayUnit },   { M7b, 4 * delayUnit },  { M1, 4 * delayUnit },
//     { M1, 6 * delayUnit },   { L7b, 2 * delayUnit },  { M1, 2 * delayUnit },
//     { L7b, 2 * delayUnit },  { M1, 2 * delayUnit },   { M5, 4 * delayUnit },
//     { M6b, 4 * delayUnit },  { M5, 4 * delayUnit },   { M1, 4 * delayUnit },
//     { L7b, 2 * delayUnit },  { M1, 6 * delayUnit },   { L7b, 2 * delayUnit },
//     { M1, 2 * delayUnit },   { L7b, 2 * delayUnit },  { M1, 2 * delayUnit },
//     { M5, 4 * delayUnit },   { M6b, 4 * delayUnit },  { M7b, 10 * delayUnit
//     }, { Z0, 2 * delayUnit },   { M1, 2 * delayUnit },   { M1, 2 * delayUnit
//     }, { L7b, 1 * delayUnit },  { M1, 3 * delayUnit },   { L7b, 2 * delayUnit
//     }, { M1, 2 * delayUnit },   { M3b, 2 * delayUnit },  { Z0, 2 * delayUnit
//     }, { M1, 2 * delayUnit },   { M1, 2 * delayUnit },   { L7b, 1 * delayUnit
//     }, { M1, 3 * delayUnit },   { L7b, 2 * delayUnit },  { M1, 2 * delayUnit
//     }, { M3b, 4 * delayUnit },  { L6b, 6 * delayUnit },  { Z0, 4 * delayUnit
//     }, { Z0, 4 * delayUnit },   { Z0, 2 * delayUnit },   { M1, 2 * delayUnit
//     }, { M1, 2 * delayUnit },   { L7b, 1 * delayUnit },  { M1, 1 * delayUnit
//     }, { L7b, 2 * delayUnit },  { L7b, 1 * delayUnit },  { L6b, 1 * delayUnit
//     }, { L6b, 2 * delayUnit },  { L7b, 2 * delayUnit },  { Z0, 2 * delayUnit
//     }, { M1, 2 * delayUnit },   { M1, 2 * delayUnit },   { L7b, 1 * delayUnit
//     }, { M1, 3 * delayUnit },   { L7b, 2 * delayUnit },  { M1, 2 * delayUnit
//     }, { M3b, 2 * delayUnit },  { Z0, 2 * delayUnit },   { M1, 2 * delayUnit
//     }, { M1, 2 * delayUnit },   { L7b, 1 * delayUnit },  { M1, 3 * delayUnit
//     }, { L7b, 2 * delayUnit },  { M1, 2 * delayUnit },   { M6b, 4 * delayUnit
//     }, { L6b, 6 * delayUnit },  { Z0, 4 * delayUnit },   { Z0, 4 * delayUnit
//     }, { Z0, 2 * delayUnit },   { M1, 2 * delayUnit },   { M1, 2 * delayUnit
//     }, { L7b, 1 * delayUnit },  { M1, 3 * delayUnit },   { L7b, 2 * delayUnit
//     }, { L5, 2 * delayUnit },   { L4, 1 * delayUnit },   { L3b, 1 * delayUnit
//     }, { L3b, 4 * delayUnit },  { Z0, 1 * delayUnit },   { L3b, 1 * delayUnit
//     }, { L3b, 1 * delayUnit },  { L3b, 1 * delayUnit },  { M3b, 4 * delayUnit
//     }, { M1, 3 * delayUnit },   { L7b, 1 * delayUnit },  { L7b, 1 * delayUnit
//     }, { L6b, 1 * delayUnit },  { L6b, 4 * delayUnit },  { L6b, 2 * delayUnit
//     }, { L6b, 2 * delayUnit },  { L7b, 2 * delayUnit },  { M1, 2 * delayUnit
//     }, { L4, 6 * delayUnit },   { Z0, 2 * delayUnit },   { L4, 2 * delayUnit
//     }, { M1, 2 * delayUnit },   { L7b, 2 * delayUnit },  { L6b, 2 * delayUnit
//     }, { L7b, 10 * delayUnit }, { M1, 4 * delayUnit },   { M2b, 4 * delayUnit
//     }, { M3b, 6 * delayUnit },  { M1, 2 * delayUnit },   { M3b, 2 * delayUnit
//     }, { M1, 1 * delayUnit },   { M3b, 3 * delayUnit },  { M5, 4 * delayUnit
//     }, { M6b, 6 * delayUnit },  { L6b, 2 * delayUnit },  { L7b, 2 * delayUnit
//     }, { M1, 2 * delayUnit },   { M3b, 4 * delayUnit },  { M4, 4 * delayUnit
//     }, { M3b, 1 * delayUnit },  { M4, 3 * delayUnit },   { M3b, 1 * delayUnit
//     }, { M3b, 3 * delayUnit },  { M3b, 2 * delayUnit },  { L7b, 2 * delayUnit
//     }, { L7b, 6 * delayUnit },  { M1, 4 * delayUnit },   { M2b, 4 * delayUnit
//     }, { M3b, 6 * delayUnit },  { M1, 2 * delayUnit },   { M3b, 2 * delayUnit
//     }, { M1, 1 * delayUnit },   { M3b, 3 * delayUnit },  { M5, 4 * delayUnit
//     }, { M6b, 6 * delayUnit },  { L6b, 2 * delayUnit },  { L7b, 2 * delayUnit
//     }, { M1, 2 * delayUnit },   { M5, 4 * delayUnit },   { M4, 6 * delayUnit
//     }, { M4, 2 * delayUnit },   { M3b, 1 * delayUnit },  { M4, 3 * delayUnit
//     }, { M6b, 4 * delayUnit },  { M7b, 6 * delayUnit },  { Z0, 2 * delayUnit
//     }, { M3b, 2 * delayUnit },  { M6b, 2 * delayUnit },  { M5, 1 * delayUnit
//     }, { M6b, 1 * delayUnit },  { M1, 6 * delayUnit },   { L7b, 2 * delayUnit
//     }, { M1, 2 * delayUnit },   { L7b, 2 * delayUnit },  { M1, 2 * delayUnit
//     }, { M5, 4 * delayUnit },   { M6b, 4 * delayUnit },  { M5, 4 * delayUnit
//     }, { M1, 4 * delayUnit },   { L7b, 2 * delayUnit },  { M1, 6 * delayUnit
//     }, { L7b, 2 * delayUnit },  { M1, 2 * delayUnit },   { L7b, 2 * delayUnit
//     }, { M1, 2 * delayUnit },   { M5, 4 * delayUnit },   { M6b, 4 * delayUnit
//     }, { M7b, 10 * delayUnit }, { Z0, 2 * delayUnit },   { M1, 2 * delayUnit
//     }, { M1, 2 * delayUnit },   { L7b, 1 * delayUnit },  { M1, 3 * delayUnit
//     }, { L7b, 2 * delayUnit },  { M1, 2 * delayUnit },   { M3b, 2 * delayUnit
//     }, { Z0, 2 * delayUnit },   { M1, 2 * delayUnit },   { M1, 2 * delayUnit
//     }, { L7b, 1 * delayUnit },  { M1, 3 * delayUnit },   { L7b, 2 * delayUnit
//     }, { M1, 2 * delayUnit },   { M3b, 4 * delayUnit },  { L6b, 6 * delayUnit
//     }, { Z0, 4 * delayUnit },   { Z0, 4 * delayUnit },   { Z0, 2 * delayUnit
//     }, { M1, 2 * delayUnit },   { M1, 2 * delayUnit },   { L7b, 1 * delayUnit
//     }, { M1, 1 * delayUnit },   { L7b, 2 * delayUnit },  { L7b, 1 * delayUnit
//     }, { L6b, 1 * delayUnit },  { L6b, 2 * delayUnit },  { L7b, 2 * delayUnit
//     }, { Z0, 2 * delayUnit },   { M1, 2 * delayUnit },   { M1, 2 * delayUnit
//     }, { L7b, 1 * delayUnit },  { M1, 3 * delayUnit },   { L7b, 2 * delayUnit
//     }, { M1, 2 * delayUnit },   { M3b, 2 * delayUnit },  { Z0, 2 * delayUnit
//     }, { M1, 2 * delayUnit },   { M1, 2 * delayUnit },   { L7b, 1 * delayUnit
//     }, { M1, 3 * delayUnit },   { L7b, 2 * delayUnit },  { M1, 2 * delayUnit
//     }, { M6b, 4 * delayUnit },  { L6b, 6 * delayUnit },  { Z0, 4 * delayUnit
//     }, { Z0, 4 * delayUnit },   { Z0, 2 * delayUnit },   { M1, 2 * delayUnit
//     }, { M1, 2 * delayUnit },   { L7b, 1 * delayUnit },  { M1, 3 * delayUnit
//     }, { L7b, 2 * delayUnit },  { L5, 2 * delayUnit },   { L4, 1 * delayUnit
//     }, { L3b, 1 * delayUnit },  { L3b, 4 * delayUnit },  { Z0, 1 * delayUnit
//     }, { L3b, 1 * delayUnit },  { L3b, 1 * delayUnit },  { L3b, 1 * delayUnit
//     }, { M3b, 4 * delayUnit },  { M1, 3 * delayUnit },   { L7b, 1 * delayUnit
//     }, { L7b, 1 * delayUnit },  { L6b, 1 * delayUnit },  { L6b, 4 * delayUnit
//     }, { L6b, 2 * delayUnit },  { L6b, 2 * delayUnit },  { L7b, 2 * delayUnit
//     }, { M1, 2 * delayUnit },   { L4, 6 * delayUnit },   { Z0, 2 * delayUnit
//     }, { L4, 2 * delayUnit },   { M1, 2 * delayUnit },   { L7b, 2 * delayUnit
//     }, { L6b, 2 * delayUnit },  { L7b, 26 * delayUnit }, { M1, 4 * delayUnit
//     }, { M2b, 4 * delayUnit },  { M3b, 6 * delayUnit },  { M1, 2 * delayUnit
//     }, { M3b, 2 * delayUnit },  { M1, 1 * delayUnit },   { M3b, 3 * delayUnit
//     }, { M5, 4 * delayUnit },   { M6b, 6 * delayUnit },  { L6b, 2 * delayUnit
//     }, { L7b, 2 * delayUnit },  { M1, 2 * delayUnit },   { M3b, 4 * delayUnit
//     }, { M4, 5 * delayUnit },   { M3b, 1 * delayUnit },  { M4, 2 * delayUnit
//     }, { M3b, 1 * delayUnit },  { M3b, 3 * delayUnit },  { M3b, 2 * delayUnit
//     }, { L7b, 2 * delayUnit },  { L7b, 6 * delayUnit },  { M1, 4 * delayUnit
//     }, { M2b, 4 * delayUnit },  { M3b, 8 * delayUnit },  { M3b, 2 * delayUnit
//     }, { M1, 1 * delayUnit },   { M3b, 3 * delayUnit },  { M7b, 4 * delayUnit
//     }, { M6b, 6 * delayUnit },  { L6b, 2 * delayUnit },  { L7b, 2 * delayUnit
//     }, { M1, 2 * delayUnit },   { M6b, 4 * delayUnit },  { M4, 5 * delayUnit
//     }, { M3b, 1 * delayUnit },  { M4, 2 * delayUnit },   { M3b, 1 * delayUnit
//     }, { M4, 3 * delayUnit },   { M6b, 4 * delayUnit },  { M7b, 6 * delayUnit
//     }, { Z0, 2 * delayUnit },   { L7b, 2 * delayUnit },  { M1, 2 * delayUnit
//     }, { L7b, 1 * delayUnit },  { L6b, 17 * delayUnit },
// };
Buzzer buzzer(&htim12, TIM_CHANNEL_2, 275 * 1e6, 100);
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/

/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

/**
 * @brief  The application entry point.
 * @retval int
 */
int main(void)
{
    /* USER CODE BEGIN 1 */

    /* USER CODE END 1 */

    /* MCU
     * Configuration--------------------------------------------------------*/

    /* Reset of all peripherals, Initializes the Flash interface and the
     * Systick. */
    HAL_Init();

    /* USER CODE BEGIN Init */

    /* USER CODE END Init */

    /* Configure the system clock */
    SystemClock_Config();

    /* Configure the peripherals common clocks */
    PeriphCommonClock_Config();

    /* USER CODE BEGIN SysInit */

    /* USER CODE END SysInit */

    /* Initialize all configured peripherals */
    MX_GPIO_Init();
    MX_DMA_Init();
    MX_USART2_UART_Init();
    MX_USART3_UART_Init();
    MX_FDCAN1_Init();
    MX_SPI2_Init();
    MX_TIM12_Init();
    MX_UART5_Init();
    MX_TIM3_Init();
    MX_SPI6_Init();
    MX_TIM7_Init();
    /* USER CODE BEGIN 2 */
    HAL_TIM_Base_Start(&htim12);
    HAL_TIM_PWM_Start(&htim12, TIM_CHANNEL_2);

    for (auto &n : note) {
        buzzer.play(n);
    }

    wheelControlMutex = xSemaphoreCreateMutex();
    /* USER CODE END 2 */

    /* Init scheduler */
    osKernelInitialize();

    /* Call init function for freertos objects (in cmsis_os2.c) */
    MX_FREERTOS_Init();
    WheelLegTasksInit();

    /* Start scheduler */
    osKernelStart();

    /* We should never get here as control is now taken by the scheduler */

    /* Infinite loop */
    /* USER CODE BEGIN WHILE */
    while (1) {
        /* USER CODE END WHILE */

        /* USER CODE BEGIN 3 */
    }
    /* USER CODE END 3 */
}

/**
 * @brief System Clock Configuration
 * @retval None
 */
void SystemClock_Config(void)
{
    RCC_OscInitTypeDef RCC_OscInitStruct = { 0 };
    RCC_ClkInitTypeDef RCC_ClkInitStruct = { 0 };

    /** Supply configuration update enable
     */
    HAL_PWREx_ConfigSupply(PWR_LDO_SUPPLY);

    /** Configure the main internal regulator output voltage
     */
    __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE0);

    while (!__HAL_PWR_GET_FLAG(PWR_FLAG_VOSRDY)) {
    }

    /** Initializes the RCC Oscillators according to the specified parameters
     * in the RCC_OscInitTypeDef structure.
     */
    RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
    RCC_OscInitStruct.HSEState = RCC_HSE_ON;
    RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
    RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
    RCC_OscInitStruct.PLL.PLLM = 12;
    RCC_OscInitStruct.PLL.PLLN = 275;
    RCC_OscInitStruct.PLL.PLLP = 1;
    RCC_OscInitStruct.PLL.PLLQ = 4;
    RCC_OscInitStruct.PLL.PLLR = 2;
    RCC_OscInitStruct.PLL.PLLRGE = RCC_PLL1VCIRANGE_3;
    RCC_OscInitStruct.PLL.PLLVCOSEL = RCC_PLL1VCOWIDE;
    RCC_OscInitStruct.PLL.PLLFRACN = 0;
    if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK) {
        Error_Handler();
    }

    /** Initializes the CPU, AHB and APB buses clocks
     */
    RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_SYSCLK |
                                  RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2 |
                                  RCC_CLOCKTYPE_D3PCLK1 | RCC_CLOCKTYPE_D1PCLK1;
    RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
    RCC_ClkInitStruct.SYSCLKDivider = RCC_SYSCLK_DIV1;
    RCC_ClkInitStruct.AHBCLKDivider = RCC_HCLK_DIV2;
    RCC_ClkInitStruct.APB3CLKDivider = RCC_APB3_DIV2;
    RCC_ClkInitStruct.APB1CLKDivider = RCC_APB1_DIV2;
    RCC_ClkInitStruct.APB2CLKDivider = RCC_APB2_DIV2;
    RCC_ClkInitStruct.APB4CLKDivider = RCC_APB4_DIV2;

    if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_3) != HAL_OK) {
        Error_Handler();
    }
}

/**
 * @brief Peripherals Common Clock Configuration
 * @retval None
 */
void PeriphCommonClock_Config(void)
{
    RCC_PeriphCLKInitTypeDef PeriphClkInitStruct = { 0 };

    /** Initializes the peripherals clock
     */
    PeriphClkInitStruct.PeriphClockSelection =
        RCC_PERIPHCLK_UART5 | RCC_PERIPHCLK_USART2 | RCC_PERIPHCLK_USART3;
    PeriphClkInitStruct.PLL3.PLL3M = 24;
    PeriphClkInitStruct.PLL3.PLL3N = 384;
    PeriphClkInitStruct.PLL3.PLL3P = 1;
    PeriphClkInitStruct.PLL3.PLL3Q = 5;
    PeriphClkInitStruct.PLL3.PLL3R = 2;
    PeriphClkInitStruct.PLL3.PLL3RGE = RCC_PLL3VCIRANGE_0;
    PeriphClkInitStruct.PLL3.PLL3VCOSEL = RCC_PLL3VCOMEDIUM;
    PeriphClkInitStruct.PLL3.PLL3FRACN = 0;
    PeriphClkInitStruct.Usart234578ClockSelection =
        RCC_USART234578CLKSOURCE_PLL3;
    if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInitStruct) != HAL_OK) {
        Error_Handler();
    }
}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

/**
 * @brief  Period elapsed callback in non blocking mode
 * @note   This function is called  when TIM2 interrupt took place, inside
 * HAL_TIM_IRQHandler(). It makes a direct call to HAL_IncTick() to increment
 * a global variable "uwTick" used as application time base.
 * @param  htim : TIM handle
 * @retval None
 */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
    /* USER CODE BEGIN Callback 0 */

    /* USER CODE END Callback 0 */
    if (htim->Instance == TIM2) {
        HAL_IncTick();
    }
    /* USER CODE BEGIN Callback 1 */

    /* USER CODE END Callback 1 */
}

/**
 * @brief  This function is executed in case of error occurrence.
 * @retval None
 */
void Error_Handler(void)
{
    /* USER CODE BEGIN Error_Handler_Debug */
    /* User can add his own implementation to report the HAL error return state
     */
    __disable_irq();
    while (1) {
    }
    /* USER CODE END Error_Handler_Debug */
}

#ifdef USE_FULL_ASSERT
/**
 * @brief  Reports the name of the source file and the source line number
 *         where the assert_param error has occurred.
 * @param  file: pointer to the source file name
 * @param  line: assert_param error line source number
 * @retval None
 */
void assert_failed(uint8_t *file, uint32_t line)
{
    /* USER CODE BEGIN 6 */
    /* User can add his own implementation to report the file name and line
       number, ex: printf("Wrong parameters value: file %s on line %d\r\n",
       file, line) */
    /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
