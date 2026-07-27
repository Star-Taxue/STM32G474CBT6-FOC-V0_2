/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2026 STMicroelectronics.
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
#include "adc.h"
#include "hrtim.h"
#include "spi.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "app_uart.h"
#include "app_current_sense.h"
#include "app_openloop.h"
#include "app_foc.h"
#include "app_serial.h"
#include "mt6701.h"
#include "stdio.h"
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
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
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

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_USART1_UART_Init();
  MX_SPI1_Init();
  MX_HRTIM1_Init();
  MX_ADC1_Init();
  /* USER CODE BEGIN 2 */
  printf("init start\r\n");
  mt6701_init();

  /* 启动 ADC1 注入组 (JEOC 中断), 由 HRTIM ADC_TRG2 触发 */
  curr_sense_init();

  /*
   * 先启动 HRTIM 计数器产生 ADC 触发脉冲,
   * 但不启动 PWM 输出 (电机无电流), 用于零点校准.
   */
  HAL_HRTIM_WaveformCounterStart(&hhrtim1, HRTIM_TIMERID_MASTER
                                          | HRTIM_TIMERID_TIMER_A
                                          | HRTIM_TIMERID_TIMER_B
                                          | HRTIM_TIMERID_TIMER_D);

  /* 等待 ADC 稳定 + 注入组开始转换 */
  HAL_Delay(100);

  /* 零点校准: 采集 1024 次取平均 (电机必须静止) */
  printf("calibrating ADC offset (1024 samples)...\r\n");
  curr_sense_calibrate(1024);

  uint16_t off_a, off_b, off_c;
  curr_sense_get_offsets(&off_a, &off_b, &off_c);
  printf("ADC offset: A=%u, B=%u, C=%u\r\n", off_a, off_b, off_c);

  /* 启动 PWM 输出 */
  HAL_HRTIM_WaveformOutputStart(&hhrtim1, HRTIM_OUTPUT_TA1 | HRTIM_OUTPUT_TA2
                                       | HRTIM_OUTPUT_TB1 | HRTIM_OUTPUT_TB2
                                       | HRTIM_OUTPUT_TD1 | HRTIM_OUTPUT_TD2);

  /* 初始化开环电压驱动 (默认不输出, 等待上位机命令) */
  openloop_init();
  openloop_set_freq(10.0f);
  openloop_set_voltage(0.0f);   /* K=0, 无输出 */

  /* 初始化闭环 FOC (默认 idle, 从 openloop 读取 encoder offset) */
  foc_init();

  /* 初始化串口命令解析 (启动 UART RX 中断) */
  serial_init();

  printf("init done. type ? for help\r\n");


  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
    /*
     * 快速控制环: ADC 每次扫描完成 (20kHz) 时运行.
     * FOC 闭环与开环互斥调度.
     *
     * 注意: foc_step() 内部通过 curr_sense_get_all() 清除 new_data_flag,
     * 但 openloop_step() 不读电流, 因此必须在这里原子读取并清除标志,
     * 防止 openloop_step() 以 CPU 速度失控运行.
     */
    if (curr_sense_data_ready()) {
        if (foc_is_enabled()) {
            foc_step();
        } else {
            /* 原子清除 data_ready 标志, 让 openloop_step() 严格按 20kHz 运行 */
            __disable_irq();
            new_data_flag_clear();
            __enable_irq();
            openloop_step();
        }
    }

    /* 串口命令轮询: 尽量频繁调用, 确保命令及时响应 */
    serial_poll();

    /* 开环调试快照打印 (非实时, 不阻塞控制环) */
    openloop_debug_poll();
  }

  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Configure the main internal regulator output voltage
  */
  HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE1);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = RCC_PLLM_DIV4;
  RCC_OscInitStruct.PLL.PLLN = 75;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = RCC_PLLQ_DIV2;
  RCC_OscInitStruct.PLL.PLLR = RCC_PLLR_DIV2;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_4) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1)
  {
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
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
