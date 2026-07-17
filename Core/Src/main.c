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
#include "dma.h"
#include "hrtim.h"
#include "spi.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "app_uart.h"
#include "mt6701.h"
#include "stdio.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
/* 电流采样参数 */
#define ADC_VREF          3.3f      /* ADC 参考电压 (V)               */
#define ADC_RESOLUTION    4096.0f   /* 12-bit 量程                    */
#define SHUNT_RESISTANCE  0.002f    /* 采样电阻 2mΩ                    */
#define OPAMP_GAIN        50.0f     /* 外部运放增益 100k/2k = 50 倍    */

/* 三相电流零点校准值 (电机未通电时 ADC 实测均值反推, 后续精确校准后更新) */
#define ADC_OFFSET_A      2041   /* Ia: 实测 ~2040.5 */
#define ADC_OFFSET_B      2024   /* Ib: 实测 ~2024.4 */
#define ADC_OFFSET_C      1990   /* Ic: 实测 ~1989.7 */

/* 预计算: ADC每LSB对应电流 mA (带符号) */
#define ADC_SCALE_MA_PER_LSB  ((ADC_VREF * 1000.0f) / (ADC_RESOLUTION * OPAMP_GAIN * SHUNT_RESISTANCE))
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
static uint32_t last_sensor_tick = 0;  /* 上次传感器读取的 systick 值 */
#define SENSOR_READ_INTERVAL_MS  10U    /* 传感器读取间隔 (ms)          */

/* ADC DMA 循环缓冲: 3 相电流 Ia, Ib, Ic */
static uint16_t adc_curr_buf[3] = {0};
/* 各通道零点 ADC 值 (运行时可用) */
static const uint16_t adc_offset[3] = {ADC_OFFSET_A, ADC_OFFSET_B, ADC_OFFSET_C};
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/**
 * @brief  将 ADC 值转换为电流 (mA), 使用各通道独立零点校准
 */
static inline float adc_to_current_ma(uint16_t adc_val, uint8_t channel)
{
    int32_t delta = (int32_t)adc_val - (int32_t)adc_offset[channel];
    return (float)delta * ADC_SCALE_MA_PER_LSB;
}

/**
 * @brief  发送三相电流调试数据
 * @note   格式: "I:ia,ib,ic\\r\\n"  mA×10 整数
 */
static void debug_print_currents(void)
{
    float ia = adc_to_current_ma(adc_curr_buf[0], 0);
    float ib = adc_to_current_ma(adc_curr_buf[1], 1);
    float ic = adc_to_current_ma(adc_curr_buf[2], 2);
    printf("I:%d,%d,%d\r\n",
           (int)(ia * 10.0f), (int)(ib * 10.0f), (int)(ic * 10.0f));
}

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
  MX_DMA_Init();
  MX_USART1_UART_Init();
  MX_SPI1_Init();
  MX_HRTIM1_Init();
  MX_ADC1_Init();
  /* USER CODE BEGIN 2 */
  printf("init start\r\n");
  mt6701_init();

  /* 启动 ADC DMA 循环采样 (由 HRTIM ADC Trigger 1 触发) */
  HAL_ADC_Start_DMA(&hadc1, (uint32_t *)adc_curr_buf, 3);

  HAL_HRTIM_WaveformCounterStart(&hhrtim1, HRTIM_TIMERID_MASTER
                                          | HRTIM_TIMERID_TIMER_A
                                          | HRTIM_TIMERID_TIMER_B
                                          | HRTIM_TIMERID_TIMER_D);
  HAL_HRTIM_WaveformOutputStart(&hhrtim1, HRTIM_OUTPUT_TA1 | HRTIM_OUTPUT_TA2
                                       | HRTIM_OUTPUT_TB1 | HRTIM_OUTPUT_TB2
                                       | HRTIM_OUTPUT_TD1 | HRTIM_OUTPUT_TD2);

  printf("init done\r\n");


  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
    /* 非阻塞定时读取传感器 (基于 SysTick) */
    uint32_t now = HAL_GetTick();
    if ((now - last_sensor_tick) >= SENSOR_READ_INTERVAL_MS) {
        last_sensor_tick = now;

        /* 编码器角度 */
        mt6701_data_t sensor = mt6701_read_angle();
        if (sensor.crc_valid) {
            int deg_int = (int)sensor.angle_deg;
            int deg_frac = (int)((sensor.angle_deg - (float)deg_int) * 100.0f + 0.5f);
            printf("angle:%d.%02d deg, status:%d, crc:0x%02X\r\n",
                   deg_int, deg_frac, sensor.status, sensor.crc);
        } else {
            printf("sensor read error (CRC mismatch)\r\n");
        }

        /* 三相电流 (mA×10, 即小数点后1位) */
        debug_print_currents();
    }
    /* 这里可以添加其他非阻塞任务 */
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
