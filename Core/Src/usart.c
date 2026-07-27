/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    usart.c
  * @brief   This file provides code for the configuration
  *          of the USART instances.
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
#include "usart.h"

/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

UART_HandleTypeDef huart1;

/* USART1 init function */

void MX_USART1_UART_Init(void)
{

  /* USER CODE BEGIN USART1_Init 0 */

  /* USER CODE END USART1_Init 0 */

  /* USER CODE BEGIN USART1_Init 1 */

  /* USER CODE END USART1_Init 1 */
  huart1.Instance = USART1;
  huart1.Init.BaudRate = 115200;
  huart1.Init.WordLength = UART_WORDLENGTH_8B;
  huart1.Init.StopBits = UART_STOPBITS_1;
  huart1.Init.Parity = UART_PARITY_NONE;
  huart1.Init.Mode = UART_MODE_TX_RX;
  huart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart1.Init.OverSampling = UART_OVERSAMPLING_16;
  huart1.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart1.Init.ClockPrescaler = UART_PRESCALER_DIV1;
  huart1.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_UART_Init(&huart1) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_SetTxFifoThreshold(&huart1, UART_TXFIFO_THRESHOLD_1_8) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_SetRxFifoThreshold(&huart1, UART_RXFIFO_THRESHOLD_1_8) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_DisableFifoMode(&huart1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART1_Init 2 */

  /* USER CODE END USART1_Init 2 */

}

void HAL_UART_MspInit(UART_HandleTypeDef* uartHandle)
{

  GPIO_InitTypeDef GPIO_InitStruct = {0};
  RCC_PeriphCLKInitTypeDef PeriphClkInit = {0};
  if(uartHandle->Instance==USART1)
  {
  /* USER CODE BEGIN USART1_MspInit 0 */

  /* USER CODE END USART1_MspInit 0 */

  /** Initializes the peripherals clocks
  */
    PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_USART1;
    PeriphClkInit.Usart1ClockSelection = RCC_USART1CLKSOURCE_PCLK2;
    if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
    {
      Error_Handler();
    }

    /* USART1 clock enable */
    __HAL_RCC_USART1_CLK_ENABLE();

    __HAL_RCC_GPIOB_CLK_ENABLE();
    /**USART1 GPIO Configuration
    PB6     ------> USART1_TX
    PB7     ------> USART1_RX
    */
    GPIO_InitStruct.Pin = GPIO_PIN_6|GPIO_PIN_7;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
    GPIO_InitStruct.Alternate = GPIO_AF7_USART1;
    HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

    /* USART1 interrupt Init */
    HAL_NVIC_SetPriority(USART1_IRQn, 0, 0);
    HAL_NVIC_EnableIRQ(USART1_IRQn);
  /* USER CODE BEGIN USART1_MspInit 1 */

  /* USER CODE END USART1_MspInit 1 */
  }
}

void HAL_UART_MspDeInit(UART_HandleTypeDef* uartHandle)
{

  if(uartHandle->Instance==USART1)
  {
  /* USER CODE BEGIN USART1_MspDeInit 0 */

  /* USER CODE END USART1_MspDeInit 0 */
    /* Peripheral clock disable */
    __HAL_RCC_USART1_CLK_DISABLE();

    /**USART1 GPIO Configuration
    PB6     ------> USART1_TX
    PB7     ------> USART1_RX
    */
    HAL_GPIO_DeInit(GPIOB, GPIO_PIN_6|GPIO_PIN_7);

    /* USART1 interrupt Deinit */
    HAL_NVIC_DisableIRQ(USART1_IRQn);
  /* USER CODE BEGIN USART1_MspDeInit 1 */

  /* USER CODE END USART1_MspDeInit 1 */
  }
}

/* USER CODE BEGIN 1 */

/* 非阻塞 TX 环形缓冲区 (中断驱动, printf 不阻塞控制环) ---------------*/
#define TX_RING_SIZE  512
static uint8_t  tx_ring[TX_RING_SIZE];
static volatile uint16_t tx_wr;   /* putchar 写入位置                   */
static volatile uint16_t tx_rd;   /* TX ISR 读出位置                    */
static volatile uint8_t  tx_active; /* TX 中断正在发送标志               */

/**
 * \brief           启动一次中断发送 (从 tx_rd 取 1 字节)
 */
static void tx_start(void)
{
    tx_active = 1;
    HAL_UART_Transmit_IT(&huart1, &tx_ring[tx_rd], 1);
}

/**
 * \brief           printf 重定向 (非阻塞)
 * \note            字符写入环形缓冲, 由 UART TX 中断后台发送.
 *                  缓冲满时丢弃 (不阻塞, 不掉帧会导致时序错乱).
 */
int __io_putchar(int ch)
{
    uint16_t next = (tx_wr + 1) % TX_RING_SIZE;
    if (next == tx_rd) {
        return ch;  /* 缓冲满, 丢弃 (不阻塞控制环) */
    }
    tx_ring[tx_wr] = (uint8_t)ch;
    tx_wr = next;

    /* 首次写入时启动中断发送 */
    if (!tx_active) {
        tx_start();
    }
    return ch;
}

/**
 * \brief           UART TX 完成回调
 */
void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart)
{
    if (huart->Instance != USART1) return;

    tx_rd = (tx_rd + 1) % TX_RING_SIZE;
    if (tx_rd != tx_wr) {
        /* 还有数据, 继续发送下一字节 */
        HAL_UART_Transmit_IT(&huart1, &tx_ring[tx_rd], 1);
    } else {
        tx_active = 0;  /* 发送完毕 */
    }
}

/**
 * \brief           UART 错误回调 (HAL 弱定义覆盖)
 * \note            电机 EMI 可导致 UART 帧错误/过载.
 *                  必须清除错误状态后重启 TX 和 RX 中断链.
 */
void HAL_UART_ErrorCallback(UART_HandleTypeDef *huart)
{
    if (huart->Instance != USART1) return;

    /* --- 恢复 TX --- */
    tx_active = 0;  /* 强制复位, 下次 printf 重新启动 TX */
    if (tx_wr != tx_rd) {
        tx_start();  /* 还有未发数据, 立即发送 */
    }

    /* --- 恢复 RX ---
     * 丢弃一个字符重启中断链, app_serial.c 的 RxCpltCallback
     * 会在下一个字符成功接收后重新指向正确的 rx_ring 缓冲区.
     */
    static uint8_t uart_err_dummy;
    HAL_UART_AbortReceive_IT(&huart1);
    HAL_UART_Receive_IT(&huart1, &uart_err_dummy, 1);
}

/* USER CODE END 1 */

