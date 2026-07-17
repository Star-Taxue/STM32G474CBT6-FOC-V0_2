/**
 * @file    mt6701.c
 * @brief   MT6701 磁编码器 SPI 驱动
 * @note    通过 SPI1 读取 14-bit 角度, 4-bit 状态, 6-bit CRC
 */

#include "mt6701.h"
#include "main.h"
#include "spi.h"
#include "stm32g4xx_hal.h"

/* -------------------------------------------------------------------------- */
/*  片选宏定义                                                                 */
/* -------------------------------------------------------------------------- */
#define MT6701_CSN_PORT     CS_GPIO_Port
#define MT6701_CSN_PIN      CS_Pin

#define MT6701_CSN_LOW()    HAL_GPIO_WritePin(MT6701_CSN_PORT, MT6701_CSN_PIN, GPIO_PIN_RESET)
#define MT6701_CSN_HIGH()   HAL_GPIO_WritePin(MT6701_CSN_PORT, MT6701_CSN_PIN, GPIO_PIN_SET)

/* SPI 通信超时 (ms) */
#define MT6701_SPI_TIMEOUT  10U

/* -------------------------------------------------------------------------- */
/*  微秒级延时 (基于 DWT 周期计数器)                                            */
/* -------------------------------------------------------------------------- */
static void delay_us(uint32_t us)
{
    uint32_t start = DWT->CYCCNT;
    uint32_t ticks = us * (SystemCoreClock / 1000000U);
    /* 等待直到计数器差值达到目标 tick 数, 自然处理 32-bit 翻转 */
    while ((DWT->CYCCNT - start) < ticks) {
        /* busy-wait */
    }
}

/**
 * @brief  计算 MT6701 CRC-6 校验值
 * @param  data  18-bit 输入数据 (高14位角度 + 低4位状态)
 * @retval 6-bit CRC 值
 * @note   多项式 MT6701_CRC6_POLY, 初始值 0x00
 *         CRC 覆盖: angle[13:0] + status[3:0] 共 18 bits
 */
static uint8_t mt6701_calc_crc6(uint32_t data)
{
    uint8_t crc = 0;
    /* 从最高位 (bit 17) 开始逐位计算 */
    for (int i = 17; i >= 0; i--) {
        crc <<= 1;
        if (((data >> i) & 1U) ^ ((crc >> 6) & 1U)) {
            crc ^= MT6701_CRC6_POLY;
        }
        crc &= 0x3FU;  /* 保持 6-bit 宽度 */
    }
    return crc;
}

/* -------------------------------------------------------------------------- */
/*  初始化                                                                     */
/* -------------------------------------------------------------------------- */
void mt6701_init(void)
{
    /* 使能 DWT 周期计数器, 用于 delay_us() */
    CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;
    DWT->CYCCNT = 0;
    DWT->CTRL |= DWT_CTRL_CYCCNTENA_Msk;

    /* 释放片选, 确保 SPI 总线空闲 */
    MT6701_CSN_HIGH();

    /* 等待芯片上电稳定 */
    HAL_Delay(1);
}

/* -------------------------------------------------------------------------- */
/*  读取角度数据 (带 CRC 失败重试)                                               */
/* -------------------------------------------------------------------------- */
#define MT6701_RETRY_MAX  3   /* CRC 失败重试次数 */

mt6701_data_t mt6701_read_angle(void)
{
    uint8_t  rx_buf[3] = {0};
    mt6701_data_t result;
    static mt6701_data_t last_valid = {0};
    HAL_StatusTypeDef hal_ret;

    for (int retry = 0; retry < MT6701_RETRY_MAX; retry++) {

        /* --- 1. SPI 读取 3 字节 --- */
        MT6701_CSN_LOW();
        delay_us(2);
        hal_ret = HAL_SPI_Receive(&hspi1, rx_buf, 3, MT6701_SPI_TIMEOUT);
        MT6701_CSN_HIGH();

        if (hal_ret != HAL_OK) {
            continue;  /* SPI 硬件错误, 重试 */
        }

        /* --- 2. 解析 24-bit 数据包 --- */
        uint32_t raw = ((uint32_t)rx_buf[0] << 16)
                     | ((uint32_t)rx_buf[1] << 8)
                     | ((uint32_t)rx_buf[2]);

        uint16_t angle    = (raw >> 10) & 0x3FFFU;
        uint8_t  status   = (raw >> 6)  & 0x0FU;
        uint8_t  crc_rcvd =  raw        & 0x3FU;

        /* --- 3. CRC 校验 --- */
        uint32_t crc_input = ((uint32_t)angle << 4) | status;
        uint8_t  crc_calc  = mt6701_calc_crc6(crc_input);

        if (crc_calc != crc_rcvd) {
            continue;  /* CRC 不匹配, 重试 */
        }

        /* --- 4. 校验通过, 保存并返回 --- */
        result.raw_angle = angle;
        result.status    = status;
        result.crc       = crc_rcvd;
        result.crc_valid = 1U;
        result.angle_deg = (float)angle * (360.0f / 16384.0f);
        last_valid = result;
        return result;
    }

    /* 所有重试均失败, 返回上次有效值 + invalid 标记 */
    last_valid.crc_valid = 0;
    return last_valid;
}
