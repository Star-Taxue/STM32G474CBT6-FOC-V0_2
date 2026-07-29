/**
 * @file    mt6701.c
 * @brief   MT6701 磁编码器 SPI 驱动 (非阻塞 DMA 版本)
 * @note    通过 SPI1 DMA 异步读取 14-bit 角度, 4-bit 状态, 6-bit CRC.
 *          数据由 DMA 完成中断解析并存入缓存, 主循环通过 mt6701_read_angle()
 *          直接从内存读取, 无 SPI 阻塞. 需要周期性调用 mt6701_trigger_read()
 *          启动新的 DMA 传输 (建议在 20kHz 控制步结束时调用).
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

/* CRC 失败重试次数 (仅阻塞模式) */
#define MT6701_RETRY_MAX    3

/* -------------------------------------------------------------------------- */
/*  DMA 非阻塞读取状态                                                          */
/* -------------------------------------------------------------------------- */

static uint8_t  dma_rx_buf[3];              /**< DMA 接收缓冲 (3 字节)        */
static volatile mt6701_data_t g_cached;     /**< 缓存的最近一次角度数据        */
static volatile uint8_t g_data_updated;     /**< ISR 写入后置 1, reader 清 0  */
static volatile uint8_t g_spi_busy;         /**< 1 = DMA 传输进行中            */
static volatile uint8_t g_crc_fail_count;   /**< 连续 CRC 失败计数 (诊断用)    */
static mt6701_data_t g_last_valid;          /**< CRC 失败时返回的上次有效值    */

/* -------------------------------------------------------------------------- */
/*  微秒级延时 (基于 DWT 周期计数器)                                            */
/* -------------------------------------------------------------------------- */
static void delay_us(uint32_t us)
{
    uint32_t start = DWT->CYCCNT;
    uint32_t ticks = us * (SystemCoreClock / 1000000U);
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
    for (int i = 17; i >= 0; i--) {
        crc <<= 1;
        if (((data >> i) & 1U) ^ ((crc >> 6) & 1U)) {
            crc ^= MT6701_CRC6_POLY;
        }
        crc &= 0x3FU;
    }
    return crc;
}

/* -------------------------------------------------------------------------- */
/*  HAL SPI 回调 (覆盖 HAL 弱定义)                                              */
/* -------------------------------------------------------------------------- */

/**
 * @brief  SPI DMA 接收完成回调 (DMA ISR 上下文)
 * @note   解析 24-bit 数据帧, CRC 校验, 存入 g_cached.
 *         失败时保留上次有效数据, 递增错误计数.
 */
void HAL_SPI_RxCpltCallback(SPI_HandleTypeDef *hspi)
{
    if (hspi->Instance != SPI1) return;

    /* 等待 SPI 外设完成最后一个字节的移位 (BSY → 0) */
    while (__HAL_SPI_GET_FLAG(hspi, SPI_FLAG_BSY)) {
        /* 通常只需数个 APB 周期 */
    }

    /* 释放片选 */
    MT6701_CSN_HIGH();

    /* 解析 24-bit 数据包 */
    uint32_t raw = ((uint32_t)dma_rx_buf[0] << 16)
                 | ((uint32_t)dma_rx_buf[1] << 8)
                 | ((uint32_t)dma_rx_buf[2]);

    uint16_t angle    = (raw >> 10) & 0x3FFFU;
    uint8_t  status   = (raw >> 6)  & 0x0FU;
    uint8_t  crc_rcvd =  raw        & 0x3FU;

    /* CRC 校验 */
    uint32_t crc_input = ((uint32_t)angle << 4) | status;
    uint8_t  crc_calc  = mt6701_calc_crc6(crc_input);

    if (crc_calc == crc_rcvd) {
        g_crc_fail_count = 0;

        g_cached.raw_angle = angle;
        g_cached.status    = status;
        g_cached.crc       = crc_rcvd;
        g_cached.crc_valid = 1U;
        g_cached.angle_deg = (float)angle * MT6701_ANGLE_SCALE;

        g_last_valid = g_cached;
    } else {
        g_crc_fail_count++;
        /* 保留上次有效数据, 仅标记 invalid */
        g_cached = g_last_valid;
        g_cached.crc_valid = 0;
    }

    g_data_updated = 1;
    g_spi_busy     = 0;
}

/**
 * @brief  SPI 错误回调 (DMA ISR 上下文)
 * @note   超时/CRC/DMA 错误时 HAL 调用此函数. 释放 CS 和 busy 标志.
 */
void HAL_SPI_ErrorCallback(SPI_HandleTypeDef *hspi)
{
    if (hspi->Instance != SPI1) return;

    MT6701_CSN_HIGH();
    g_spi_busy     = 0;
    g_crc_fail_count++;
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

    /* 阻塞读取一次, 填充缓存供后续非阻塞读取 */
    g_cached = mt6701_read_angle_blocking();
    g_last_valid    = g_cached;
    g_data_updated  = 1;
    g_spi_busy      = 0;
    g_crc_fail_count = 0;
}

/* -------------------------------------------------------------------------- */
/*  非阻塞 DMA 触发                                                            */
/* -------------------------------------------------------------------------- */
void mt6701_trigger_read(void)
{
    if (g_spi_busy) {
        /* 上次传输尚未完成 (不应在 50μs 周期内出现), 跳过本次触发 */
        return;
    }

    g_spi_busy = 1;
    MT6701_CSN_LOW();
    delay_us(2);

    HAL_StatusTypeDef ret = HAL_SPI_Receive_DMA(&hspi1, dma_rx_buf, 3);
    if (ret != HAL_OK) {
        /* DMA 启动失败 (通常为 HAL_BUSY), 回退到中断模式 */
        MT6701_CSN_HIGH();
        g_spi_busy = 0;
    }
}

/* -------------------------------------------------------------------------- */
/*  非阻塞读取: 直接从内存读缓存                                                */
/* -------------------------------------------------------------------------- */
mt6701_data_t mt6701_read_angle(void)
{
    mt6701_data_t result;

    /* 关中断原子拷贝, 防止 ISR 写入期间 CPU 读到半更新数据 */
    __disable_irq();
    result = g_cached;
    g_data_updated = 0;
    __enable_irq();

    return result;
}

/**
 * @brief  查询自上次 mt6701_read_angle() 以来是否有新数据
 */
uint8_t mt6701_is_data_fresh(void)
{
    return g_data_updated;
}

/**
 * @brief  查询 SPI+DMA 是否正忙
 */
uint8_t mt6701_is_busy(void)
{
    return g_spi_busy;
}

/* -------------------------------------------------------------------------- */
/*  阻塞读取: 用于校准、诊断等非实时场合                                        */
/* -------------------------------------------------------------------------- */
mt6701_data_t mt6701_read_angle_blocking(void)
{
    uint8_t  rx_buf[3] = {0};
    mt6701_data_t result;
    static mt6701_data_t last_good = {0};
    static uint8_t had_good = 0;

    for (int retry = 0; retry < MT6701_RETRY_MAX; retry++) {

        /* SPI 阻塞读取 3 字节 */
        MT6701_CSN_LOW();
        delay_us(2);
        HAL_StatusTypeDef hal_ret = HAL_SPI_Receive(&hspi1, rx_buf, 3, MT6701_SPI_TIMEOUT);
        MT6701_CSN_HIGH();

        if (hal_ret != HAL_OK) {
            continue;
        }

        /* 解析 24-bit 数据包 */
        uint32_t raw = ((uint32_t)rx_buf[0] << 16)
                     | ((uint32_t)rx_buf[1] << 8)
                     | ((uint32_t)rx_buf[2]);

        uint16_t angle    = (raw >> 10) & 0x3FFFU;
        uint8_t  status   = (raw >> 6)  & 0x0FU;
        uint8_t  crc_rcvd =  raw        & 0x3FU;

        /* CRC 校验 */
        uint32_t crc_input = ((uint32_t)angle << 4) | status;
        uint8_t  crc_calc  = mt6701_calc_crc6(crc_input);

        if (crc_calc != crc_rcvd) {
            continue;
        }

        /* 校验通过 */
        result.raw_angle = angle;
        result.status    = status;
        result.crc       = crc_rcvd;
        result.crc_valid = 1U;
        result.angle_deg = (float)angle * MT6701_ANGLE_SCALE;
        last_good = result;
        had_good  = 1;
        return result;
    }

    /* 所有重试均失败, 返回上次有效值 (若有) 并标记 invalid */
    if (had_good) {
        last_good.crc_valid = 0;
        return last_good;
    }

    /* 从未读到有效数据, 返回零 */
    result.raw_angle = 0;
    result.angle_deg = 0.0f;
    result.status    = 0;
    result.crc       = 0;
    result.crc_valid = 0;
    return result;
}
