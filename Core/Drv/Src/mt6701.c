#include "mt6701.h"
#include "main.h"
#include "spi.h"
#include "stm32g4xx_hal.h"

/* 片选信号宏定义 */

#define MT6701_CSN_PIN      CS_Pin
#define MT6701_CSN_PORT     CS_GPIO_Port

#define MT6701_CSN_HIGH()   HAL_GPIO_WritePin(MT6701_CSN_PORT, MT6701_CSN_PIN, GPIO_PIN_SET)
#define MT6701_CSN_LOW()    HAL_GPIO_WritePin(MT6701_CSN_PORT, MT6701_CSN_PIN, GPIO_PIN_RESET)

static void delay_us(uint32_t us)
{
    uint32_t start = DWT->CYCCNT;
    uint32_t ticks = us * (SystemCoreClock / 1000000);
    while ((DWT->CYCCNT - start) < ticks);
}

void mt6701_init(void)
{
    CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;
    DWT->CYCCNT = 0;
    DWT->CTRL |= DWT_CTRL_CYCCNTENA_Msk;

    MT6701_CSN_HIGH();

    HAL_Delay(100);
}

mt6701_data_t mt6701_read_angle(void)
{
    uint8_t buf[3] = {0};
    mt6701_data_t data = {0};

    MT6701_CSN_LOW();
    delay_us(2);

    HAL_SPI_Receive(&hspi1, buf, 3, 100);

    MT6701_CSN_HIGH();

    uint32_t raw = ((uint32_t)buf[0] << 16) |
                   ((uint32_t)buf[1] << 8)  |
                   ((uint32_t)buf[2]);

    uint16_t angle  = (raw >> 10) & 0x3FFF;
    uint8_t  status = (raw >> 6)  & 0x0F;
    uint8_t  crc    =  raw        & 0x3F;

    data.raw_angle = angle;
    data.status    = status;
    data.crc       = crc;
    data.angle_deg = (float)angle * 360.0f / 16384.0f;
    data.crc_valid = 1;

    return data;
}
