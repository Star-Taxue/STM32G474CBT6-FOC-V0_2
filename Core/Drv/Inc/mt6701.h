#ifndef __MT6701_H
#define __MT6701_H

#include <stdint.h>

typedef struct {
    uint16_t raw_angle;
    float angle_deg;
    uint8_t status;
    uint8_t crc;
    uint8_t crc_valid;
} mt6701_data_t;

void mt6701_init(void);
mt6701_data_t mt6701_read_angle(void);

#endif
