#include "app_uart.h"
#include <stdio.h>

void uart_send_channels(float ch1, float ch2, float ch3, float ch4)
{
    printf("channels: %d,%d,%d,%d\n", (int)ch1, (int)ch2, (int)ch3, (int)ch4);
}
