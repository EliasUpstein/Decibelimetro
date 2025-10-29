#include <driver/i2s.h>
//#include "driver/adc.h"

void i2s_setpin(uint8_t sck, uint8_t ws, uint8_t sd);
void i2s_install(uint32_t sample_rate, uint32_t block_size);