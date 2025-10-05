#include "i2s_esp32.h"

// Configura pines I2S
void i2s_setpin(uint8_t sck, uint8_t ws, uint8_t sd)
{
  const i2s_pin_config_t pin_config = {
    .bck_io_num = sck,
    .ws_io_num = ws,
    .data_out_num = -1,
    .data_in_num = sd
  };
  i2s_set_pin(I2S_PORT, &pin_config);
}

// Inicializa el I2S.
void i2s_install(uint32_t sample_rate, uint32_t block_size)
{
  const i2s_config_t i2s_config = {
    .mode = (i2s_mode_t)(I2S_MODE_MASTER | I2S_MODE_RX),
    .sample_rate = sample_rate,
    .bits_per_sample = I2S_BITS_PER_SAMPLE_24BIT,
    .channel_format = I2S_CHANNEL_FMT_ONLY_LEFT,

    .communication_format = I2S_COMM_FORMAT_STAND_I2S,
    .intr_alloc_flags = 0,
    .dma_buf_count = 8,
    .dma_buf_len = block_size,
    .use_apll = false,
    .tx_desc_auto_clear = false,
    .fixed_mclk = 0
  };
  i2s_driver_install(I2S_PORT, &i2s_config, 0, NULL);
}