#include "i2s_esp32.h"
#include "A_Weighting.h"

// Pines I2S del INMP441
#define I2S_WS 25
#define I2S_SD 33
#define I2S_SCK 32
#define I2S_PORT I2S_NUM_0

// Parámetros I2S
#define SAMPLE_RATE 48000
#define BLOCK_SIZE (SAMPLE_RATE/1000) // Cantidad de muestras para un 1ms
#define MIC_SEN -26
#define EIN 33//db

// Compensacion Temporal.
#define F_MODE 0
#define S_MODE 1
#define F_MODE_SAMPLES (SAMPLE_RATE*125)/1000
#define S_MODE_SAMPLES SAMPLE_RATE

void setup() {
  Serial.begin(115200);
  i2s_install(SAMPLE_RATE, BLOCK_SIZE);
  i2s_setpin(I2S_SCK, I2S_WS, I2S_SD);
  i2s_start(I2S_PORT);
  delay(500);
}

bool INMP441_Read(float* db, uint8_t t_mode, float db_cal)
{
  bool ret = false;
  static uint16_t samples = 0;
  double sample_f;
  uint16_t samplesCant = (t_mode == F_MODE) ? F_MODE_SAMPLES : S_MODE_SAMPLES; // Determina la cantidad de muestras que debe ponderar.
  static double sum_power = 0.0;
  double mili_power = 0.0;
  double filtered_sample;
  float dbSPL;
  static int32_t sampleBuffer[BLOCK_SIZE];
  static double powerBuffer[S_MODE_SAMPLES / BLOCK_SIZE];
  static bool powerBufferFull = false;
  static uint16_t powerBufferIndex = 0;
  size_t bytesRead;
  uint32_t samplesRead;
  uint16_t i = 0;


  esp_err_t res = i2s_read(I2S_PORT, sampleBuffer, sizeof(sampleBuffer), &bytesRead, portMAX_DELAY);
  if (res == ESP_OK)
  {
    samplesRead = bytesRead / sizeof(int32_t);
    while ((i < samplesRead) && (samples <= samplesCant)) 
    {
      sample_f = (sampleBuffer[i] >> 8)/8388608.0f; // La data llega en los 24 MSB. Normalizo a 1
      filtered_sample =  compA(sample_f); // Aplico compensacion tipo A. 
      mili_power += ((double)filtered_sample  * filtered_sample); 
      samples++;
      ++i;
    }
    if(samples >= samplesCant)
    {
      samples = 0;
      powerBufferFull = true;
    }
    if (powerBufferFull) {sum_power -= powerBuffer[powerBufferIndex];}
    powerBuffer[powerBufferIndex] = mili_power;
    sum_power += powerBuffer[powerBufferIndex];
    powerBufferIndex++;
    powerBufferIndex %= samplesCant/BLOCK_SIZE;

    if (powerBufferFull)
    {
      dbSPL = sqrtf(sum_power / samplesCant);
      dbSPL = 94.0f + 20.0f * log10f(dbSPL) - MIC_SEN + db_cal;
      dbSPL = (dbSPL < EIN) ? EIN : dbSPL;
      (*db) = dbSPL;
      ret = true;
    }
  }
  return ret;
}

void loop() 
{
  float dbSPL;
  if ( INMP441_Read(&dbSPL, S_MODE, 0) )
  {
    Serial.print(120);
    Serial.print(" ");
    Serial.print(0);
    Serial.print(" ");
    Serial.println(dbSPL);
  }
}
