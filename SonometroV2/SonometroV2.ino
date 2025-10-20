#include "i2s_esp32.h"
#include "A_Weighting.h"
#include <Adafruit_GFX.h>
#include <Adafruit_ST7735.h>
#include <SPI.h>
#include "flash_mem.h"
#include "utn_logo.h"

// Pines TFT
#define TFT_CS   5
#define TFT_DC   3
#define TFT_RST 13
Adafruit_ST7735 tft = Adafruit_ST7735(TFT_CS, TFT_DC, TFT_RST);

// Pines I2S INMP441
#define I2S_WS 25
#define I2S_SD 33
#define I2S_SCK 32
#define I2S_PORT I2S_NUM_0

// Parámetros I2S
#define SAMPLE_RATE 48000
#define BLOCK_SIZE (SAMPLE_RATE/250) // Cantidad de muestras para un 1ms
#define MIC_SEN -26
#define EIN 33//db

// Compensacion Temporal.
#define F_MODE 0
#define S_MODE 1
#define F_MODE_SAMPLES (SAMPLE_RATE*125)/1000
#define S_MODE_SAMPLES SAMPLE_RATE


unsigned long lastDisplay = 0;
const unsigned long displayInterval = 500; // 1s
float cal;

// --- Setup ---
void setup() {
  Serial.begin(115200);
  i2s_install(SAMPLE_RATE, BLOCK_SIZE);
  i2s_setpin(I2S_SCK, I2S_WS, I2S_SD);
  i2s_start(I2S_PORT);

  tft.initR(INITR_BLACKTAB);
  tft.setRotation(1);

  delay(2500);
  tft.fillScreen(ST77XX_BLACK);

  tft.setTextColor(ST77XX_WHITE);
  tft.setTextSize(2);
  tft.setCursor(10, 20);
  tft.println("SONOMETRO");

  get_cal(cal);
}

// --- Lectura y cálculo RMS ---
bool INMP441_Read(float* db, float db_cal) 
{
  bool ret = false;
  static uint16_t samples = 0;
  double sample_f;
  uint16_t samplesCant = S_MODE_SAMPLES; // Determina la cantidad de muestras que debe ponderar.
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

// --- Loop ---
void loop() {
  float dbSPL;
  INMP441_Read(&dbSPL, cal); // siempre acumulando muestras

  // Actualizar pantalla cada 1s
  if(millis() - lastDisplay >= displayInterval){
    lastDisplay = millis();
    

    tft.fillRect(0, 60, 128, 30, ST77XX_BLACK);
    tft.setCursor(10, 70);
    tft.setTextSize(3);
    tft.setTextColor(ST77XX_GREEN);
    tft.print(dbSPL,1);
    tft.setTextSize(2);
    tft.setTextColor(ST77XX_WHITE);
    tft.print(" dB");
  }
}
