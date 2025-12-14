#include "i2s_esp32.h"
#include "A_Weighting.h"
#include <Adafruit_GFX.h>
#include <Adafruit_ST7735.h>
#include <SPI.h>
#include "flash_mem.h"

// Pines TFT
#define TFT_CS   5
#define TFT_DC   3
#define TFT_RST 13
Adafruit_ST7735 tft = Adafruit_ST7735(TFT_CS, TFT_DC, TFT_RST);

#define BG_COLOR ST7735_BLACK
#define DB_COLOR ST7735_WHITE
#define MM_COLOR ST7735_GREEN
#define TT_COLOR ST7735_RED 
#define T2_COLOR ST7735_BLUE   
#define CAL_COLOR ST7735_CYAN  

// Pines I2S INMP441
#define I2S_WS 25
#define I2S_SD 33
#define I2S_SCK 32
#define I2S_PORT I2S_NUM_0

// Parámetros I2S
#define SAMPLE_RATE 48000
#define BLOCK_SIZE (SAMPLE_RATE/250) // 1ms
#define MIC_SEN -26
#define EIN 33 // db

// Compensacion Temporal.
#define F_MODE 0
#define S_MODE 1
#define F_MODE_SAMPLES (SAMPLE_RATE*125)/1000
#define S_MODE_SAMPLES SAMPLE_RATE

// Botones
#define BTN_UP   14
#define BTN_DOWN 27

//Led
#define LED_UMBRAL 26
#define dB_UMBRAL 85

#define LECTURA_S 0
#define CAL_S 1


uint8_t state = LECTURA_S;

unsigned long lastDisplay = 0;
const unsigned long displayInterval = 500; // ms


int16_t cal; // cal*10 
float dbSPL;
float dbMAX = 0;
float dbMIN = 120;

// --- Setup ---
void setup() {
  Serial.begin(115200);

  pinMode(BTN_UP, INPUT_PULLUP);
  pinMode(BTN_DOWN, INPUT_PULLUP);
  pinMode(LED_UMBRAL, OUTPUT);

  i2s_install(SAMPLE_RATE, BLOCK_SIZE);
  i2s_setpin(I2S_SCK, I2S_WS, I2S_SD);
  i2s_start(I2S_PORT);

  tft.initR(INITR_BLACKTAB);
  tft.setRotation(0);
  delay(2500);
  tft.fillScreen(BG_COLOR);
  tft.setTextColor(TT_COLOR);
  tft.setTextSize(2);
  tft.setCursor(10, 20);
  tft.println("DECIBELES");

  get_cal(&cal);
}

// --- Lectura y cálculo RMS ---
bool INMP441_Read(float* db, int16_t db_cal) 
{
  bool ret = false;
  static uint16_t samples = 0;
  double sample_f;
  uint16_t samplesCant = S_MODE_SAMPLES;
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
      sample_f = (sampleBuffer[i] >> 8) / 8388608.0f; 
      filtered_sample = compA(sample_f);
      mili_power += (filtered_sample * filtered_sample);
      samples++;
      ++i;
    }
    if(samples >= samplesCant)
    {
      samples = 0;
      powerBufferFull = true;
    }
    if (powerBufferFull) { sum_power -= powerBuffer[powerBufferIndex]; }
    powerBuffer[powerBufferIndex] = mili_power;
    sum_power += powerBuffer[powerBufferIndex];
    powerBufferIndex++;
    powerBufferIndex %= samplesCant / BLOCK_SIZE;

    if (powerBufferFull)
    {
      dbSPL = sqrtf(sum_power / samplesCant);
      dbSPL = 94.0f + 20.0f * log10f(dbSPL) - MIC_SEN + ((float)db_cal/10);
      dbSPL = (dbSPL < EIN) ? EIN : dbSPL;
      (*db) = dbSPL;
      ret = true;
    }
  }
  return ret;
}

void tft_refresh(float dbSPL)
{
      tft.fillRect(0, 60, 100, 60, BG_COLOR);
      tft.setCursor(10, 60);
      tft.setTextColor(DB_COLOR);
      tft.setTextSize(3);
      tft.print(dbSPL, 1);
      tft.setTextSize(2);
      tft.print(" dB");
      return;
}

// --- Loop ---
void loop() {
  INMP441_Read(&dbSPL, cal);
  bool upPressed = !digitalRead(BTN_UP);
  bool downPressed = !digitalRead(BTN_DOWN);

  // Detección de modo calibración
  static bool bothPressedLast = false;
  bool bothPressedNow = upPressed && downPressed;

  switch(state)
  {
    case LECTURA_S:
      if(bothPressedNow && !bothPressedLast)
      {
        tft.fillScreen(BG_COLOR);
        tft.setTextColor(T2_COLOR);
        tft.setTextSize(2);
        tft.setCursor(10, 20);
        tft.println("CALIBRATE");
        state = CAL_S;
      }
      else if (millis() - lastDisplay >= displayInterval) 
      {
        lastDisplay = millis();
        if (upPressed)
        {
          dbMAX = 0;
          dbMIN = 120;
        }
        if (dbSPL > dbMAX) dbMAX = dbSPL;
        if (dbSPL < dbMIN) dbMIN = dbSPL;
        tft_refresh(dbSPL);
        tft.fillRect(10, 100, 140, 30, BG_COLOR); 
        tft.setCursor(10, 100);
        tft.setTextColor(MM_COLOR);
        tft.setTextSize(2);
        tft.print(dbMAX, 1);
        tft.print(" ");
        tft.print(dbMIN, 1);

        if(dbSPL >= dB_UMBRAL)
          digitalWrite(LED_UMBRAL, HIGH);
        else
          digitalWrite(LED_UMBRAL, LOW);
      }
    break;
    case CAL_S:
      if(bothPressedNow && !bothPressedLast)
      {
        save_cal(cal);
        tft.fillScreen(BG_COLOR);
        tft.setTextColor(TT_COLOR);
        tft.setTextSize(2);
        tft.setCursor(10, 20);
        tft.println("DECIBELES");
        state = LECTURA_S;
      }
      else
      {


        if (millis() - lastDisplay >= displayInterval) 
        {
          if (upPressed)  cal++;
          if (downPressed) cal--;
          lastDisplay = millis();
          tft_refresh(dbSPL);
          tft.fillRect(10, 100, 140, 30, BG_COLOR); 
          tft.setCursor(10, 100);
          tft.setTextColor(CAL_COLOR);
          tft.setTextSize(2);
          tft.print("Cal: ");
          tft.print(( (float) cal )/ 10.0f, 1);
        }

      }
    break;
    default:
    break;
  }
  bothPressedLast = bothPressedNow;
}
