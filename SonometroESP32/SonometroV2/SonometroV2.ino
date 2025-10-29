#include "i2s_esp32.h"
#include "A_Weighting.h"

//#include <Adafruit_GFX.h>
//#include <Adafruit_ST7735.h>
//#include <SPI.h>

// Pines I2S del INMP441
#define I2S_WS 25
#define I2S_SD 33
#define I2S_SCK 32
#define I2S_PORT I2S_NUM_0

//Pines de la TFT
#define TFT_CS        17
#define TFT_RST       14
#define TFT_DC         2

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

// Posiciones fijas para los valores
#define Y_VALORES  120   // columna donde empiezan los números
#define X_PROMEDIO 4
#define X_MAXIMO   44
#define X_MINIMO   84
#define TEXT3_DESP 23
#define Y_MEDICION 70
#define X_MEDICION 20

// Inicialización de la pantalla
//Adafruit_ST7735 tft = Adafruit_ST7735(TFT_CS, TFT_DC, TFT_RST);

//void initTFT(void);
//void drawEncabezado(void);
//void drawUTN(int cx, int cy, int r);

void setup() {
  //initTFT();

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
  //float dbSPL;

  //static float promedio = 0;
  //static float maximo = 0;
  //static float minimo = 0;

  float dbSPL;
  int db;

  static int promedio = 0;
  static int maximo = 0;
  static int minimo = 0;

  if ( INMP441_Read(&dbSPL, S_MODE, 0) )
  {
    Serial.print(120);
    Serial.print(" ");
    Serial.print(0);
    Serial.print(" ");
    Serial.println(dbSPL);
  }
/*
  db = (int) dbSPL;

  // Mostrar valores actualizados
  tft.setTextColor(ST77XX_YELLOW, ST77XX_BLUE);
  tft.setCursor(X_MEDICION, Y_MEDICION);
  tft.setTextSize(6);
  tft.print(db);
  tft.setTextSize(1);
  tft.setTextColor(ST77XX_YELLOW, ST77XX_BLACK);
  tft.setCursor(tft.getCursorX(), Y_VALORES-10);
  tft.print(" dB");

  tft.setTextSize(1);
  tft.setTextColor(ST77XX_YELLOW, ST77XX_BLACK);
  tft.setCursor(X_PROMEDIO+TEXT3_DESP, Y_VALORES);
  tft.print(promedio);

  tft.setCursor(X_MAXIMO+TEXT3_DESP, Y_VALORES);
  tft.print(maximo);

  tft.setCursor(X_MINIMO+TEXT3_DESP, Y_VALORES);
  tft.print(minimo);

  if (db > maximo) maximo = db;
  if (db < minimo) minimo = db;
  promedio = (promedio + db) / 2;
*/
}

/*
void initTFT(){
  tft.initR(INITR_144GREENTAB); 
  tft.setRotation(2);
  tft.fillScreen(ST77XX_BLACK);

  drawEncabezado();

  tft.setCursor(X_PROMEDIO, Y_VALORES);
  tft.println("AVG:");

  tft.setCursor(X_MAXIMO, Y_VALORES);
  tft.println("MAX:");

  tft.setCursor(X_MINIMO, Y_VALORES);
  tft.println("MIN:");
}

void drawEncabezado(){

  drawUTN(30, 30, 20);
  // --- Texto de cabecera ---
  tft.setCursor(60, 10);
  tft.setTextColor(ST77XX_WHITE);
  tft.setTextSize(1);
  tft.println("UTN.BA");
  
  tft.setCursor(60, 22);
  tft.setTextColor(ST77XX_GREEN);
  tft.println("INGENIERIA");
  tft.setCursor(60, 32);
  tft.println("ELECTRONICA");
  
  tft.setCursor(60, 45);
  tft.setTextColor(ST77XX_GREEN);
  tft.println("MEDIDAS I");

  // --- Titulo ---
  tft.setCursor(1, 60);
  tft.setTextColor(ST77XX_WHITE);
  tft.setTextSize(1);
  tft.println("DECIBELIMETRO DIGITAL");
}

void drawUTN(int cx, int cy, int r) {

  //Semicírculo superior
  tft.drawCircle(cx, cy-(r/2)-1, r/2, ST77XX_WHITE);
  tft.drawCircle(cx, cy-(r/2)-1, (r/2)-1, ST77XX_WHITE);
  tft.drawCircle(cx, cy-(r/2)-1, (r/2)+1, ST77XX_WHITE);     
  tft.fillRect(cx/2, r/4, cx, (cy/2)-1, ST77XX_BLACK);

  //Semicírculo inferior
  tft.drawCircle(cx, cy+(r/2)+2, r/2, ST77XX_WHITE); 
  tft.drawCircle(cx, cy+(r/2)+2, (r/2)-1, ST77XX_WHITE);  
  tft.drawCircle(cx, cy+(r/2)+2, (r/2)+1, ST77XX_WHITE);     
  tft.fillRect(cx/2, (r*2)+1, cx, (cy/2), ST77XX_BLACK);

  tft.fillRect(cx-1, cy-(r/2)-1, 3, r+2, ST77XX_WHITE); // línea vertical
  tft.fillRect(cx-(r/2), cy-1, r+1, 3, ST77XX_WHITE); // línea horizontal

  // Dibujar los círculos completos
  tft.drawCircle(cx, cy, r-2, ST77XX_WHITE);      // círculo exterior
  tft.drawCircle(cx, cy, r, ST77XX_WHITE);  // círculo interior (borde)
}
*/