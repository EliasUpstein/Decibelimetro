#include <EEPROM.h>
#include <driver/i2s.h>
#include "driver/adc.h"
// Pines I2S del INMP441
#define I2S_WS 25
#define I2S_SD 33
#define I2S_SCK 32
#define I2S_PORT I2S_NUM_0

// Parámetros I2S
#define SAMPLE_RATE 48000
#define BLOCK_SIZE (SAMPLE_RATE/1000) // Cantidad de muestras para un 1ms

// Compensacion Temporal.
#define F_MODE 0
#define S_MODE 1
#define F_MODE_SAMPLES (SAMPLE_RATE*125)/1000
#define S_MODE_SAMPLES SAMPLE_RATE

// Compensacion Frecuencial.
#define MAX_IIR_ORDER 10  // Define el orden máximo que soportará tu filtro

#define EEPROM_SIZE 512  
#define EEPROM_FIRST_ADDR 0

typedef struct {
    int order;          // Orden actual del filtro
    double b[MAX_IIR_ORDER + 1]; // Coeficientes del numerador
    double a[MAX_IIR_ORDER + 1];   // Coeficientes del denominador
    double x_history[MAX_IIR_ORDER + 1]; // Historial de entradas
    double y_history[MAX_IIR_ORDER + 1]; // Historial de salidas
} IIRFilter;

// Filtro compensacion A.
IIRFilter filter_compA 
{
    .order = 6, // El orden real de tu filtro
    .b = {0.234301792299513, -0.468603584599026, -0.234301792299513,
          0.937207169198054, -0.234301792299515, -0.468603584599025, 0.234301792299513},
    .a = {1.000000000000000, -4.113043408775871, 6.553121752655047,
          -4.990849294163381, 1.785737302937573, -0.246190595319487, 0.011224250033231},
    .x_history = {0},
    .y_history = {0}
};

// Filtro de calibracion.
IIRFilter filter_cal;

// Recupera el filtro de la EEPROM
void get_Filter(IIRFilter *filter) 
{
  EEPROM.begin(EEPROM_SIZE);
  EEPROM.get(EEPROM_FIRST_ADDR, *filter);
    for (int i = 0; i <= filter->order ; i++) {
        filter->x_history[i] = 0.0;
        filter->y_history[i] = 0.0;
    }
  EEPROM.end();
  return;
}

// Procesa una muestra de entrada
double iir_process(IIRFilter *filter, double input) 
{
      // Calcular nueva salida
    double y = 0.0;
    // Desplazar historial de entradas
    for (int i = filter->order; i > 0; i--) 
    {
        filter->x_history[i] = filter->x_history[i - 1];
    }
    filter->x_history[0] = input;
    
    // Desplazar historial de salidas
    for (int i = filter->order; i > 0; i--)
    {
        filter->y_history[i] = filter->y_history[i - 1];
    }
    
    // Parte del numerador (feedforward)
    for (int i = 0; i <= filter->order; i++) 
    {
        y += filter->b[i] * filter->x_history[i];
    }
    
    // Parte del denominador (feedback)
    for (int i = 1; i <= filter->order; i++) 
    {
        y -= filter->a[i] * filter->y_history[i];
    }
    
    // Normalizar por a0
    y /= filter->a[0];
    
    // Guardar la salida actual
    filter->y_history[0] = y;
    
    return y;
}

// Inicializa el I2S.
void i2s_install()
{
  const i2s_config_t i2s_config = {
    .mode = (i2s_mode_t)(I2S_MODE_MASTER | I2S_MODE_RX),
    .sample_rate = SAMPLE_RATE,
    .bits_per_sample = I2S_BITS_PER_SAMPLE_24BIT,
    .channel_format = I2S_CHANNEL_FMT_ONLY_LEFT,
    .communication_format = I2S_COMM_FORMAT_STAND_I2S,
    .intr_alloc_flags = 0,
    .dma_buf_count = 8,
    .dma_buf_len = BLOCK_SIZE,
    .use_apll = false,
    .tx_desc_auto_clear = false,
    .fixed_mclk = 0
  };
  i2s_driver_install(I2S_PORT, &i2s_config, 0, NULL);
}

// Configura pines I2S
void i2s_setpin()
{
  const i2s_pin_config_t pin_config = {
    .bck_io_num = I2S_SCK,
    .ws_io_num = I2S_WS,
    .data_out_num = -1,
    .data_in_num = I2S_SD
  };
  i2s_set_pin(I2S_PORT, &pin_config);
}

// Guarda el filtro en EEPROM
void save_Filter(IIRFilter *filter) {
  EEPROM.begin(EEPROM_SIZE);
  EEPROM.put(EEPROM_FIRST_ADDR, *filter);
  EEPROM.commit();
  EEPROM.end();
}

// Lee línea de Serial y carga filtro
bool receive_FilterFromSerial(IIRFilter *filter) 
{
  uint16_t index = 0;
  uint16_t lastIndex = 0;
  uint16_t filterIndex = 0;

  if (!Serial.available()) return false;

  String input = Serial.readStringUntil('\n'); // Ejemplo: "6;0.1;0.2;...;1.0;-0.5;..."
  input.trim();

  while ((index = input.indexOf(';', lastIndex)) != -1 && filterIndex < 2*(MAX_IIR_ORDER+1)+1)
  {
    String token = input.substring(lastIndex, index);
    token.trim();

    if (filterIndex == 0) 
    {
      filter->order = token.toInt();
      if (filter->order > MAX_IIR_ORDER) return false; // Protección
    }
    else if (filterIndex <= filter->order+1)
    {
      filter->b[filterIndex-1] = token.toDouble(); // Coef b
    }
    else if (filterIndex <= 2*(filter->order + 1))
    {
      filter->a[filterIndex-filter->order-2] = token.toDouble(); // Coef a
    }
    filterIndex++;
    lastIndex = index + 1;
  }

  // Reset de históricos
  for (int i = 0; i <= filter->order; i++) {
    filter->x_history[i] = 0.0;
    filter->y_history[i] = 0.0;
  }

  save_Filter(filter); // Guardar en EEPROM
  return true;
}

void setup() {
  Serial.begin(115200);
  i2s_install();
  i2s_setpin();
  i2s_start(I2S_PORT);
  get_Filter(&filter_cal); 
  delay(500);
}

bool INMP441_Read(float* db, uint8_t t_mode, uint8_t f_mode, IIRFilter* comp_filter, IIRFilter* cal_filter)
{
  bool ret = false;
  static uint16_t samples = 0;
  uint16_t samplesCant = (t_mode == F_MODE) ? F_MODE_SAMPLES : S_MODE_SAMPLES;
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
      sampleBuffer[i] = (sampleBuffer[i] >> 8);
      filtered_sample =  iir_process(comp_filter, sampleBuffer[i]);
      filtered_sample = iir_process(cal_filter, filtered_sample);
      mili_power += (double)filtered_sample  * filtered_sample;
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
      dbSPL = 94.0f + 20.0f * log10f(dbSPL / 420426.0f);
      (*db) = dbSPL;
      ret = true;
    }
  }
  return ret;
}

void loop() 
{
  receive_FilterFromSerial(&filter_cal); 
  float dbSPL;
  if ( INMP441_Read(&dbSPL, S_MODE, 0, &filter_compA, &filter_cal) )
  {
    Serial.print(120);
    Serial.print(" ");
    Serial.print(0);
    Serial.print(" ");
    Serial.println(dbSPL);
  }
}
