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

typedef struct {
    int order;          // Orden actual del filtro
    const double* b;  // Coeficientes del numerador
    const double* a;   // Coeficientes del denominador
    double x_history[MAX_IIR_ORDER + 1]; // Historial de entradas
    double y_history[MAX_IIR_ORDER + 1]; // Historial de salidas
} IIRFilter;

// Filtro compensacion A.
IIRFilter filter_compA;

static const double filter_b_compA[7] =  {0.234301792299513, -0.468603584599026, -0.234301792299513, 0.937207169198054, -0.234301792299515, -0.468603584599025, 0.234301792299513};
static const double filter_a_compA[7] =  {1.000000000000000, -4.113043408775871, 6.553121752655047, -4.990849294163381, 1.785737302937573, -0.246190595319487, 0.011224250033231};


// Filtro de calibracion.
IIRFilter filter_cal;

static const double filter_b_cal[11] = {31.1731764721, -11.1958078377, 17.3522965260, -31.5198533231, 13.3427037312, -8.0573960575, 6.1766219147, -7.7137757576, 9.8510863270, 2.0064312072, 13.8922132609};
static const double filter_a_cal[11] = {1.0000000000, 0.9629113099, 0.9215213630, 0.7989663617, 0.8075455208, 0.5282350058, 0.5986875691, 0.4440402441, 0.4008130649, 0.2260699122, 0.0850956565};

// Inicializa el filtro con coeficientes
void iir_init(IIRFilter *filter, int order, const double *b, const double *a) 
{
    filter->order = order;
    filter->b = b;
    filter->a = a;
    // Inicializar historiales a cero
    for (int i = 0; i <= order; i++) {
        filter->x_history[i] = 0.0;
        filter->y_history[i] = 0.0;
    }
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

void setup() {
  Serial.begin(115200);
  i2s_install();
  i2s_setpin();
  i2s_start(I2S_PORT);

  iir_init(&filter_compA, 6, filter_b_compA, filter_a_compA);
  iir_init(&filter_cal, 10, filter_b_cal, filter_a_cal);
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
