#include <A_Weighting.h>

// Filtros para formar la compensacion A
IIRFilter H_1 
{    
    .order = 1,
    .b = {1, 0},
    .a = {1, -0.202148475},
    .x_history = {0},
    .y_history = {0}
};
IIRFilter H_2 
{
    .order = 1,
    .b = {1, 0.2998046875},
    .a = {1, -0.202148475},
    .x_history = {0},
    .y_history = {0}
};
IIRFilter H_3 
{
    .order = 1,
    .b = {1, -1},
    .a = {1, -0.986328125},
    .x_history = {0},
    .y_history = {0}
};
IIRFilter H_4 
{
    .order = 1,
    .b = {1, -1},
    .a = {1, -0.91015625},
    .x_history = {0},
    .y_history = {0}
};
IIRFilter H_5 
{
    .order = 1,
    .b = {1, -1},
    .a = {1, -0.9970703125},
    .x_history = {0},
    .y_history = {0}
};
IIRFilter H_6 
{
    .order = 1,
    .b = {1, -1},
    .a = {1, -0.9970703125},
    .x_history = {0},
    .y_history = {0}
};


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

// Compensacion TipoA
double compA(double input)
{
  double y;
  y = iir_process(&H_1, input); 
  y = iir_process(&H_2, y); 
  y = iir_process(&H_3, y);
  y = iir_process(&H_4, y);
  y = iir_process(&H_5, y);
  y = iir_process(&H_6, y);
  return y;    
} 