#ifndef IIRFILTER_H
#define IIRFILTER_H
typedef struct 
{
    uint8_t order;
    double b[2]; // Coeficientes del numerador
    double a[2];   // Coeficientes del denominador
    double x_history[2]; // Historial de entradas
    double y_history[2]; // Historial de salidas
} IIRFilter;
#endif 

double iir_process(IIRFilter *filter, double input);
double compA(double input);