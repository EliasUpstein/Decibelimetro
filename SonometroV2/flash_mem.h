#include <EEPROM.h>

#define EEPROM_SIZE 512  
#define EEPROM_FIRST_ADDR 0

void get_cal(float cal);
void save_cal(float cal);