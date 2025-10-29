#include "flash_mem.h"

void get_cal(int16_t* cal) 
{
  EEPROM.begin(EEPROM_SIZE);
  EEPROM.get(EEPROM_FIRST_ADDR, *cal);
  EEPROM.end();
  return;
}

void save_cal(int16_t cal) 
{
  EEPROM.begin(EEPROM_SIZE);
  EEPROM.put(EEPROM_FIRST_ADDR, cal);
  EEPROM.commit();
  EEPROM.end();
  return;
}