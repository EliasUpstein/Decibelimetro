#include "flash_mem.h"

void get_cal(float cal) 
{
  EEPROM.begin(EEPROM_SIZE);
  EEPROM.get(EEPROM_FIRST_ADDR, cal);
  EEPROM.end();
  return;
}

void save_cal(float cal) 
{
  EEPROM.begin(EEPROM_SIZE);
  EEPROM.put(EEPROM_FIRST_ADDR, cal);
  EEPROM.commit();
  EEPROM.end();
  return;
}