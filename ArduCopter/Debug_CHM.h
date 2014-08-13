#ifndef __DEBUG_CHM_H__
#define __DEBUG_CHM_H__

#include "defines.h"

void Log_Write_Data(uint8_t id, int16_t value);
void Log_Write_Data(uint8_t id, uint16_t value);
void Log_Write_Data(uint8_t id, int32_t value);
void Log_Write_Data(uint8_t id, uint32_t value);
void Log_Write_Data(uint8_t id, float value);

#endif //__DEBUG_CHM_H__