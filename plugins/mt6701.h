#ifndef APP_MT6701_H_
#define APP_MT6701_H_
 
#include "main.h"
 
// void SPI2_Init_(void);
// unsigned char SPIx_ReadWrByte(unsigned char byte);
// uint32_t ReadAngle(void);
uint16_t MT6701_GetRawData(void);
int16_t MT6701_GetRawAngle(void);
#endif /* APP_MT6701_H_ */