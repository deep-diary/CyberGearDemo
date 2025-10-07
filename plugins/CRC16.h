// modbus_crc.h
#ifndef MODBUS_CRC_H
#define MODBUS_CRC_H

#include <stdint.h>
#include <stddef.h>

#ifdef __cplusplus
extern "C" {
#endif

uint16_t modbus_crc_calculate(const uint8_t *data, size_t length);

#ifdef __cplusplus
}
#endif

#endif // MODBUS_CRC_H