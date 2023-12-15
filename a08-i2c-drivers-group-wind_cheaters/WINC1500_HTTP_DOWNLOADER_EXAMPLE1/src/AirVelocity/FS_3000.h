/*
 * FS_3000.h
 *
 * Created: 12/3/2023 7:04:30 PM
 *  Author: praty
 */ 

#ifndef FS_3000_H_
#define FS_3000_H_

#include <stdio.h>
#include <stdint.h>
#include "i2c_master.h"

#define FS3000_TO_READ 5 // Number of Bytes Read:
                         // [0]Checksum, [1]data high, [2]data low,
                         // [3]generic checksum data, [4]generic checksum data

#define FS3000_DEVICE_ADDRESS 0x28 // Note, the FS3000 does not have an adjustable address.
#define AIRFLOW_RANGE_7_MPS 0x00   // FS3000-1005 has a range of 0-7.23 meters per second
#define AIRFLOW_RANGE_15_MPS 0x01  // FS3000-1015 has a range of 0-15 meters per second

bool FS3000_begin(void); // Initialize I2C Port in here
bool FS3000_isConnected(void);
uint16_t FS3000_readRaw(void);
float FS3000_readMetersPerSecond(void);
float FS3000_readMilesPerHour(void);
void FS3000_setRange(uint8_t range);

void FS3000_readData(uint8_t *buffer_in);

/*
 * @param data_in: 5 Bytes Buffer
 * @param debug: print debug info (default: false)
 */
bool FS3000_checksum(uint8_t *data_in, bool debug);
void FS3000_printHexByte( uint8_t x);

#endif /* FS_3000_H_ */