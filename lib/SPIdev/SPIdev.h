#ifndef __SPIDEV_H
#define __SPIDEV_H

#include <stdio.h>

class SPIdev {
public:
    static void initialize();
    static bool writeByte(uint8_t devIndex, uint8_t regAddr, uint8_t wrData);
    static int8_t readByte(uint8_t devIndex, uint8_t regAddr, uint8_t *rdData);
    static int8_t readBytes(uint8_t devIndex, uint8_t regAddr, uint8_t rdLen, uint8_t *rdBuf);
};

#endif
