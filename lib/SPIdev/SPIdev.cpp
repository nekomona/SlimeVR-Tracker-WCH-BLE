#include "SPIdev.h"

#include <CH58x_common.h>

void SPIdev::initialize() {
    GPIOPinRemap(ENABLE, RB_PIN_SPI0);

    /* SPI 0 */
    GPIOB_SetBits(GPIO_Pin_4 | GPIO_Pin_13 | GPIO_Pin_14);
    GPIOB_ModeCfg(GPIO_Pin_4 | GPIO_Pin_13 | GPIO_Pin_14 | GPIO_Pin_15, GPIO_ModeOut_PP_5mA);
    GPIOA_SetBits(GPIO_Pin_15);
    GPIOA_ModeCfg(GPIO_Pin_15, GPIO_ModeOut_PP_5mA);
    
    R8_SPI0_CLOCK_DIV = 15;
    R8_SPI0_CTRL_MOD = RB_SPI_ALL_CLEAR;
    // Master, 2-Wire (Output with MISO), Mode 3 (SCK idle high)
    R8_SPI0_CTRL_MOD = RB_SPI_MISO_OE | RB_SPI_SCK_OE | RB_SPI_MST_SCK_MOD | RB_SPI_2WIRE_MOD;
    R8_SPI0_CTRL_CFG |= RB_SPI_AUTO_IF;
    R8_SPI0_CTRL_CFG &= ~RB_SPI_DMA_ENABLE;
}

void spi0_iodir_tx() {
    R8_SPI0_CTRL_CFG |= RB_SPI_MISO_OE;
    GPIOB_ModeCfg(GPIO_Pin_15, GPIO_ModeOut_PP_5mA);
}

void spi0_iodir_rx() {
    R8_SPI0_CTRL_CFG &= ~RB_SPI_MISO_OE;
    GPIOB_ModeCfg(GPIO_Pin_15, GPIO_ModeIN_Floating);
}

void spi0_set_cs(uint8_t devIndex) {
    switch(devIndex) {
    case 0:
        GPIOB_ResetBits(GPIO_Pin_4);
        break;
    case 1:
        GPIOB_ResetBits(GPIO_Pin_14);
        break;
    case 2:
    default:
        GPIOA_ResetBits(GPIO_Pin_15);
    }
}

void spi0_clear_cs(uint8_t devIndex) {
    switch(devIndex) {
    case 0:
        GPIOB_SetBits(GPIO_Pin_4);
        break;
    case 1:
        GPIOB_SetBits(GPIO_Pin_14);
        break;
    case 2:
    default:
        GPIOA_SetBits(GPIO_Pin_15);
    }
}

bool SPIdev::writeByte(uint8_t devIndex, uint8_t regAddr, uint8_t wrData) {
    spi0_iodir_tx();
    spi0_set_cs(devIndex);
    SPI0_MasterSendByte(regAddr);
    SPI0_MasterSendByte(wrData);
    spi0_clear_cs(devIndex);
    return true;
}

int8_t SPIdev::readByte(uint8_t devIndex, uint8_t regAddr, uint8_t *rdData) {
    spi0_iodir_tx();
    spi0_set_cs(devIndex);
    SPI0_MasterSendByte(0x80 | regAddr);
    spi0_iodir_rx();
    *rdData = SPI0_MasterRecvByte();
    spi0_clear_cs(devIndex);
    spi0_iodir_tx();
    return 1;
}

int8_t SPIdev::readBytes(uint8_t devIndex, uint8_t regAddr, uint8_t rdLen, uint8_t *rdBuf) {
    spi0_iodir_tx();
    spi0_set_cs(devIndex);
    SPI0_MasterSendByte(0x80 | regAddr);
    spi0_iodir_rx();
    SPI0_MasterRecv(rdBuf, rdLen);
    spi0_clear_cs(devIndex);
    spi0_iodir_tx();
    return (int8_t)rdLen;
}
