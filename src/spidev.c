/********************************** (C) COPYRIGHT *******************************
 * File Name          : Main.c
 * Author             : WCH
 * Version            : V1.0
 * Date               : 2020/08/06
 * Description        : SPI0演示 Master/Slave 模式数据收发
 *********************************************************************************
 * Copyright (c) 2021 Nanjing Qinheng Microelectronics Co., Ltd.
 * Attention: This software (modified or not) and binary are used for 
 * microcontroller manufactured by Nanjing Qinheng Microelectronics.
 *******************************************************************************/

#include "CH58x_common.h"

__attribute__((aligned(4))) UINT8 spiBuff[] = {1, 2, 3, 4, 5, 6, 7, 8, 9, 0, 1, 2, 3, 4, 5, 6};
__attribute__((aligned(4))) UINT8 spiBuffrev[16];


void spi0_iodir_tx() {
    R8_SPI0_CTRL_CFG |= RB_SPI_MISO_OE;
    GPIOB_ModeCfg(GPIO_Pin_15, GPIO_ModeOut_PP_5mA);
}

void spi0_iodir_rx() {
    R8_SPI0_CTRL_CFG &= ~RB_SPI_MISO_OE;
    GPIOB_ModeCfg(GPIO_Pin_15, GPIO_ModeIN_Floating);
}

void spidev()
{
    UINT8 i;

    /* 主机模式 */
    PRINT("1.spi0 mul master mode send data ...\n");
    DelayMs(100);

    GPIOPinRemap(ENABLE, RB_PIN_SPI0);

    /* SPI 0 */
    GPIOB_SetBits(GPIO_Pin_4);
    GPIOB_ModeCfg(GPIO_Pin_4 | GPIO_Pin_13 | GPIO_Pin_15, GPIO_ModeOut_PP_5mA);
    
    SPI0_MasterDefInit();
    R8_SPI0_CLOCK_DIV = 60;
    R8_SPI0_CTRL_MOD = RB_SPI_ALL_CLEAR;
    // Master, 2-Wire (Output with MISO), Mode 3 (SCK idle high)
    R8_SPI0_CTRL_MOD = RB_SPI_MISO_OE | RB_SPI_SCK_OE | RB_SPI_MST_SCK_MOD | RB_SPI_2WIRE_MOD;
    R8_SPI0_CTRL_CFG |= RB_SPI_AUTO_IF;
    R8_SPI0_CTRL_CFG &= ~RB_SPI_DMA_ENABLE;



    // 单字节发送
    spi0_iodir_tx();
    GPIOB_ResetBits(GPIO_Pin_4);
    SPI0_MasterSendByte(0x55);
    DelayMs(1);
    spi0_iodir_rx();
    i = SPI0_MasterRecvByte();
    GPIOB_SetBits(GPIO_Pin_4);
    DelayMs(2);
    PRINT("RX %02x\n", i);

    // FIFO 连续发送
    spi0_iodir_tx();
    GPIOB_ResetBits(GPIO_Pin_4);
    SPI0_MasterTrans(spiBuff, 12);
    DelayMs(2);
    spi0_iodir_rx();
    SPI0_MasterRecv(spiBuffrev, 12);
    GPIOB_SetBits(GPIO_Pin_4);
    DelayMs(2);

    // DMA 连续发送
    spi0_iodir_tx();
    GPIOB_ResetBits(GPIO_Pin_4);
    SPI0_MasterDMATrans(spiBuffrev, 12);
    DelayMs(2);
    spi0_iodir_rx();
    SPI0_MasterDMARecv(spiBuffrev, 12);
    GPIOB_SetBits(GPIO_Pin_4);

    PRINT("END ...\n");
}
