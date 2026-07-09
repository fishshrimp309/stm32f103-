/**
 * @file    SPI_tx.h
 * @brief   NRF24L01+ 发送端（遥控器）驱动头文件
 *          基于官方例程 nRF24L01P.c/h 移植，使用 HAL SPI 通信
 *
 * 引脚定义：
 *   SPI1: SCK=PA5, MISO=PA6, MOSI=PA7
 *   CSN=PA4, CE=PA3
 */

#ifndef __SPI_TX_H
#define __SPI_TX_H

#include "main.h"

/* ================================================================
 * SPI (nRF24L01+) 命令
 * ================================================================ */
#define NRF_READ_REG    0x00
#define NRF_WRITE_REG   0x20
#define RD_RX_PLOAD     0x61
#define WR_TX_PLOAD     0xA0
#define FLUSH_TX        0xE1
#define FLUSH_RX        0xE2
#define NOP             0xFF

/* ================================================================
 * NRF24L01+ 寄存器地址
 * ================================================================ */
#define CONFIG          0x00
#define EN_AA           0x01
#define EN_RXADDR       0x02
#define SETUP_AW        0x03
#define SETUP_RETR      0x04
#define RF_CH           0x05
#define RF_SETUP        0x06
#define STATUS          0x07
#define OBSERVE_TX      0x08
#define RX_ADDR_P0      0x0A
#define TX_ADDR         0x10
#define RX_PW_P0        0x11
#define FIFO_STATUS     0x17
#define DYNPD           0x1C
#define FEATURE         0x1D

/* ================================================================
 * STATUS 寄存器位定义
 * ================================================================ */
#define RX_DR           0x40
#define TX_DS           0x20
#define MAX_RT          0x10

/* ================================================================
 * 通信参数
 * ================================================================ */
#define TX_ADR_WIDTH    5
#define TX_PLOAD_WIDTH  8

/* ================================================================
 * 硬件引脚宏 (PA4=CSN, PA3=CE)
 * ================================================================ */
#define NF04_CSN_L()    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, GPIO_PIN_RESET)
#define NF04_CSN_H()    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, GPIO_PIN_SET)
#define NF04_CE_L()     HAL_GPIO_WritePin(GPIOA, GPIO_PIN_3, GPIO_PIN_RESET)
#define NF04_CE_H()     HAL_GPIO_WritePin(GPIOA, GPIO_PIN_3, GPIO_PIN_SET)

/* ================================================================
 * 公开接口
 * ================================================================ */
void     NF04_TX_Init(void);
uint8_t  NF04_Read_Reg(uint8_t reg);
void     NF04_Write_Reg(uint8_t reg, uint8_t value);
void     NF04_Transmit_Packet(uint8_t *pBuf);
uint8_t  NF04_Poll_Receive(uint8_t *pBuf);

/* ACK 回传缓冲区（8字节遥测数据） */
extern volatile uint8_t ACK_Rx_Buffer[8];

#endif /* __SPI_TX_H */
