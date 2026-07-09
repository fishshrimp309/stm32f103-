/**
 * @file    SPI_rc.h
 * @brief   NRF24L01+ 接收端（飞机）驱动头文件
 *          基于官方例程 nRF24L01P.c/h 移植，使用 HAL SPI 通信
 *
 * 引脚定义：
 *   SPI1: SCK=PA5, MISO=PA6, MOSI=PA7
 *   CSN=PA4, CE=PA3, IRQ=PA2
 */

#ifndef __SPI_RC_H
#define __SPI_RC_H

#include <stdint.h>
#include "main.h"

/* ================================================================
 * SPI (nRF24L01+) 命令
 * ================================================================ */
#define NRF_READ_REG    0x00  /* 读寄存器命令 */
#define NRF_WRITE_REG   0x20  /* 写寄存器命令 */
#define RD_RX_PLOAD     0x61  /* 读 RX 有效载荷 */
#define WR_TX_PLOAD     0xA0  /* 写 TX 有效载荷 */
#define FLUSH_TX        0xE1  /* 清空 TX FIFO */
#define FLUSH_RX        0xE2  /* 清空 RX FIFO */
#define REUSE_TX_PL     0xE3  /* 重用上一包 TX 载荷 */
#define W_ACK_PAYLOAD   0xA8  /* 预装 ACK 载荷 (pipe 0) */
#define NOP             0xFF  /* 空操作，用于读状态寄存器 */

/* ================================================================
 * NRF24L01+ 寄存器地址
 * ================================================================ */
#define CONFIG          0x00  /* 配置寄存器 */
#define EN_AA           0x01  /* 自动应答使能 */
#define EN_RXADDR       0x02  /* 接收地址使能 */
#define SETUP_AW        0x03  /* 地址宽度设置 */
#define SETUP_RETR      0x04  /* 自动重发设置 */
#define RF_CH           0x05  /* 射频通道 */
#define RF_SETUP        0x06  /* 射频设置 */
#define STATUS          0x07  /* 状态寄存器 */
#define OBSERVE_TX      0x08  /* 发送观察 */
#define RPD             0x09  /* 接收功率检测 */
#define RX_ADDR_P0      0x0A  /* 接收通道 0 地址 */
#define RX_ADDR_P1      0x0B  /* 接收通道 1 地址 */
#define RX_ADDR_P2      0x0C  /* 接收通道 2 地址 */
#define RX_ADDR_P3      0x0D  /* 接收通道 3 地址 */
#define RX_ADDR_P4      0x0E  /* 接收通道 4 地址 */
#define RX_ADDR_P5      0x0F  /* 接收通道 5 地址 */
#define TX_ADDR         0x10  /* 发送地址 */
#define RX_PW_P0        0x11  /* 通道 0 载荷宽度 */
#define RX_PW_P1        0x12  /* 通道 1 载荷宽度 */
#define RX_PW_P2        0x13  /* 通道 2 载荷宽度 */
#define RX_PW_P3        0x14  /* 通道 3 载荷宽度 */
#define RX_PW_P4        0x15  /* 通道 4 载荷宽度 */
#define RX_PW_P5        0x16  /* 通道 5 载荷宽度 */
#define FIFO_STATUS     0x17  /* FIFO 状态寄存器 */
#define DYNPD           0x1C  /* 动态载荷使能 */
#define FEATURE         0x1D  /* 特性寄存器 */

/* ================================================================
 * STATUS 寄存器位定义
 * ================================================================ */
#define RX_DR           0x40  /* 接收数据就绪 */
#define TX_DS           0x20  /* 发送完成 */
#define MAX_RT          0x10  /* 最大重发次数到达 */

/* ================================================================
 * 通信参数
 * ================================================================ */
#define TX_ADR_WIDTH    5     /* 5 字节地址宽度 */
#define TX_PLOAD_WIDTH  8     /* 8 字节有效载荷宽度 */

/* ================================================================
 * 硬件引脚宏 (PA4=CSN, PA3=CE, PA2=IRQ)
 * ================================================================ */
#define NF04_CSN_L()    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, GPIO_PIN_RESET)
#define NF04_CSN_H()    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, GPIO_PIN_SET)
#define NF04_CE_L()     HAL_GPIO_WritePin(GPIOA, GPIO_PIN_3, GPIO_PIN_RESET)
#define NF04_CE_H()     HAL_GPIO_WritePin(GPIOA, GPIO_PIN_3, GPIO_PIN_SET)
#define NF04_IRQ_READ() HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_2)

/* ================================================================
 * 公开接口
 * ================================================================ */
void    NF04_Init(void);
uint8_t NF04_Read_Reg(uint8_t reg);
void    NF04_Write_Reg(uint8_t reg, uint8_t value);
void    NF04_Read_Buf(uint8_t reg, uint8_t *pBuf, uint8_t len);
void    NF04_Clear_Interrupt(void);
void    NF04_Write_Ack_Payload(uint8_t *pBuf);
uint8_t NF04_Send_Packet(uint8_t *pBuf);

#endif /* __SPI_RC_H */
