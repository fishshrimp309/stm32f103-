/**
 * @file    SPI_tx.c
 * @brief   NRF24L01+ 发送端（遥控器）驱动
 *          基于官方例程 nRF24L01P.c 移植，使用 HAL SPI 通信
 *
 * 功能说明：
 *   1. 常态处于 RX 模式监听，等待接收飞机的 ACK 载荷回传
 *   2. 发送时切换到 TX 模式，CE脉冲触发发射，等待 IRQ 结果
 *   3. 收到 ACK 载荷时解析遥测数据（Roll, Pitch, Yaw, Throttle）
 *
 * 引脚：SPI1(PA5/6/7), CSN=PA4, CE=PA3
 */

#include "SPI_tx.h"
#include "spi.h"
#include "usart.h"

/* 收发地址（与接收端保持一致） */
static const uint8_t TX_ADDRESS[TX_ADR_WIDTH] = {0xAA, 0xBB, 0xCC, 0xDD, 0xEE};

/* ACK 回传数据缓冲区 */
volatile uint8_t ACK_Rx_Buffer[8] = {0};

/* ================================================================
 * SPI 单字节收发（对应例程 SPI_RW）
 * ================================================================ */
static uint8_t SPI_RW(uint8_t byte)
{
    uint8_t rx_data = 0;
    HAL_SPI_TransmitReceive(&hspi1, &byte, &rx_data, 1, 10);
    return rx_data;
}

/* ================================================================
 * 写寄存器（单字节值）
 * 对应例程 nRF24L01P_Write_Reg
 * ================================================================ */
void NF04_Write_Reg(uint8_t reg, uint8_t value)
{
    NF04_CSN_L();
    SPI_RW(NRF_WRITE_REG | reg);
    SPI_RW(value);
    NF04_CSN_H();
}

/* ================================================================
 * 写寄存器（多字节）
 * 对应例程 nRF24L01P_Write_Buf
 * ================================================================ */
static uint8_t NF04_Write_Buf(uint8_t reg, const uint8_t *pBuf, uint8_t bytes)
{
    uint8_t status;
    NF04_CSN_L();
    status = SPI_RW(NRF_WRITE_REG | reg);
    for (uint8_t i = 0; i < bytes; i++)
        SPI_RW(*pBuf++);
    NF04_CSN_H();
    return status;
}

/* ================================================================
 * 读寄存器（单字节值）
 * 对应例程 nRF24L01P_Read_Reg
 * ================================================================ */
uint8_t NF04_Read_Reg(uint8_t reg)
{
    uint8_t value;
    NF04_CSN_L();
    SPI_RW(NRF_READ_REG | reg);
    value = SPI_RW(NOP);
    NF04_CSN_H();
    return value;
}

/* ================================================================
 * 设置为发送模式（对应例程 nRF24L01P_TX_Mode）
 *
 * 配置：CRC16, 上电, TX 模式
 *       自动重发: 250us+86us间隔, 最多10次
 * ================================================================ */
static void NF04_TX_Mode(void)
{
    NF04_CE_L();

    /* 写发送地址 */
    NF04_Write_Buf(TX_ADDR, TX_ADDRESS, TX_ADR_WIDTH);

    /* 写接收通道0地址（为接收ACK，与发送地址相同） */
    NF04_Write_Buf(RX_ADDR_P0, TX_ADDRESS, TX_ADR_WIDTH);

    /* 使能通道0自动应答 */
    NF04_Write_Reg(EN_AA, 0x01);

    /* 使能接收通道0 */
    NF04_Write_Reg(EN_RXADDR, 0x01);

    /* 设置5字节地址宽度 */
    NF04_Write_Reg(SETUP_AW, 0x03);

    /* 自动重发：250us+86us间隔, 最多10次 */
    NF04_Write_Reg(SETUP_RETR, 0x0a);

    /* RF通道40 (2.440GHz) */
    NF04_Write_Reg(RF_CH, 40);

    /* 射频设置：1Mbps, 0dBm, LNA增益 */
    NF04_Write_Reg(RF_SETUP, 0x07);

    /* CONFIG: CRC使能(16位), 上电, TX模式 */
    NF04_Write_Reg(CONFIG, 0x0e);
}

/* ================================================================
 * 设置为接收模式（对应例程 nRF24L01P_RX_Mode）
 * ================================================================ */
static void NF04_RX_Mode(void)
{
    NF04_CE_L();

    /* 写接收通道0地址 */
    NF04_Write_Buf(RX_ADDR_P0, TX_ADDRESS, TX_ADR_WIDTH);

    /* 使能通道0自动应答 */
    NF04_Write_Reg(EN_AA, 0x01);

    /* 使能接收通道0 */
    NF04_Write_Reg(EN_RXADDR, 0x01);

    /* 设置5字节地址宽度 */
    NF04_Write_Reg(SETUP_AW, 0x03);

    /* 设置通道0载荷宽度 */
    NF04_Write_Reg(RX_PW_P0, TX_PLOAD_WIDTH);

    /* 使能动态载荷 + ACK载荷功能 */
    NF04_Write_Reg(DYNPD, 0x01);
    NF04_Write_Reg(FEATURE, 0x06);

    /* RF通道40 (2.440GHz) */
    NF04_Write_Reg(RF_CH, 40);

    /* 射频设置：1Mbps, 0dBm */
    NF04_Write_Reg(RF_SETUP, 0x07);

    /* CONFIG: CRC使能(16位), 上电, RX模式 */
    NF04_Write_Reg(CONFIG, 0x0f);

    /* 清除中断标志 */
    NF04_Write_Reg(STATUS, 0xff);

    NF04_CE_H();  /* 启动接收 */
}

/* ================================================================
 * 初始化（对应例程 nRF24L01P_Init + 首次配对前配置）
 *
 * 上电默认进入 RX 模式监听
 * ================================================================ */
void NF04_TX_Init(void)
{
    NF04_CSN_H();
    NF04_CE_L();

    /* Si24R1 激活序列（正版 NRF24L01+ 会忽略，不影响） */
    NF04_CSN_L(); SPI_RW(0x50); SPI_RW(0x73); NF04_CSN_H();

    /* 进入 RX 模式完成初始配置，发送前再切 TX */
    NF04_RX_Mode();

    /* 清空 FIFO */
    NF04_CSN_L(); SPI_RW(FLUSH_RX); NF04_CSN_H();
    NF04_CSN_L(); SPI_RW(FLUSH_TX); NF04_CSN_H();
}

/* ================================================================
 * 发送一包数据并等待结果（对应例程 nRF24L01P_TxPacket）
 *
 * 流程：切TX模式 → 写载荷 → CE脉冲 → 轮询STATUS →
 *       读ACK载荷 → 清除标志 → 切回RX模式
 *
 * @param  pBuf: 要发送的 8 字节数据
 * ================================================================ */
void NF04_Transmit_Packet(uint8_t *pBuf)
{
    uint8_t status;

    NF04_CE_L();

    /* 切换到 TX 模式 */
    NF04_TX_Mode();

    /* 清空 TX FIFO */
    NF04_CSN_L(); SPI_RW(FLUSH_TX); NF04_CSN_H();

    /* 写载荷到 TX FIFO（对应例程 WR_TX_PLOAD） */
    NF04_CSN_L();
    SPI_RW(WR_TX_PLOAD);
    for (uint8_t i = 0; i < TX_PLOAD_WIDTH; i++)
        SPI_RW(pBuf[i]);
    NF04_CSN_H();

    /* CE 脉冲触发发射（对应例程 CE=1） */
    NF04_CE_H();
    for (volatile int i = 0; i < 300; i++);
    NF04_CE_L();

    /* 轮询 STATUS 寄存器等待完成（对应例程 while(IRQ==1)） */
    status = 0;
    {
        uint32_t tick = HAL_GetTick();
        while ((HAL_GetTick() - tick) < 100)  /* 100ms 超时 */
        {
            status = NF04_Read_Reg(STATUS);
            if ((status & TX_DS) || (status & MAX_RT))
                break;
            HAL_Delay(1);
        }
    }

    /* 处理发送结果 */
    if (status & MAX_RT)
    {
        /* 重发耗尽：清空 TX FIFO */
        NF04_CSN_L(); SPI_RW(FLUSH_TX); NF04_CSN_H();
    }
    else if ((status & TX_DS) && (status & RX_DR))
    {
        /* TX_DS + RX_DR：收到带 ACK 载荷的回复（遥测数据） */
        uint8_t rx_buf[TX_PLOAD_WIDTH];
        NF04_CSN_L();
        SPI_RW(RD_RX_PLOAD);
        for (uint8_t i = 0; i < TX_PLOAD_WIDTH; i++)
            rx_buf[i] = SPI_RW(NOP);
        NF04_CSN_H();

        /* 保存到全局缓冲区 */
        for (uint8_t i = 0; i < TX_PLOAD_WIDTH; i++)
            ACK_Rx_Buffer[i] = rx_buf[i];

        /* 通过串口发送给上位机 */
        HAL_UART_Transmit(&huart1, rx_buf, TX_PLOAD_WIDTH, 10);

        /* 清空 RX FIFO */
        NF04_CSN_L(); SPI_RW(FLUSH_RX); NF04_CSN_H();
    }
    else if (status & TX_DS)
    {
        /* 仅 TX_DS：空 ACK（无载荷） */
        NF04_CSN_L(); SPI_RW(FLUSH_RX); NF04_CSN_H();
    }

    /* 清除 STATUS 标志 */
    NF04_Write_Reg(STATUS, status);

    /* 切回 RX 模式继续监听 */
    NF04_RX_Mode();
    NF04_CSN_L(); SPI_RW(FLUSH_TX); NF04_CSN_H();
    NF04_CSN_L(); SPI_RW(FLUSH_RX); NF04_CSN_H();
    NF04_CE_H();
}

/* ================================================================
 * 读寄存器（多字节）- 内部使用
 * ================================================================ */
static void NF04_Read_Buf(uint8_t reg, uint8_t *pBuf, uint8_t bytes)
{
    NF04_CSN_L();
    SPI_RW(NRF_READ_REG | reg);
    for (uint8_t i = 0; i < bytes; i++)
        pBuf[i] = SPI_RW(NOP);
    NF04_CSN_H();
}

/* ================================================================
 * 轮询接收数据包（对应例程 nRF24L01P_RxPacket）
 *
 * 仅在 RX 模式下有效。收到数据后自动清除 RX_DR 标志。
 *
 * @param  pBuf: 存放 8 字节载荷的缓冲区
 * @return 0=收到数据, 1=无数据
 * ================================================================ */
uint8_t NF04_Poll_Receive(uint8_t *pBuf)
{
    uint8_t state = NF04_Read_Reg(STATUS);

    /* 清除 RX_DR 中断标志 */
    NF04_Write_Reg(STATUS, state);

    if (state & RX_DR)
    {
        /* 收到数据：读载荷 */
        NF04_Read_Buf(RD_RX_PLOAD, pBuf, TX_PLOAD_WIDTH);

        /* 清空 RX FIFO（对应例程 FLUSH_RX） */
        NF04_CSN_L(); SPI_RW(FLUSH_RX); NF04_CSN_H();
        return 0;
    }
    return 1;  /* 无数据 */
}
