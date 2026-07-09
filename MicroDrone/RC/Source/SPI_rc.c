/**
 * @file    SPI_rc.c
 * @brief   NRF24L01+ 接收端（飞机）驱动
 *          基于官方例程 nRF24L01P.c 移植，使用 HAL SPI 通信
 *
 * 功能说明：
 *   1. 常态处于 RX 模式，通过 IRQ 中断接收遥控指令
 *   2. 预装 ACK 载荷，收到有效包时硬件自动回复遥测数据
 *   3. 支持主动切换到 TX 模式发送数据包，发送完毕后切回 RX
 *
 * 引脚：SPI1(PA5/6/7), CSN=PA4, CE=PA3, IRQ=PA2
 */

#include "SPI_rc.h"
#include "spi.h"

/* 收发地址（与发送端保持一致） */
static const uint8_t TX_ADDRESS[TX_ADR_WIDTH] = {0xAA, 0xBB, 0xCC, 0xDD, 0xEE};

/* ================================================================
 * SPI 单字节收发（对应例程 SPI_RW）
 * 使用 HAL_SPI_TransmitReceive，与例程位操作逻辑等效
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
 * 读寄存器（多字节）
 * 对应例程 nRF24L01P_Read_Buf
 * ================================================================ */
void NF04_Read_Buf(uint8_t reg, uint8_t *pBuf, uint8_t bytes)
{
    NF04_CSN_L();
    SPI_RW(NRF_READ_REG | reg);
    for (uint8_t i = 0; i < bytes; i++)
        pBuf[i] = SPI_RW(NOP);
    NF04_CSN_H();
}

/* ================================================================
 * 清除所有中断标志
 * ================================================================ */
void NF04_Clear_Interrupt(void)
{
    uint8_t status = NF04_Read_Reg(STATUS);
    NF04_Write_Reg(STATUS, status);
}

/* ================================================================
 * 设置为接收模式（对应例程 nRF24L01P_RX_Mode）
 *
 * 配置：CRC16, 上电, RX 模式
 *       RF通道40, 2Mbps, 0dBm
 *       通道0自动应答, 5字节地址, 8字节载荷
 * ================================================================ */
static void NF04_RX_Mode(void)
{
    NF04_CE_L();

    /* 写接收通道0地址（与发送地址相同） */
    NF04_Write_Buf(RX_ADDR_P0, TX_ADDRESS, TX_ADR_WIDTH);

    /* 使能通道0自动应答 */
    NF04_Write_Reg(EN_AA, 0x01);

    /* 使能接收通道0 */
    NF04_Write_Reg(EN_RXADDR, 0x01);

    /* 设置5字节地址宽度 */
    NF04_Write_Reg(SETUP_AW, 0x03);

    /* 选择RF通道40 (2.440GHz) */
    NF04_Write_Reg(RF_CH, 40);

    /* 设置通道0载荷宽度 */
    NF04_Write_Reg(RX_PW_P0, TX_PLOAD_WIDTH);

    /* 使能动态载荷 + ACK载荷功能 */
    NF04_Write_Reg(DYNPD, 0x01);
    NF04_Write_Reg(FEATURE, 0x06);

    /* 射频设置：1Mbps, 0dBm, LNA增益 */
    NF04_Write_Reg(RF_SETUP, 0x07);

    /* CONFIG: CRC使能(16位), 上电, RX模式 */
    NF04_Write_Reg(CONFIG, 0x0f);

    /* 清除所有中断标志 */
    NF04_Write_Reg(STATUS, 0xff);

    NF04_CE_H();  /* 启动接收 */
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

    /* 写接收通道0地址（为接收ACK，地址与发送地址相同） */
    NF04_Write_Buf(RX_ADDR_P0, TX_ADDRESS, TX_ADR_WIDTH);

    /* 使能通道0自动应答 */
    NF04_Write_Reg(EN_AA, 0x01);

    /* 使能接收通道0 */
    NF04_Write_Reg(EN_RXADDR, 0x01);

    /* 设置5字节地址宽度 */
    NF04_Write_Reg(SETUP_AW, 0x03);

    /* 自动重发设置：250us+86us等待, 最多10次重发 */
    NF04_Write_Reg(SETUP_RETR, 0x0a);

    /* RF通道40 (2.440GHz) */
    NF04_Write_Reg(RF_CH, 40);

    /* 射频设置：2Mbps, 0dBm, LNA增益 */
    NF04_Write_Reg(RF_SETUP, 0x07);

    /* CONFIG: CRC使能(16位), 上电, TX模式 */
    NF04_Write_Reg(CONFIG, 0x0e);

    /* 清除所有中断标志 */
    NF04_Write_Reg(STATUS, 0xff);
}

/* ================================================================
 * 初始化模块（对应例程 nRF24L01P_Init + nRF24L01P_RX_Mode）
 *
 * 上电默认进入 RX 模式，同时预装 ACK 载荷用于配对
 * ================================================================ */
void NF04_Init(void)
{
    /* 引脚初始状态（对应例程 nRF24L01P_Init） */
    NF04_CSN_H();   /* CSN 高：未选中 */
    NF04_CE_L();    /* CE 低：待机模式 */

    /* Si24R1 激活序列（正版 NRF24L01+ 会忽略，不影响） */
    NF04_CSN_L(); SPI_RW(0x50); SPI_RW(0x73); NF04_CSN_H();

    /* 进入 RX 模式（配置所有寄存器并启动监听） */
    NF04_RX_Mode();

    /* 清空 FIFO */
    NF04_CSN_L(); SPI_RW(FLUSH_RX); NF04_CSN_H();
    NF04_CSN_L(); SPI_RW(FLUSH_TX); NF04_CSN_H();

    /* 预装第一个 ACK 载荷，配对时自动回复 */
    uint8_t init_ack[TX_PLOAD_WIDTH] = {0xBB, 0x22, 0x33, 0x44, 0x55, 0x66, 0x77, 0x88};
    NF04_Write_Ack_Payload(init_ack);
}

/* ================================================================
 * 预装 ACK 载荷（pipe 0）
 *
 * 下次收到有效数据包时，硬件会在 ACK 中自动携带此载荷。
 * 发送端收到 ACK 载荷后，硬件自动将 RX_DR 和 TX_DS 同时置位。
 *
 * 对应例程思路：例程没有使用 ACK 载荷，这里扩展使用
 * ================================================================ */
void NF04_Write_Ack_Payload(uint8_t *pBuf)
{
    NF04_CSN_L();
    SPI_RW(W_ACK_PAYLOAD);  /* 0xA8 = W_ACK_PAYLOAD | pipe 0 */
    for (uint8_t i = 0; i < TX_PLOAD_WIDTH; i++)
        SPI_RW(pBuf[i]);
    NF04_CSN_H();
}

/* ================================================================
 * 主动发送一包数据（对应例程 nRF24L01P_TxPacket）
 *
 * 流程：切 TX 模式 → 装载荷 → CE脉冲发射 → 等IRQ/超时 →
 *       读状态 → 清除标志 → 切回 RX 模式
 *
 * @param  pBuf: 要发送的 8 字节数据
 * @return 0x20=TX_DS发送成功, 0x10=MAX_RT重发耗尽, 0xFF=其他失败
 * ================================================================ */
uint8_t NF04_Send_Packet(uint8_t *pBuf)
{
    uint8_t state;

    /* 关接收中断，防止中断中同时操作 SPI */
    HAL_NVIC_DisableIRQ(EXTI2_IRQn);

    /* 切换到 TX 模式 */
    NF04_TX_Mode();

    /* 写载荷到 TX FIFO（对应例程 WR_TX_PLOAD） */
    NF04_CSN_L();
    SPI_RW(WR_TX_PLOAD);
    for (uint8_t i = 0; i < TX_PLOAD_WIDTH; i++)
        SPI_RW(pBuf[i]);
    NF04_CSN_H();

    /* CE 脉冲触发发射（对应例程 CE=1） */
    NF04_CE_H();
    for (volatile int i = 0; i < 300; i++);  /* 短延时确保发射启动 */
    NF04_CE_L();

    /* 等待 IRQ 变低，带超时保护（避免阻塞死循环） */
    {
        uint32_t tick = HAL_GetTick();
        while (NF04_IRQ_READ() == 1)
        {
            if ((HAL_GetTick() - tick) > 100) break;  /* 100ms 超时 */
        }
    }

    /* 读状态寄存器，清除中断标志（对应例程逻辑） */
    state = NF04_Read_Reg(STATUS);
    NF04_Write_Reg(STATUS, state);

    /* 判断发送结果 */
    if (state & MAX_RT)
    {
        /* 达到最大重发次数：清空 TX FIFO */
        NF04_CSN_L(); SPI_RW(FLUSH_TX); NF04_CSN_H();
    }

    /* 切回 RX 模式继续监听 */
    NF04_RX_Mode();

    /* 清空 RX FIFO（避免残留数据触发误判） */
    NF04_CSN_L(); SPI_RW(FLUSH_RX); NF04_CSN_H();
    NF04_CSN_L(); SPI_RW(FLUSH_TX); NF04_CSN_H();

    HAL_NVIC_EnableIRQ(EXTI2_IRQn);

    if (state & TX_DS) return TX_DS;   /* 发送成功 */
    if (state & MAX_RT) return MAX_RT; /* 重发耗尽 */
    return 0xFF;                        /* 其他失败 */
}
