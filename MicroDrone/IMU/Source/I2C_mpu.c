#include "MPUI2C.h"
#include "i2c.h"

static uint16_t i2c_err_cnt = 0;

/* ================================================================
 *  9 脉冲 I2C 总线恢复（~200us，适合无人机实时需求）
 *  原理：从机可能 SDA 拉死了，主控在 SCL 上发 9 个时钟，
 *        每收到一个时钟从机释放一 bit，最后发 STOP 释放总线。
 *  只动 SCL 脚，不动任何寄存器，速度极快。
 * ================================================================ */
static void i2c_bus_recovery_9pulse(void) {
    GPIO_InitTypeDef GPIO_InitStruct = {0};

    /* PB6=SCL 临时改成推挽输出 */
    GPIO_InitStruct.Pin = GPIO_PIN_6;
    GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_OD;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

    /* 发 9 个时钟脉冲，释放被锁死的 SDA */
    for (int i = 0; i < 9; i++) {
        HAL_GPIO_WritePin(GPIOB, GPIO_PIN_6, GPIO_PIN_RESET);
        for (volatile int d = 0; d < 5; d++);  // ~2us
        HAL_GPIO_WritePin(GPIOB, GPIO_PIN_6, GPIO_PIN_SET);
        for (volatile int d = 0; d < 5; d++);
    }

    /* 发 STOP 条件：SCL 高时拉高 SDA */
    GPIO_InitStruct.Pin = GPIO_PIN_7;
    GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_OD;
    HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_7, GPIO_PIN_SET);
    for (volatile int d = 0; d < 20; d++);

    /* 恢复 I2C 功能 */
    GPIO_InitStruct.Pin = GPIO_PIN_6 | GPIO_PIN_7;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_OD;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
    HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

    /* 清 I2C 状态机 */
    I2C1->CR1 &= ~I2C_CR1_PE;
    for (volatile int i = 0; i < 10; i++);
    I2C1->CR1 |= I2C_CR1_PE;
}

/* 三级递进恢复：9脉冲 → SWRST → DeInit/Init */
static void i2c_recover(void) {
    if (i2c_err_cnt < 3) return;
    i2c_err_cnt = 0;

    /* 第1级：9脉冲恢复（最快，~200us） */
    i2c_bus_recovery_9pulse();

    /* 检查 SDA 是否还被拉低 */
    GPIO_InitTypeDef check = {0};
    check.Pin = GPIO_PIN_7;
    check.Mode = GPIO_MODE_INPUT;
    check.Pull = GPIO_PULLUP;
    HAL_GPIO_Init(GPIOB, &check);
    for (volatile int i = 0; i < 20; i++);
    if (HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_7) == GPIO_PIN_RESET) {
        /* SDA 仍然低 → 硬件问题（线被短路？）无法恢复 */
        HAL_GPIO_Init(GPIOB,
            &(GPIO_InitTypeDef){.Pin = GPIO_PIN_6|GPIO_PIN_7,
                                .Mode = GPIO_MODE_AF_OD,
                                .Speed = GPIO_SPEED_FREQ_HIGH});
        return;
    }
    /* 恢复 AF 模式 */
    GPIO_InitTypeDef af = {0};
    af.Pin = GPIO_PIN_6 | GPIO_PIN_7;
    af.Mode = GPIO_MODE_AF_OD;
    af.Speed = GPIO_SPEED_FREQ_HIGH;
    HAL_GPIO_Init(GPIOB, &af);

    /* 第2级：SWRST 软复位 */
    I2C1->CR1 |= I2C_CR1_SWRST;
    for (volatile int i = 0; i < 20; i++);
    I2C1->CR1 &= ~I2C_CR1_SWRST;

    /* 第3级：完整重新初始化 */
    HAL_I2C_DeInit(&hi2cMPU6050);
    MX_I2C1_Init();
}

HAL_StatusTypeDef i2c_write(uint8_t slave_addr, uint8_t reg_addr,
    uint8_t length, uint8_t const *data) {
  HAL_StatusTypeDef ret;
  ret = HAL_I2C_Mem_Write(&hi2cMPU6050, slave_addr << 1, reg_addr,
  I2C_MEMADD_SIZE_8BIT, (uint8_t*)data, length, 5);   // 5ms 超时，够快
  if (ret != HAL_OK) {
      i2c_err_cnt++;
      i2c_recover();
  } else {
      i2c_err_cnt = 0;
  }
  return ret;
}

HAL_StatusTypeDef i2c_read(uint8_t slave_addr, uint8_t reg_addr, uint8_t length,
    uint8_t *data) {
  HAL_StatusTypeDef ret;
  ret = HAL_I2C_Mem_Read(&hi2cMPU6050, slave_addr << 1, reg_addr,
  I2C_MEMADD_SIZE_8BIT, data, length, 5);   // 5ms 超时
  if (ret != HAL_OK) {
      i2c_err_cnt++;
      i2c_recover();
  } else {
      i2c_err_cnt = 0;
  }
  return ret;
}

HAL_StatusTypeDef IICwriteBit(uint8_t slave_addr, uint8_t reg_addr,
    uint8_t bitNum, uint8_t data) {
  uint8_t tmp;
  i2c_read(slave_addr, reg_addr, 1, &tmp);
  tmp = (data != 0) ? (tmp | (1 << bitNum)) : (tmp & ~(1 << bitNum));
  return i2c_write(slave_addr, reg_addr, 1, &tmp);
};

HAL_StatusTypeDef IICwriteBits(uint8_t slave_addr, uint8_t reg_addr,
    uint8_t bitStart, uint8_t length, uint8_t data) {

  uint8_t tmp, dataShift;
  HAL_StatusTypeDef status = i2c_read(slave_addr, reg_addr, 1, &tmp);
  if (status == HAL_OK) {
    uint8_t mask = (((1 << length) - 1) << (bitStart - length + 1));
    dataShift = data << (bitStart - length + 1);
    tmp &= ~mask;
    tmp |= dataShift;
    return i2c_write(slave_addr, reg_addr, 1, &tmp);
  } else {
    return status;
  }
}
