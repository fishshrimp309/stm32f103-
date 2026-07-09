#include "MPU6050/inv_mpu.h"
#include "MPU6050/inv_mpu_dmp_motion_driver.h"
#include "MPU6050/I2C.h"
#include "MPU6050/mpu6050.h"
#include "string.h" //for reset buffer

#define PRINT_ACCEL     (0x01)
#define PRINT_GYRO      (0x02)
#define PRINT_QUAT      (0x04)
#define ACCEL_ON        (0x01)
#define GYRO_ON         (0x02)
#define MOTION          (0)
#define NO_MOTION       (1)
#define DEFAULT_MPU_HZ  (200)
#define FLASH_SIZE      (512)
#define FLASH_MEM_START ((void*)0x1800)
#define q30  1073741824.0f

short gyro[3], accel[3], sensors;
//float Pitch;
float q0 = 1.0f, q1 = 0.0f, q2 = 0.0f, q3 = 0.0f;
static signed char gyro_orientation[9] = { -1, 0, 0, 0, -1, 0, 0, 0, 1 };

float Pitch, Roll, Yaw;
float Accel_X, Accel_Y, Accel_Z;  // m/s^2
float Gyro_X,  Gyro_Y,  Gyro_Z;   // deg/s
volatile uint8_t MPU6050_Conn_OK_Flag = 0;

// 在文件开头，或者与 Pitch/Roll 同一地方定义全局变量
float Filtered_Yaw = 0;  // 这是我们最终拿去用的、没有零飘的纯净 Yaw
float Last_DMP_Yaw = 0;  // 记录上一次 DMP 算出的原始 Yaw

// 死区阈值（度）。你需要根据循环速度和实际漂移情况微调这个值
// 假设每次循环大概几毫秒，DMP自身漂移每次一般小于 0.05 度
#define YAW_DEADZONE 0.2f







static unsigned short inv_row_2_scale(const signed char *row) {
  unsigned short b;

  if (row[0] > 0)
    b = 0;
  else if (row[0] < 0)
    b = 4;
  else if (row[1] > 0)
    b = 1;
  else if (row[1] < 0)
    b = 5;
  else if (row[2] > 0)
    b = 2;
  else if (row[2] < 0)
    b = 6;
  else
    b = 7;      // error
  return b;
}

static unsigned short inv_orientation_matrix_to_scalar(const signed char *mtx) {
  unsigned short scalar;
  scalar = inv_row_2_scale(mtx);
  scalar |= inv_row_2_scale(mtx + 3) << 3;
  scalar |= inv_row_2_scale(mtx + 6) << 6;

  return scalar;
}

static void run_self_test(void) {
  int result;
  long gyro[3], accel[3];

  result = mpu_run_self_test(gyro, accel);
  if (result == 0x7) {
    /* Test passed. We can trust the gyro data here, so let's push it down
     * to the DMP.
     */
    float sens;
    unsigned short accel_sens;
    mpu_get_gyro_sens(&sens);
    gyro[0] = (long) (gyro[0] * sens);
    gyro[1] = (long) (gyro[1] * sens);
    gyro[2] = (long) (gyro[2] * sens);
    dmp_set_gyro_bias(gyro);
    mpu_get_accel_sens(&accel_sens);
    accel[0] *= accel_sens;
    accel[1] *= accel_sens;
    accel[2] *= accel_sens;
    dmp_set_accel_bias(accel);
    log_i("setting bias succesfully ......\r\n");
  }
}

uint8_t buffer[14];

int16_t MPU6050_FIFO[6][11];
int16_t Gx_offset = 0, Gy_offset = 0, Gz_offset = 0;

/**************************实现函数********************************************
 *函数原型:		void MPU6050_setClockSource(uint8_t source)
 *功　　能:	    设置  MPU6050 的时钟源
 * CLK_SEL | Clock Source
 * --------+--------------------------------------
 * 0       | Internal oscillator
 * 1       | PLL with X Gyro reference
 * 2       | PLL with Y Gyro reference
 * 3       | PLL with Z Gyro reference
 * 4       | PLL with external 32.768kHz reference
 * 5       | PLL with external 19.2MHz reference
 * 6       | Reserved
 * 7       | Stops the clock and keeps the timing generator in reset
 *******************************************************************************/
void MPU6050_setClockSource(uint8_t source) {
  IICwriteBits(devAddr, MPU6050_RA_PWR_MGMT_1, MPU6050_PWR1_CLKSEL_BIT,
  MPU6050_PWR1_CLKSEL_LENGTH, source);

}

/**************************实现函数********************************************
 // *函数原型:		void  MPU6050_newValues(int16_t ax,int16_t ay,int16_t az,int16_t gx,int16_t gy,int16_t gz)
 // *功　　能:	    将新的ADC数据更新到 FIFO数组，进行滤波处理
 // *******************************************************************************/
void MPU6050_newValues(int16_t ax, int16_t ay, int16_t az, int16_t gx,
    int16_t gy, int16_t gz) {
  unsigned char i;
  int32_t sum = 0;
  for (i = 1; i < 10; i++) {	//FIFO 操作
    MPU6050_FIFO[0][i - 1] = MPU6050_FIFO[0][i];
    MPU6050_FIFO[1][i - 1] = MPU6050_FIFO[1][i];
    MPU6050_FIFO[2][i - 1] = MPU6050_FIFO[2][i];
    MPU6050_FIFO[3][i - 1] = MPU6050_FIFO[3][i];
    MPU6050_FIFO[4][i - 1] = MPU6050_FIFO[4][i];
    MPU6050_FIFO[5][i - 1] = MPU6050_FIFO[5][i];
  }
  MPU6050_FIFO[0][9] = ax;	//将新的数据放置到 数据的最后面
  MPU6050_FIFO[1][9] = ay;
  MPU6050_FIFO[2][9] = az;
  MPU6050_FIFO[3][9] = gx;
  MPU6050_FIFO[4][9] = gy;
  MPU6050_FIFO[5][9] = gz;

  sum = 0;
  for (i = 0; i < 10; i++) {	//求当前数组的合，再取平均值
    sum += MPU6050_FIFO[0][i];
  }
  MPU6050_FIFO[0][10] = sum / 10;

  sum = 0;
  for (i = 0; i < 10; i++) {
    sum += MPU6050_FIFO[1][i];
  }
  MPU6050_FIFO[1][10] = sum / 10;

  sum = 0;
  for (i = 0; i < 10; i++) {
    sum += MPU6050_FIFO[2][i];
  }
  MPU6050_FIFO[2][10] = sum / 10;

  sum = 0;
  for (i = 0; i < 10; i++) {
    sum += MPU6050_FIFO[3][i];
  }
  MPU6050_FIFO[3][10] = sum / 10;

  sum = 0;
  for (i = 0; i < 10; i++) {
    sum += MPU6050_FIFO[4][i];
  }
  MPU6050_FIFO[4][10] = sum / 10;

  sum = 0;
  for (i = 0; i < 10; i++) {
    sum += MPU6050_FIFO[5][i];
  }
  MPU6050_FIFO[5][10] = sum / 10;
}

/** Set full-scale gyroscope range.
 * @param range New full-scale gyroscope range value
 * @see getFullScaleRange()
 * @see MPU6050_GYRO_FS_250
 * @see MPU6050_RA_GYRO_CONFIG
 * @see MPU6050_GCONFIG_FS_SEL_BIT
 * @see MPU6050_GCONFIG_FS_SEL_LENGTH
 */
void MPU6050_setFullScaleGyroRange(uint8_t range) {
  IICwriteBits(devAddr, MPU6050_RA_GYRO_CONFIG, MPU6050_GCONFIG_FS_SEL_BIT,
  MPU6050_GCONFIG_FS_SEL_LENGTH, range);
}

/**************************实现函数********************************************
 *函数原型:		void MPU6050_setFullScaleAccelRange(uint8_t range)
 *功　　能:	    设置  MPU6050 加速度计的最大量程
 *******************************************************************************/
void MPU6050_setFullScaleAccelRange(uint8_t range) {
  IICwriteBits(devAddr, MPU6050_RA_ACCEL_CONFIG, MPU6050_ACONFIG_AFS_SEL_BIT,
  MPU6050_ACONFIG_AFS_SEL_LENGTH, range);
}

/**************************实现函数********************************************
 *函数原型:		void MPU6050_setSleepEnabled(uint8_t enabled)
 *功　　能:	    设置  MPU6050 是否进入睡眠模式
 enabled =1   睡觉
 enabled =0   工作
 *******************************************************************************/
void MPU6050_setSleepEnabled(uint8_t enabled) {
  IICwriteBit(devAddr, MPU6050_RA_PWR_MGMT_1, MPU6050_PWR1_SLEEP_BIT, enabled);
}

/**************************实现函数********************************************
 *函数原型:		uint8_t MPU6050_getDeviceID(void)
 *功　　能:	    读取  MPU6050 WHO_AM_I 标识	 将返回 0x68
 *******************************************************************************/
uint8_t MPU6050_getDeviceID(void) {
  memset(buffer,0,sizeof(buffer));
  i2c_read(devAddr, MPU6050_RA_WHO_AM_I, 1, buffer);
  return buffer[0];
}

/**************************实现函数********************************************
 *函数原型:		uint8_t MPU6050_testConnection(void)
 *功　　能:	    检测MPU6050 是否已经连接
 *******************************************************************************/
uint8_t MPU6050_testConnection(void) {
  if (MPU6050_getDeviceID() == 0x68)  //0b01101000;
    MPU6050_Conn_OK_Flag = 1;
  else
    MPU6050_Conn_OK_Flag = 0;

  return MPU6050_Conn_OK_Flag;
}

/**************************实现函数********************************************
 *函数原型:		void MPU6050_setI2CMasterModeEnabled(uint8_t enabled)
 *功　　能:	    设置 MPU6050 是否为AUX I2C线的主机
 *******************************************************************************/
void MPU6050_setI2CMasterModeEnabled(uint8_t enabled) {
  IICwriteBit(devAddr, MPU6050_RA_USER_CTRL, MPU6050_USERCTRL_I2C_MST_EN_BIT,
      enabled);
}

/**************************实现函数********************************************
 *函数原型:		void MPU6050_setI2CBypassEnabled(uint8_t enabled)
 *功　　能:	    设置 MPU6050 是否为AUX I2C线的主机
 *******************************************************************************/
void MPU6050_setI2CBypassEnabled(uint8_t enabled) {
  IICwriteBit(devAddr, MPU6050_RA_INT_PIN_CFG, MPU6050_INTCFG_I2C_BYPASS_EN_BIT,
      enabled);
}

/**************************实现函数********************************************
 *函数原型:		void MPU6050_initialize(void)
 *功　　能:	    初始化 	MPU6050 以进入可用状态。
 *******************************************************************************/
void MPU6050_initialize(void) {
  MPU6050_setClockSource(MPU6050_CLOCK_PLL_XGYRO); //设置时钟
  MPU6050_setFullScaleGyroRange(MPU6050_GYRO_FS_2000); //陀螺仪最大量程 +-1000度每秒
  MPU6050_setFullScaleAccelRange(MPU6050_ACCEL_FS_2);	//加速度度最大量程 +-2G
  MPU6050_setSleepEnabled(0); //进入工作状态
  MPU6050_setI2CMasterModeEnabled(0);	 //不让MPU6050 控制AUXI2C
  MPU6050_setI2CBypassEnabled(0);	//主控制器的I2C与	MPU6050的AUXI2C	直通。控制器可以直接访问HMC5883L
  MPU6050_testConnection();
}

/**************************************************************************
 函数功能：MPU6050内置DMP的初始化
 入口参数：无
 返回  值：无
 作    者：平衡小车之家
 **********************l
****************************************************/
void DMP_Init(void) {
  if (MPU6050_getDeviceID() != 0x68)
    NVIC_SystemReset();
  if (!mpu_init(NULL)) {
    if (!mpu_set_sensors(INV_XYZ_GYRO | INV_XYZ_ACCEL))
      log_i("mpu_set_sensor complete ......\r\n");
    if (!mpu_configure_fifo(INV_XYZ_GYRO | INV_XYZ_ACCEL))
      log_i("mpu_configure_fifo complete ......\r\n");
    if (!mpu_set_sample_rate(DEFAULT_MPU_HZ))
      log_i("mpu_set_sample_rate complete ......\r\n");
    if (!dmp_load_motion_driver_firmware())
      log_i("dmp_load_motion_driver_firmware complete ......\r\n");
    if (!dmp_set_orientation(
        inv_orientation_matrix_to_scalar(gyro_orientation)))
      log_i("dmp_set_orientation complete ......\r\n");
    if (!dmp_enable_feature(
        DMP_FEATURE_6X_LP_QUAT | DMP_FEATURE_TAP |
        DMP_FEATURE_ANDROID_ORIENT | DMP_FEATURE_SEND_RAW_ACCEL
            | DMP_FEATURE_SEND_CAL_GYRO |
            DMP_FEATURE_GYRO_CAL))
      log_i("dmp_enable_feature complete ......\r\n");
    if (!dmp_set_fifo_rate(DEFAULT_MPU_HZ))
      log_i("dmp_set_fifo_rate complete ......\r\n");
    run_self_test();
    if (!mpu_set_dmp_state(1))
      log_i("mpu_set_dmp_state complete ......\r\n");
  }
}
/**************************************************************************
 函数功能：读取MPU6050内置DMP的姿态信息
 入口参数：无
 返回  值：无
 作    者：平衡小车之家
 **************************************************************************/
void Read_DMP(void) {
  unsigned long sensor_timestamp;
  unsigned char more;
  long quat[4];

  dmp_read_fifo(gyro, accel, quat, &sensor_timestamp, &sensors, &more);
  if (sensors & INV_WXYZ_QUAT) {
    q0 = quat[0] / q30;
    q1 = quat[1] / q30;
    q2 = quat[2] / q30;
    q3 = quat[3] / q30;
    Pitch = sinf(-2 * q1 * q3 + 2 * q0 * q2) * 57.3;
  }

}
/**************************************************************************
 函数功能：读取MPU6050内置温度传感器数据
 入口参数：无
 返回  值：摄氏温度
 作    者：平衡小车之家
 **************************************************************************/
int Read_Temperature(void) {
  float Temp;
  uint8_t H, L;
  i2c_read(devAddr, MPU6050_RA_TEMP_OUT_H, 1, &H);
  i2c_read(devAddr, MPU6050_RA_TEMP_OUT_L, 1, &L);
  Temp = (H << 8) + L;
  if (Temp > 32768)
    Temp -= 65536;
  Temp = (36.53 + Temp / 340) * 10;
  return (int) Temp;
}








/* ================================================================
 *  新增：PB5 中断检查与读取函数
 *  放在 mpu6050.c 的末尾
 * ================================================================ */

//// 声明在 main.c 中定义的全局标志位（需要与 main.c 中的变量名一致）
//extern volatile uint8_t mpu_interrupt_flag;

// 非阻塞读取函数
uint8_t MPU_Check_And_Read(void) {
    // 1. 检查标志位
//    if (mpu_interrupt_flag) {
//        mpu_interrupt_flag = 0; // 清除标志

        // 2. 声明局部变量 (解决红叉的关键！)
        long quat[4];
        unsigned long timestamp;
        short gyro[3], accel[3], sensors;
        unsigned char more;
        float q0, q1, q2, q3; // 四元数浮点值

        // 3. 直接调用底层 DMP 读取函数
        // 注意：这里不再调用 Read_DMP()，而是把它的逻辑搬过来了
        dmp_read_fifo(gyro, accel, quat, &timestamp, &sensors, &more);

        // 4. 如果成功读到了四元数
        if (sensors & INV_WXYZ_QUAT) {
            // 格式转换
            q0 = quat[0] / q30;
            q1 = quat[1] / q30;
            q2 = quat[2] / q30;
            q3 = quat[3] / q30;

            // 5. 计算欧拉角 (更新全局变量)
            Pitch = asin(-2 * q1 * q3 + 2 * q0 * q2) * 57.3f;
            Roll  = atan2(2 * q2 * q3 + 2 * q0 * q1, -2 * q1 * q1 - 2 * q2 * q2 + 1) * 57.3f;
            float raw_yaw = atan2(2 * (q1 * q2 + q0 * q3), q0 * q0 + q1 * q1 - q2 * q2 - q3 * q3) * 57.3f;

            // --- DMP 死区滤波抗零飘 ---
            float delta_yaw = raw_yaw - Last_DMP_Yaw;
            if (delta_yaw > 180.0f)  delta_yaw -= 360.0f;
            if (delta_yaw < -180.0f) delta_yaw += 360.0f;
            if (delta_yaw > YAW_DEADZONE || delta_yaw < -YAW_DEADZONE) {
                Filtered_Yaw += delta_yaw;
                if (Filtered_Yaw > 180.0f)  Filtered_Yaw -= 360.0f;
                if (Filtered_Yaw < -180.0f) Filtered_Yaw += 360.0f;
            }
            Last_DMP_Yaw = raw_yaw;
            Yaw = Filtered_Yaw;

            // 6. 加速度：硬件单位 → m/s^2（±2g, 16384 LSB/g, 1g=9.80665 m/s^2）
            Accel_X = accel[0] / 16384.0f * 9.80665f;
            Accel_Y = accel[1] / 16384.0f * 9.80665f;
            Accel_Z = accel[2] / 16384.0f * 9.80665f;

            // 7. 角速度：硬件单位 → °/s（±2000dps, 16.4 LSB/(°/s)）
            Gyro_X = gyro[0] / 16.4f;
            Gyro_Y = gyro[1] / 16.4f;
            Gyro_Z = gyro[2] / 16.4f;

            return 1;   // 返回 1 表示数据更新成功
        }
//    }
    return 0; // 没数据或读取失败
}
//------------------End of File----------------------------
