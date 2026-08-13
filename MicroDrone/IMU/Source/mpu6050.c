#include "inv_mpu.h"
#include "inv_mpu_dmp_motion_driver.h"
#include "MPUI2C.h"
#include "mpu6050.h"
#include "string.h"
#include "cmsis_os.h"
#include <math.h>

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
float q0 = 1.0f, q1 = 0.0f, q2 = 0.0f, q3 = 0.0f;
static signed char gyro_orientation[9] = { -1, 0, 0, 0, -1, 0, 0, 0, 1 };

float Pitch_Offset = 2.6, Roll_Offset = 0, Yaw_Offset = 0;
float Pitch, Roll, Yaw;
float Accel_X, Accel_Y, Accel_Z;
float Gyro_X,  Gyro_Y,  Gyro_Z;
volatile uint8_t MPU6050_Conn_OK_Flag = 0;

float Filtered_Yaw = 0;
float Last_DMP_Yaw = 0;

#define YAW_DEADZONE 0.1f

volatile uint8_t i2c_busy = 0;


static unsigned short inv_row_2_scale(const signed char *row) {
  unsigned short b;
  if (row[0] > 0)      b = 0;
  else if (row[0] < 0) b = 4;
  else if (row[1] > 0) b = 1;
  else if (row[1] < 0) b = 5;
  else if (row[2] > 0) b = 2;
  else if (row[2] < 0) b = 6;
  else                 b = 7;
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
    float sens;
    unsigned short accel_sens;
    mpu_get_gyro_sens(&sens);
    gyro[0] = (long)(gyro[0] * sens);
    gyro[1] = (long)(gyro[1] * sens);
    gyro[2] = (long)(gyro[2] * sens);
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

void MPU6050_setClockSource(uint8_t source) {
  IICwriteBits(devAddr, MPU6050_RA_PWR_MGMT_1,
               MPU6050_PWR1_CLKSEL_BIT, MPU6050_PWR1_CLKSEL_LENGTH, source);
}

void MPU6050_newValues(int16_t ax, int16_t ay, int16_t az,
                        int16_t gx, int16_t gy, int16_t gz) {
  unsigned char i;
  int32_t sum = 0;
  for (i = 1; i < 10; i++) {
    MPU6050_FIFO[0][i-1] = MPU6050_FIFO[0][i];
    MPU6050_FIFO[1][i-1] = MPU6050_FIFO[1][i];
    MPU6050_FIFO[2][i-1] = MPU6050_FIFO[2][i];
    MPU6050_FIFO[3][i-1] = MPU6050_FIFO[3][i];
    MPU6050_FIFO[4][i-1] = MPU6050_FIFO[4][i];
    MPU6050_FIFO[5][i-1] = MPU6050_FIFO[5][i];
  }
  MPU6050_FIFO[0][9] = ax;
  MPU6050_FIFO[1][9] = ay;
  MPU6050_FIFO[2][9] = az;
  MPU6050_FIFO[3][9] = gx;
  MPU6050_FIFO[4][9] = gy;
  MPU6050_FIFO[5][9] = gz;

  for (i = 0; i < 10; i++) { sum += MPU6050_FIFO[0][i]; }
  MPU6050_FIFO[0][10] = sum / 10;
  sum = 0;
  for (i = 0; i < 10; i++) { sum += MPU6050_FIFO[1][i]; }
  MPU6050_FIFO[1][10] = sum / 10;
  sum = 0;
  for (i = 0; i < 10; i++) { sum += MPU6050_FIFO[2][i]; }
  MPU6050_FIFO[2][10] = sum / 10;
  sum = 0;
  for (i = 0; i < 10; i++) { sum += MPU6050_FIFO[3][i]; }
  MPU6050_FIFO[3][10] = sum / 10;
  sum = 0;
  for (i = 0; i < 10; i++) { sum += MPU6050_FIFO[4][i]; }
  MPU6050_FIFO[4][10] = sum / 10;
  sum = 0;
  for (i = 0; i < 10; i++) { sum += MPU6050_FIFO[5][i]; }
  MPU6050_FIFO[5][10] = sum / 10;
}

void MPU6050_setFullScaleGyroRange(uint8_t range) {
  IICwriteBits(devAddr, MPU6050_RA_GYRO_CONFIG,
               MPU6050_GCONFIG_FS_SEL_BIT, MPU6050_GCONFIG_FS_SEL_LENGTH, range);
}

void MPU6050_setFullScaleAccelRange(uint8_t range) {
  IICwriteBits(devAddr, MPU6050_RA_ACCEL_CONFIG,
               MPU6050_ACONFIG_AFS_SEL_BIT, MPU6050_ACONFIG_AFS_SEL_LENGTH, range);
}

void MPU6050_setSleepEnabled(uint8_t enabled) {
  IICwriteBit(devAddr, MPU6050_RA_PWR_MGMT_1, MPU6050_PWR1_SLEEP_BIT, enabled);
}

uint8_t MPU6050_getDeviceID(void) {
  memset(buffer, 0, sizeof(buffer));
  i2c_read(devAddr, MPU6050_RA_WHO_AM_I, 1, buffer);
  return buffer[0];
}

uint8_t MPU6050_testConnection(void) {
  if (MPU6050_getDeviceID() == 0x68)
    MPU6050_Conn_OK_Flag = 1;
  else
    MPU6050_Conn_OK_Flag = 0;
  return MPU6050_Conn_OK_Flag;
}

void MPU6050_setI2CMasterModeEnabled(uint8_t enabled) {
  IICwriteBit(devAddr, MPU6050_RA_USER_CTRL,
              MPU6050_USERCTRL_I2C_MST_EN_BIT, enabled);
}

void MPU6050_setI2CBypassEnabled(uint8_t enabled) {
  IICwriteBit(devAddr, MPU6050_RA_INT_PIN_CFG,
              MPU6050_INTCFG_I2C_BYPASS_EN_BIT, enabled);
}

void MPU6050_initialize(void) {
  MPU6050_setClockSource(MPU6050_CLOCK_PLL_XGYRO);
  MPU6050_setFullScaleGyroRange(MPU6050_GYRO_FS_2000);
  MPU6050_setFullScaleAccelRange(MPU6050_ACCEL_FS_2);
  MPU6050_setSleepEnabled(0);
  MPU6050_setI2CMasterModeEnabled(0);
  MPU6050_setI2CBypassEnabled(0);
  MPU6050_testConnection();
}

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
            DMP_FEATURE_ANDROID_ORIENT | DMP_FEATURE_SEND_RAW_ACCEL |
            DMP_FEATURE_SEND_CAL_GYRO | DMP_FEATURE_GYRO_CAL))
      log_i("dmp_enable_feature complete ......\r\n");
    if (!dmp_set_fifo_rate(DEFAULT_MPU_HZ))
      log_i("dmp_set_fifo_rate complete ......\r\n");
    run_self_test();
    if (!mpu_set_dmp_state(1))
      log_i("dmp_set_dmp_state complete ......\r\n");
  }
}

void MPU6050_DMP_Calibrate(void)
{
    for (int i = 0; i < 600; i++)
    {
        MPU_Check_And_Read();
        HAL_Delay(5);
    }
}

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

int Read_Temperature(void) {
  float Temp;
  uint8_t H, L;
  i2c_read(devAddr, MPU6050_RA_TEMP_OUT_H, 1, &H);
  i2c_read(devAddr, MPU6050_RA_TEMP_OUT_L, 1, &L);
  Temp = (H << 8) + L;
  if (Temp > 32768) Temp -= 65536;
  Temp = (36.53 + Temp / 340) * 10;
  return (int)Temp;
}

/* ================================================================
 *  互补滤波方案：
 *  - 不用 DMP 四元数算 Pitch/Roll（6X_LP_QUAT 低功耗收敛太慢）
 *  - 改用加速度计提供绝对重力参考（首帧即正确）
 *  - DMP 校准后的陀螺仪提供角速度（零偏已消除）
 *  - 自适应增益：静止时信任加速度计，运动时信任陀螺仪
 * ================================================================ */
uint8_t MPU_Check_And_Read(void) {

    long quat[4];
    unsigned long timestamp;
    short gyro[3], accel[3], sensors;
    unsigned char more;
    float q0, q1, q2, q3;

    dmp_read_fifo(gyro, accel, quat, &timestamp, &sensors, &more);

    if (sensors & INV_WXYZ_QUAT) {

        /* ====== Pitch/Roll: 互补滤波（加速度计绝对参考 + 陀螺仪角速度）====== */
        {
            // 1. 加速度计 传感器坐标系 → 机体坐标系
            //    gyro_orientation = {-1,0,0, 0,-1,0, 0,0,1} : X/Y 翻转
            float ax = -accel[0] / 16384.0f;
            float ay = -accel[1] / 16384.0f;
            float az =  accel[2] / 16384.0f;
            float norm = sqrtf(ax*ax + ay*ay + az*az);
            if (norm > 0.001f) { ax /= norm; ay /= norm; az /= norm; }

            // 2. 加速度计反算绝对倾角（重力参考，即刻正确）
            float acc_pitch = atan2f(-ax, sqrtf(ay*ay + az*az)) * 57.2958f;
            float acc_roll  = atan2f( ay, az) * 57.2958f;

            // 3. DMP 校准后角速度 deg/s
            float gx_dps = gyro[0] / 16.4f;
            float gy_dps = gyro[1] / 16.4f;
            float gz_dps = gyro[2] / 16.4f;
            Gyro_X = gx_dps;
            Gyro_Y = gy_dps;
            Gyro_Z = gz_dps;

            // 4. 互补滤波
            static float cf_pitch = 0, cf_roll = 0;
            static uint8_t cf_inited = 0;

            if (!cf_inited) {
                cf_pitch = acc_pitch;
                cf_roll  = acc_roll;
                cf_inited = 1;
            } else {
                // 陀螺积分 (deg/s * 0.005s)
                cf_pitch += gy_dps * 0.005f;
                cf_roll  += gx_dps * 0.005f;

                // 自适应增益: 加速度模长越接近 1g 越信任它
                float motion = fabsf(norm - 1.0f);
                float gain;
                if      (motion < 0.05f) gain = 0.02f;
                else if (motion < 0.2f)  gain = 0.005f;
                else                      gain = 0.0f;

                cf_pitch += (acc_pitch - cf_pitch) * gain;
                cf_roll  += (acc_roll  - cf_roll)  * gain;
            }

            Pitch = cf_pitch - Pitch_Offset;
            Roll  = cf_roll  - Roll_Offset;
        }

        /* ====== Yaw: DMP 四元数 + 死区滤波（无磁力计，无法绝对定向）====== */
        {
            q0 = quat[0] / q30;
            q1 = quat[1] / q30;
            q2 = quat[2] / q30;
            q3 = quat[3] / q30;
            float raw_yaw = atan2f(2.0f * (q1 * q2 + q0 * q3),
                                   q0 * q0 + q1 * q1 - q2 * q2 - q3 * q3) * 57.2958f;
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
        }

        /* ====== 加速度 m/s^2 (供调试) ====== */
        Accel_X = accel[0] / 16384.0f * 9.80665f;
        Accel_Y = accel[1] / 16384.0f * 9.80665f;
        Accel_Z = accel[2] / 16384.0f * 9.80665f;

        return 1;
    }
    return 0;
}

void OS_IMUCallback(void *argument)
{
  for(;;)
  {
    if(!i2c_busy)
    {
      i2c_busy = 1;
      MPU_Check_And_Read();
      i2c_busy = 0;
    }
    osDelay(1);
  }
}
