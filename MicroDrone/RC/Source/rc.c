#include "rc.h"
#include "SPI_rc.h"

#define RC_PAYLOAD_SIZE   8

/* 飞控核心的控制目标量 */
int16_t Target_Roll = 0;
int16_t Target_Pitch = 0;
int16_t Target_Yaw = 0;
int16_t Final_Throttle = 1000;

/* 无线相关的标志位和私有接收缓冲区 */
volatile uint8_t RC_Data_Ready_Flag = 0;
//static uint8_t RC_Raw_Buffer[RC_PAYLOAD_SIZE] = {0};
uint8_t RC_Raw_Buffer[RC_PAYLOAD_SIZE] = {0};
/* 12 路按键状态表 */
#define KEY_COUNT 12
static KeyInfo_t keyList[KEY_COUNT];

static const uint32_t key_mask_table[KEY_COUNT] = {
    Key_W, Key_S, Key_A, Key_D, Key_Shift, Key_Ctrl,
    Key_Q, Key_E, Key_R, Key_F, Key_Z, Key_X
};

void RC_App_Init(void)
{
    // 初始化所有按键状态
    for(int i = 0; i < KEY_COUNT; i++) {
        keyList[i].key_mask = key_mask_table[i];
        keyList[i].press_time = 0;
        keyList[i].click_flag = 0;
    }

    // 最上层初始化入口，正式调用底层硬件初始化
    NF04_Init();
}

void RC_Update_Status_Machine(uint32_t raw_key_value)
{
    for(int i = 0; i < KEY_COUNT; i++)
    {
        uint8_t is_pressed = (raw_key_value & keyList[i].key_mask) ? 1 : 0;
        if(is_pressed)
        {
            if(keyList[i].press_time == 0) keyList[i].click_flag = 1; // 首次按下边沿检测
            else keyList[i].click_flag = 0;
            keyList[i].press_time++;
        }
        else
        {
            keyList[i].press_time = 0;
            keyList[i].click_flag = 0;
        }
    }
}

void RC_Resolve_Control_Logic(void)
{
    if(RC_Raw_Buffer[0] != 0xAA) return; // 只处理 0xAA 命令帧

    // 解析位图：byte0=0xAA帧头, byte1-3=按键位图(3字节=24bit), byte4=油门
    uint32_t current_key = (uint32_t)((RC_Raw_Buffer[1] << 16) |
                              (RC_Raw_Buffer[2] << 8)  |
                               RC_Raw_Buffer[3]);

    uint8_t raw_throttle = RC_Raw_Buffer[4];

    // 更新按键状态机
    RC_Update_Status_Machine(current_key);

    // 油门线性映射（0~255 → 1000~2000）
    Final_Throttle = 1000 + (int16_t)((float)raw_throttle * 3.9215f);

    // 计算前后左右轴
    int16_t fb_axis = 0;
    int16_t lr_axis = 0;

    if (current_key & Key_W) fb_axis += 1;
    if (current_key & Key_S) fb_axis -= 1;
    if (current_key & Key_A) lr_axis -= 1;
    if (current_key & Key_D) lr_axis += 1;

    int16_t move_speed = 15;

    // 组合键判断：W + Shift 加速前冲
    if ((current_key & (Key_W | Key_Shift)) == (Key_W | Key_Shift))
    {
        move_speed = 30;
    }

    Target_Pitch = fb_axis * move_speed;
    Target_Roll  = lr_axis * move_speed;

    // Q / E 控制偏航
    if (current_key & Key_Q) Target_Yaw = -50;
    else if (current_key & Key_E) Target_Yaw = 50;
    else Target_Yaw = 0;
}

/**
 * @brief  NRF24L01 IRQ 外部中断回调（PA2 下降沿触发）
 * @note   硬件自动读包并置标志，main 循环处理
 */
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
    if(GPIO_Pin == GPIO_PIN_2)
    {
        uint8_t status = NF04_Read_Reg(STATUS);

//        if(status & 0x40)
//        {
//            NF04_Read_Buf(RD_RX_PLOAD, RC_Raw_Buffer, RC_PAYLOAD_SIZE);
//            RC_Data_Ready_Flag = 1; // 收包成功，通知主循环
//        }
            NF04_Read_Buf(RD_RX_PLOAD, RC_Raw_Buffer, RC_PAYLOAD_SIZE);
            RC_Data_Ready_Flag = 1; // 收包成功，通知主循环

        NF04_Write_Reg(STATUS, status);
    }
}
