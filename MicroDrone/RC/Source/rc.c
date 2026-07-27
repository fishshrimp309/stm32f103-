#include "rc.h"
#include "SPI_rc.h"
#include "gpio.h"
#include "mpu6050.h"
#include "cmsis_os.h"

#define RC_PAYLOAD_SIZE   8

int16_t Target_Roll = 0;
int16_t Target_Pitch = 0;
int16_t Target_Yaw = 0;
int16_t Final_Throttle = 1000;

volatile uint8_t RC_Data_Ready_Flag = 0;
uint8_t RC_Raw_Buffer[RC_PAYLOAD_SIZE] = {0};
/* 12 路按键状态表 */
#define KEY_COUNT 12
static KeyInfo_t keyList[KEY_COUNT];

static const uint32_t key_mask[KEY_COUNT] = {
    Key_W, Key_S, Key_A, Key_D, Key_Shift, Key_Ctrl,
    Key_Q, Key_E, Key_R, Key_F, Key_Z, Key_P
};

void RC_Init(void)
{
    for(int i = 0; i < KEY_COUNT; i++) {
        keyList[i].key_mask = key_mask[i];
        keyList[i].press_time = 0;
        keyList[i].click_flag = 0;
    }

    NF04_Init();
}

void RC_Update_Status_Machine(uint32_t raw_key_value)
{
    for(int i = 0; i < KEY_COUNT; i++)
    {
        uint8_t is_pressed = (raw_key_value & keyList[i].key_mask) ? 1 : 0;
        if(is_pressed)
        {
            if(keyList[i].press_time == 0) 
            {
                keyList[i].click_flag = 1;
            }
            else
            {
                keyList[i].click_flag = 0;
            }
            keyList[i].press_time++;
        }
        else
        {
            keyList[i].press_time = 0;
            keyList[i].click_flag = 0;
        }
    }
}

void Rc_Handler(void)
{
    if(RC_Raw_Buffer[0] != 0xAA) return;

    uint32_t current_key = (uint32_t)((RC_Raw_Buffer[1] << 16) | (RC_Raw_Buffer[2] << 8)  | RC_Raw_Buffer[3]);
    if(current_key & Key_P)
    {
        memset(RC_Raw_Buffer, 0, RC_PAYLOAD_SIZE);
    }
}

void RC_Resolve_Control_Logic(void)
{
    uint8_t raw_throttle = RC_Raw_Buffer[4];

    RC_Update_Status_Machine(current_key);

    // 油门（0~255 → 1000~2000）
    Final_Throttle = 1000 + (int16_t)((float)raw_throttle * 3.9215f);

    int16_t fb_axis = 0;
    int16_t lr_axis = 0;

    if (current_key & Key_W) fb_axis -= 1;
    if (current_key & Key_S) fb_axis += 1;
    if (current_key & Key_A) lr_axis += 1;
    if (current_key & Key_D) lr_axis -= 1;

    int16_t move_speed = 15;

    if ((current_key & (Key_W | Key_Shift)) == (Key_W | Key_Shift))
    {
        move_speed = 30;
    }

    Target_Pitch = fb_axis * move_speed;
    Target_Roll  = lr_axis * move_speed;

    if (current_key & Key_Q) Target_Yaw = -5;
    else if (current_key & Key_E) Target_Yaw = 5;
    else Target_Yaw = 0;
}

void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
    if(GPIO_Pin == GPIO_PIN_2)
    {
        uint8_t status = NF04_Read_Reg(STATUS);

       if(status & 0x40)
       {
           NF04_Read_Buf(RD_RX_PLOAD, RC_Raw_Buffer, RC_PAYLOAD_SIZE);
           RC_Data_Ready_Flag = 1;
       }
      NF04_Write_Reg(STATUS, status);//清除中断标志，释放 IRQ 引脚。NRF 的 STATUS 寄存器写 1 清零
    }
}

void Task_RcCallback(void)
{
    static uint32_t last_send_time = 0;
    last_send_time ++;

    if (RC_Data_Ready_Flag == 1)
    {
        RC_Data_Ready_Flag = 0;
        RC_Resolve_Control_Logic();
    }


    if (last_send_time >= 50)
    {
        last_send_time = 0;

        uint8_t ack[8];
        ack[0] = 0xBB;
        ack[1] = (int8_t)Roll;
        ack[2] = (int8_t)Pitch;
        ack[3] = (int8_t)Yaw;
        ack[4] = Final_Throttle & 0xFF;
        ack[5] = (Final_Throttle >> 8) & 0xFF;
        ack[6] = 0;
        ack[7] = 0;

        if (NF04_Send_Packet(ack) == TX_DS) {
            HAL_GPIO_TogglePin(GPIOC, GPIO_PIN_13);
        }
    }
}

void OS_RcCallback(void *argument)
{
    for(;;)
    {
        Task_RcCallback();
        osDelay(1);
    }
}
