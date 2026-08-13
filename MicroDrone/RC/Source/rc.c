#include "rc.h"
#include "SPI_rc.h"
#include "gpio.h"
#include "mpu6050.h"
#include "cmsis_os.h"

RC rc = {0};
int16_t Target_Roll = 0;
int16_t Target_Pitch = 0;
int16_t Target_Yaw = 0;
int16_t Final_Throttle = 1000;

static const uint32_t key_mask[KEY_COUNT] = {
    Key_W, Key_S, Key_A, Key_D, Key_Shift, Key_Ctrl,
    Key_Q, Key_E, Key_R, Key_F, Key_I, Key_P
};

void RC_Init(void)
{
    for(int i = 0; i < KEY_COUNT; i++) {
        rc.keyList[i].key_mask = key_mask[i];
        rc.keyList[i].press_time = 0;
        rc.keyList[i].click_flag = 0;
    }

    NF04_Init();
	
	rc.isStop_flag = 1;
	rc.isStart_flag = 0;
	rc.rc_data_ready_flag = 0;
}

void RC_Update_Status_Machine(uint32_t raw_key_value)
{
    for(int i = 0; i < KEY_COUNT; i++)
    {
        uint8_t is_pressed = (raw_key_value & rc.keyList[i].key_mask) ? 1 : 0;
        if(is_pressed)
        {
            if(rc.keyList[i].press_time == 0) 
            {
                rc.keyList[i].click_flag = 1;
            }
            else
            {
                rc.keyList[i].click_flag = 0;
            }
            rc.keyList[i].press_time++;
        }
        else
        {
            rc.keyList[i].press_time = 0;
            rc.keyList[i].click_flag = 0;
        }
    }
}

void Rc_Handler(void)
{
    if(rc.raw_buffer[0] != 0xAA) return;

    rc.current_key = (uint32_t)((rc.raw_buffer[1] << 16) | (rc.raw_buffer[2] << 8)  | rc.raw_buffer[3]);
    
    if(rc.current_key & Key_P)
    {
        rc.isStop_flag = 1;
        rc.isStart_flag = 0;
    }

    if(rc.isStop_flag == 1)
    {
		Final_Throttle = 1000;
		Target_Pitch = 0;
		Target_Roll = 0;
		Target_Yaw = 0;
    }
	
    RC_Resolve_Control_Logic();

    if((rc.isStop_flag ==1)&&(rc.current_key & Key_I)&&(rc.raw_buffer[4] < 5))
    {
        rc.isStop_flag = 0;
        rc.isStart_flag = 1;
    }
}

//void RC_Resolve_Control_Logic(void)//键鼠
//{
//    uint8_t raw_throttle = rc.raw_buffer[4];
//    RC_Update_Status_Machine(rc.current_key);

//    // 油门（0~255 → 1000~2000）
//    Final_Throttle = 1000 + (int16_t)((float)raw_throttle * 3.9215f);

//    int16_t fb_axis = 0;
//    int16_t lr_axis = 0;
//    if (rc.current_key & Key_W) fb_axis += 1;
//    if (rc.current_key & Key_S) fb_axis -= 1;
//    if (rc.current_key & Key_A) lr_axis -= 1;
//    if (rc.current_key & Key_D) lr_axis += 1;

//    int16_t move_angle = 15;
//    if ((rc.current_key & (Key_W | Key_Shift)) == (Key_W | Key_Shift))
//    {
//        move_angle = 30;
//    }

//    Target_Pitch = fb_axis * move_angle;
//    Target_Roll  = lr_axis * move_angle;

//    if (rc.current_key & Key_Q) Target_Yaw = 10;
//    else if (rc.current_key & Key_E) Target_Yaw = -10;
//    else Target_Yaw = 0;
//}

void RC_Resolve_Control_Logic(void)//摇杆
{
    RC_Update_Status_Machine(rc.current_key);
	uint8_t raw_throttle = rc.raw_buffer[4];// 油门（0~255 → 1000~2000）
	float raw_targetpitch = (rc.raw_buffer[5] - 127)/127.0f;
	float raw_targetroll = (rc.raw_buffer[6] - 127)/127.0f;
	float raw_targetyaw = (rc.raw_buffer[7] - 127)/127.0f;
	
    Final_Throttle = 1000 + (int16_t)(raw_throttle * 3.0f);
	Target_Pitch = (int16_t)(raw_targetpitch * 30);
	Target_Roll = (int16_t)(raw_targetroll * 30);
	Target_Yaw = (int16_t)(raw_targetyaw * 10);
}

void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
    if(GPIO_Pin == GPIO_PIN_2)
    {
        uint8_t status = NF04_Read_Reg(STATUS);

       if(status & 0x40)
       {
           NF04_Read_Buf(RD_RX_PLOAD, rc.raw_buffer, RC_SIZE);
           rc.rc_data_ready_flag = 1;
       }
      NF04_Write_Reg(STATUS, status);//清除中断标志，释放 IRQ 引脚。NRF 的 STATUS 寄存器写 1 清零
    }
}

void Task_RcCallback(void)
{
    static uint32_t last_send_time = 0;
    last_send_time ++;

    if (rc.rc_data_ready_flag == 1)
    {
        rc.rc_data_ready_flag = 0;
        Rc_Handler();
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
