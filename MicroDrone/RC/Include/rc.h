#ifndef __RC_H
#define __RC_H

#include <stdint.h>
#include "main.h"

typedef enum _KeyType
{
  Key_W     = 1 << 0,
  Key_S     = 1 << 1,
  Key_A     = 1 << 2,
  Key_D     = 1 << 3,
  Key_Shift = 1 << 4,
  Key_Ctrl  = 1 << 5,
  Key_Q     = 1 << 6,
  Key_E     = 1 << 7,
  Key_R     = 1 << 8,
  Key_F     = 1 << 9,
  Key_Z     = 1 << 10,
  Key_P     = 1 << 11,
} KeyType;

typedef struct {
    uint32_t key_mask;
    uint32_t press_time;
    uint8_t  click_flag;
} KeyInfo_t;

extern volatile uint8_t RC_Data_Ready_Flag;
extern int16_t Target_Roll, Target_Pitch, Target_Yaw, Final_Throttle;

void RC_Init(void);
void RC_Update_Status_Machine(uint32_t raw_key_value);
void RC_Resolve_Control_Logic(void);

#endif
