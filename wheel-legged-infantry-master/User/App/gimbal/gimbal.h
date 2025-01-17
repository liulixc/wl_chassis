#ifndef GIMBAL_H
#define GIMBAL_H

#include "robot_def.h"

typedef struct{
  ChassisCtrlMode chassis_ctrl_mode;
  ChassisCtrlInfo chassis_ctrl_info;
} GimbalMsg;

void gimbal_msg_unpack(uint32_t id, uint8_t data[]);

GimbalMsg *get_gimbal_msg();

extern void gimbal_task(void const *pvParameters);

#endif
