#ifndef DM_8009_H
#define DM_8009_H

#include "stdint-gcc.h"
#include "can_device.h"

typedef struct{
    uint32_t id;
    float pos_r;
    float angular_vel;
    float torque;
    uint32_t last_heartbeat_timestamp_ms;
} Dm8009;

void dm8009_init(Dm8009 *motor, uint32_t device_id);

void set_dm8009_torque(CanType can_type,
                       Dm8009SendID CMD_ID,
                       float torque);

void set_dm8009_MIT(CanType can_type, Dm8009SendID CMD_ID, float pos, float speed, float kp, float kd, float torque);

void set_dm8009_enable(CanType can_type, Dm8009SendID CMD_ID);

void set_dm8009_disable(CanType can_type, Dm8009SendID CMD_ID);

void set_dm8009_pos_speed(CanType can_type, Dm8009SendID CMD_ID, float pos_rad, float speed_rps);

void dm8009_can_msg_unpack(uint32_t id, uint8_t data[]);

#endif
