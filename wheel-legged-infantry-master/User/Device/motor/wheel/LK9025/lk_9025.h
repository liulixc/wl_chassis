#ifndef LK_9025_H
#define LK_9025_H

#include <stdint-gcc.h>
#include "can_device.h"

typedef struct{
    uint32_t id;
    float angular_vel;
    float torque;
    uint32_t last_heartbeat_timestamp_ms;
} Lk9025;

void lk9025_init(Lk9025 *motor, uint32_t device_id);

void lk9025_stop(CanType can_type, Lk9025SendID CMD_ID);

void lk9025_set_enable(CanType can_type, Lk9025SendID CMD_ID);

void lk9025_torque_set(CanType can_type, Lk9025SendID CMD_ID, float motor_torque);

void lk9025_can_msg_unpack(uint32_t id, uint8_t data[]);

#endif
