//
// Created by liuli on 25-3-9.
//

#ifndef WHEEL_LEG_BEAR_CAP_H
#define WHEEL_LEG_BEAR_CAP_H

#include <stdint-gcc.h>
#include <main.h>

void cap_task(void const *pvParameters);
#define CAP_TASK_INIT_TIME 357


typedef enum {
    CAP_OFF,
    CAP_ON,
}cap_cmd_e;

typedef struct {
    float input_value;
    float input_current;
    float cap_value;
    float target_power;

    float max_cap_voltage;
    float percentage;
    float min_percentage;
    cap_cmd_e ctrl_cmd;

    float chassis_power;

    float bat_voltage;
    float min_voltage;
}cap_info_t;

typedef struct {
    uint8_t send_data[6];
    uint8_t receive_data[6];

    cap_cmd_e mode;
    float bat_voltage;

    union {

        uint16_t charge_current;

    };

    int16_t remain_power;
    uint8_t charge_status;
    uint8_t rec_cap_cmd;
    uint16_t cap_voltage;
    uint16_t chassis_current;
}__packed cap2_info_t;

extern void cap_info_update();
#endif //WHEEL_LEG_BEAR_CAP_H
