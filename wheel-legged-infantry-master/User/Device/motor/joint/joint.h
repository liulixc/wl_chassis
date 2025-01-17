#ifndef JOINT_H
#define JOINT_H

#include "can_device.h"
#include "dm_8009.h"
#include "cybergear.h"

enum JointMotorIndex{
    LF=0,
    LB=1,
    RB=2,
    RF=3,
};

void joint_init();

void joint_enable();

void set_joint_torque(float joint_LF_torque, float joint_LB_torque, float joint_RF_torque, float joint_RB_torque);

Dm8009* get_joint_motors();

#endif //JOINT_H
