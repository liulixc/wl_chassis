#include <stdbool.h>
#include <math.h>
#include <cmsis_os.h>
#include <stm32f4xx_hal.h>
#include <stm32f4xx_hal_can.h>

#include "chassis.h"
#include "gimbal.h"

#include "lqr.h"
#include "Atti.h"
#include "user_lib.h"
#include "moving_filter.h"
#include "vx_kalman_filter.h"

#include "joint.h"
#include "wheel.h"
#include "remote.h"
#include "error.h"


extern CAN_HandleTypeDef hcan1;
extern CAN_HandleTypeDef hcan2;

//int jump_finish = 0;

extern Chassis chassis;

extern KalmanFilter_t vaEstimateKF;


/*******************************************************************************
 *                                    Remote                                   *
 *******************************************************************************/


/** 模块离线处理 **/
static void chassis_device_offline_handle() {
    check_is_rc_online(get_rc_ctrl());
    if (get_errors() != 0) {
        chassis.chassis_ctrl_mode = CHASSIS_DISABLE;
    }
}

/** 底盘接收遥控器信息 **/
static void set_chassis_ctrl_info() {
    chassis.chassis_ctrl_info.v_m_per_s = (float) (get_rc_ctrl()->rc.ch[CHASSIS_SPEED_CHANNEL]) * RC_TO_VX;
    chassis.chassis_ctrl_info.x = chassis.chassis_ctrl_info.x + CHASSIS_PERIOD * 0.001f * chassis.chassis_ctrl_info.v_m_per_s;

    chassis.chassis_ctrl_info.yaw_angle_rad -= (float) (get_rc_ctrl()->rc.ch[CHASSIS_YAW_CHANNEL]) * (-RC_TO_YAW_INCREMENT);
    chassis.chassis_ctrl_info.height_m = 0.18f;

//    chassis.chassis_ctrl_info.height_m = chassis.chassis_ctrl_info.height_m + (float) (get_rc_ctrl()->rc.ch[TEST_CHASSIS_LEG_CHANNEL]) * 0.00001f;
//    VAL_LIMIT(chassis.chassis_ctrl_info.height_m, MIN_L0, MAX_L0);

//    chassis.chassis_ctrl_info.roll_angle_rad = (float) (get_rc_ctrl()->rc.ch[TEST_CHASSIS_ROLL_CHANNEL]) * 0.001f;

}

/** 底盘根据遥控器设置模式 **/
static void set_chassis_mode() {
    if (switch_is_down(get_rc_ctrl()->rc.s[RC_s_R])) { // 失能
        chassis.chassis_ctrl_mode_last = chassis.chassis_ctrl_mode;
        chassis.chassis_ctrl_mode = CHASSIS_DISABLE;
    } else if (switch_is_mid(get_rc_ctrl()->rc.s[RC_s_R]) && chassis.init_flag == false) { // 初始化模式
        chassis.chassis_ctrl_mode_last = chassis.chassis_ctrl_mode;
        chassis.chassis_ctrl_mode = CHASSIS_INIT;
    } else if (switch_is_mid(get_rc_ctrl()->rc.s[RC_s_R]) && chassis.init_flag == true) { // 使能
        chassis.chassis_ctrl_mode_last = chassis.chassis_ctrl_mode;
        chassis.chassis_ctrl_mode = CHASSIS_ENABLE;

        chassis.chassis_ctrl_info.height_m = 0.18f;

        if(switch_is_down(get_rc_ctrl()->rc.s[RC_s_L])){
            chassis.chassis_ctrl_info.height_m = 0.10f;
        }
        else if(switch_is_mid(get_rc_ctrl()->rc.s[RC_s_L])){
            chassis.chassis_ctrl_info.height_m = 0.18f;
        }
        else if(switch_is_up(get_rc_ctrl()->rc.s[RC_s_L])){
            chassis.chassis_ctrl_info.height_m = 0.35f;
        }
    }
}

/** 底盘通过板间通信接收云台的信息 **/
static void set_chassis_ctrl_info_from_gimbal_msg() {
    chassis.chassis_ctrl_info.v_m_per_s = get_gimbal_msg()->chassis_ctrl_info.v_m_per_s;
    chassis.chassis_ctrl_info.x = chassis.chassis_ctrl_info.x + CHASSIS_PERIOD * 0.001f * chassis.chassis_ctrl_info.v_m_per_s;
    chassis.chassis_ctrl_info.yaw_angle_rad = get_gimbal_msg()->chassis_ctrl_info.yaw_angle_rad;
    chassis.chassis_ctrl_info.roll_angle_rad = get_gimbal_msg()->chassis_ctrl_info.roll_angle_rad;
//    chassis.chassis_ctrl_info.height_m = get_gimbal_msg()->chassis_ctrl_info.height_m;
    chassis.chassis_ctrl_info.height_m = 0.18f;
    VAL_LIMIT(chassis.chassis_ctrl_info.height_m, MIN_L0, MAX_L0);
}

/** 底盘根据云台信息设置模式 **/
static void set_chassis_mode_from_gimbal_msg() {
    if (get_gimbal_msg()->chassis_ctrl_mode == CHASSIS_DISABLE) {
        chassis.chassis_ctrl_mode_last = chassis.chassis_ctrl_mode;
        chassis.chassis_ctrl_mode = CHASSIS_DISABLE;
    } else if (get_gimbal_msg()->chassis_ctrl_mode == CHASSIS_ENABLE && chassis.init_flag == false) {
        chassis.chassis_ctrl_mode_last = chassis.chassis_ctrl_mode;
        chassis.chassis_ctrl_mode = CHASSIS_INIT;
    } else if (get_gimbal_msg()->chassis_ctrl_mode == CHASSIS_ENABLE && chassis.init_flag == true) {
        chassis.chassis_ctrl_mode_last = chassis.chassis_ctrl_mode;
        chassis.chassis_ctrl_mode = CHASSIS_ENABLE;
    } else if (get_gimbal_msg()->chassis_ctrl_mode == CHASSIS_SPIN)
    {
        chassis.chassis_ctrl_mode_last = chassis.chassis_ctrl_mode;
        chassis.chassis_ctrl_mode = CHASSIS_SPIN;
    }
}

/*******************************************************************************
 *                                 Function                                    *
 *******************************************************************************/

/*********************** 获取底盘传感器信息 *************************/
static void get_IMU_info() {

    /** Yaw **/
    chassis.imu_reference.yaw_angle = -*(get_ins_angle() + 0);

    // 圈数检测
    if (chassis.imu_reference.yaw_angle - chassis.imu_reference.yaw_last_angle > 3.1415926f)
    {
        chassis.imu_reference.yaw_round_count--;
    }
    else if (chassis.imu_reference.yaw_angle - chassis.imu_reference.yaw_last_angle < -3.1415926f)
    {
        chassis.imu_reference.yaw_round_count++;
    }
    chassis.imu_reference.yaw_total_angle = 6.283f * chassis.imu_reference.yaw_round_count + chassis.imu_reference.yaw_angle;
    chassis.imu_reference.yaw_last_angle = chassis.imu_reference.yaw_angle;

    /** Pitch **/
    chassis.imu_reference.pitch_angle = -*(get_ins_angle() + 2);

    /** Roll **/
    chassis.imu_reference.roll_angle = -*(get_ins_angle() + 1);

    /** 更新各轴加速度和角速度 **/
    chassis.imu_reference.pitch_gyro = -*(get_ins_gyro() + 0);
    chassis.imu_reference.yaw_gyro = -*(get_ins_gyro() + 2);
    chassis.imu_reference.roll_gyro = -*(get_ins_gyro() + 1);

    chassis.imu_reference.ax = -*(get_ins_accel() + 1);
    chassis.imu_reference.ay = *(get_ins_accel() + 0);
    chassis.imu_reference.az = *(get_ins_accel() + 2);

    /** 去除重力影响的各轴加速度  旋转矩阵法 **/
    chassis.imu_reference.ax_filtered = chassis.imu_reference.ax - GRAVITY * sinf(chassis.imu_reference.pitch_angle);
    chassis.imu_reference.ay_filtered =  chassis.imu_reference.ay
                                         - GRAVITY * cosf(chassis.imu_reference.pitch_angle) * sinf(chassis.imu_reference.roll_angle);
    chassis.imu_reference.az_filtered =  chassis.imu_reference.az
                                         - GRAVITY * cosf(chassis.imu_reference.pitch_angle) * cosf(chassis.imu_reference.roll_angle);

    /** 机体竖直方向加速度 **/
    float robot_az_raw =  chassis.imu_reference.ax_filtered * sinf(chassis.imu_reference.pitch_angle)
                          + chassis.imu_reference.ay_filtered * sinf(-chassis.imu_reference.roll_angle) * cosf(chassis.imu_reference.pitch_angle)
                          + chassis.imu_reference.az_filtered * cosf(chassis.imu_reference.pitch_angle) * cosf(chassis.imu_reference.roll_angle);

    update_moving_average_filter(&chassis.robot_az_filter, robot_az_raw);
    chassis.imu_reference.robot_az = get_moving_average_filtered_value(&chassis.robot_az_filter);

}

/************************ 向底盘电机发送力矩 **********************/
static void chassis_motor_cmd_send() {

/** DEBUG_MODE: 置1时进入调试模式，关闭关节和轮毂输出 **/
#if DEBUG_MODE
    set_joint_torque(0, 0, 0, 0);
    osDelay(2);
    set_wheel_torque(0, 0);

#else

    //  set_joint_torque(0, 0, 0, 0);

    //  set_wheel_torque(0, 0);

    set_joint_torque(-chassis.leg_L.joint_F_torque,
                     -chassis.leg_L.joint_B_torque,
                     chassis.leg_R.joint_F_torque,
                     chassis.leg_R.joint_B_torque);

    set_wheel_torque(-chassis.leg_L.wheel_torque, -chassis.leg_R.wheel_torque);



#endif
}

/************************ 底盘pid初始化 **********************/
static void chassis_pid_init() {

    /** 转向PID **/
    pid_init(&chassis.chassis_turn_pid,
             CHASSIS_TURN_PID_OUT_LIMIT,
             CHASSIS_TURN_PID_IOUT_LIMIT,
             CHASSIS_TURN_PID_P,
             CHASSIS_TURN_PID_I,
             CHASSIS_TURN_PID_D);

    /** 腿长PID **/
    pid_init(&chassis.leg_L.leg_pos_pid,
             CHASSIS_LEG_L0_POS_PID_OUT_LIMIT,
             CHASSIS_LEG_L0_POS_PID_IOUT_LIMIT,
             CHASSIS_LEG_L0_POS_PID_P,
             CHASSIS_LEG_L0_POS_PID_I,
             CHASSIS_LEG_L0_POS_PID_D);

    pid_init(&chassis.leg_R.leg_pos_pid,
             CHASSIS_LEG_L0_POS_PID_OUT_LIMIT,
             CHASSIS_LEG_L0_POS_PID_IOUT_LIMIT,
             CHASSIS_LEG_L0_POS_PID_P,
             CHASSIS_LEG_L0_POS_PID_I,
             CHASSIS_LEG_L0_POS_PID_D);

    pid_init(&chassis.leg_L.leg_speed_pid,
             CHASSIS_LEG_L0_SPEED_PID_OUT_LIMIT,
             CHASSIS_LEG_L0_SPEED_PID_IOUT_LIMIT,
             CHASSIS_LEG_L0_SPEED_PID_P,
             CHASSIS_LEG_L0_SPEED_PID_I,
             CHASSIS_LEG_L0_SPEED_PID_D);

    pid_init(&chassis.leg_R.leg_speed_pid,
             CHASSIS_LEG_L0_SPEED_PID_OUT_LIMIT,
             CHASSIS_LEG_L0_SPEED_PID_IOUT_LIMIT,
             CHASSIS_LEG_L0_SPEED_PID_P,
             CHASSIS_LEG_L0_SPEED_PID_I,
             CHASSIS_LEG_L0_SPEED_PID_D);

    // 离地后的腿长PID
    pid_init(&chassis.leg_L.offground_leg_pid,
             CHASSIS_OFFGROUND_L0_PID_OUT_LIMIT,
             CHASSIS_OFFGROUND_L0_PID_IOUT_LIMIT,
             CHASSIS_OFFGROUND_LO_PID_P,
             CHASSIS_OFFGROUND_L0_PID_I,
             CHASSIS_OFFGROUND_L0_PID_D);

    pid_init(&chassis.leg_R.offground_leg_pid,
             CHASSIS_OFFGROUND_L0_PID_OUT_LIMIT,
             CHASSIS_OFFGROUND_L0_PID_IOUT_LIMIT,
             CHASSIS_OFFGROUND_LO_PID_P,
             CHASSIS_OFFGROUND_L0_PID_I,
             CHASSIS_OFFGROUND_L0_PID_D);

    /** 防劈叉PID **/
    pid_init(&chassis.chassis_leg_coordination_pid,
             CHASSIS_LEG_COORDINATION_PID_OUT_LIMIT,
             CHASSIS_LEG_COORDINATION_PID_IOUT_LIMIT,
             CHASSIS_LEG_COORDINATION_PID_P,
             CHASSIS_LEG_COORDINATION_PID_I,
             CHASSIS_LEG_COORDINATION_PID_D);

    /** Roll PID **/
    pid_init(&chassis.chassis_roll_pid,
             CHASSIS_ROLL_PID_OUT_LIMIT,
             CHASSIS_ROLL_PID_IOUT_LIMIT,
             CHASSIS_ROLL_PID_P,
             CHASSIS_ROLL_PID_I,
             CHASSIS_ROLL_PID_D);

    pid_init(&chassis.chassis_vw_speed_pid,
             30,
             10,
             -5,
             -0.1f,
             0);
    pid_init(&chassis.chassis_vw_pos_pid,
             4,
             0,
             5 ,0,50);

    pid_init(&chassis.chassis_spin_pid,
             30,10,
             -5,-0.1f,0);
}

/************************ 底盘相关参数初始化 **********************/
static void chassis_init() {

    /** 初始化底盘模式 **/
    chassis.chassis_ctrl_mode = CHASSIS_DISABLE;

    /** 轮毂电机初始化 **/
    wheel_init();

    /** 关节电机初始化 **/
    joint_init();

    /** 底盘pid初始化 **/
    chassis_pid_init();

    /** 移动平均滤波器初始化 **/
    moving_average_filter_init(&chassis.robot_az_filter);
    moving_average_filter_init(&chassis.leg_L.Fn_filter);
    moving_average_filter_init(&chassis.leg_R.Fn_filter);
    moving_average_filter_init(&chassis.leg_L.theta_ddot_filter);
    moving_average_filter_init(&chassis.leg_R.theta_ddot_filter);

//  chassis.chassis_ctrl_info.spin_speed = 5.0f;

    vTaskSuspendAll();

    /** 轮毂-速度融合加速度 卡尔曼滤波器 初始化 **/
    xvEstimateKF_Init(&vaEstimateKF);

    xTaskResumeAll();
}

// 倒地后以最低腿长起身
static void chassis_selfhelp(void)
{
    if(ABS(chassis.imu_reference.pitch_angle) > 0.1395f) // -8°~ 8°
    {
        chassis.leg_L.joint_F_torque = 0.0f;
        chassis.leg_L.joint_B_torque = 0.0f;
        chassis.leg_R.joint_F_torque = 0.0f;
        chassis.leg_R.joint_B_torque = 0.0f;

        chassis.is_chassis_balance = false;
    }
    else{
        chassis.is_chassis_balance = true;
    }



//    if(ABS(chassis.imu_reference.pitch_angle) > 0.1395f) // -8°~ 8°
//    {
//        chassis.chassis_ctrl_info.height_m = MIN_L0;
//
//        chassis.is_chassis_balance = false;
//    }
//    else{
//        chassis.is_chassis_balance = true;
//    }
}

static void is_chassis_offground(void)
{
    if(chassis.leg_L.leg_is_offground && chassis.leg_R.leg_is_offground)
    {
        chassis.chassis_is_offground = true;
    }else{
        chassis.chassis_is_offground = false;
    }
}

/*********************************************************************************/




/*******************************************************************************
 *                                    Task                                     *
 *******************************************************************************/


/************************* 失能任务 ***************************/
static void chassis_disable_task() {

    chassis.leg_L.wheel_torque = 0;
    chassis.leg_R.wheel_torque = 0;
    chassis.leg_L.joint_F_torque = 0;
    chassis.leg_L.joint_B_torque = 0;
    chassis.leg_R.joint_F_torque = 0;
    chassis.leg_R.joint_B_torque = 0;

    chassis.chassis_ctrl_mode = CHASSIS_DISABLE;

    chassis.leg_L.state_variable_feedback.x = chassis.chassis_ctrl_info.x;
    chassis.leg_R.state_variable_feedback.x = chassis.chassis_ctrl_info.x;

    chassis.chassis_ctrl_info.yaw_angle_rad = chassis.imu_reference.yaw_total_angle;

    chassis.chassis_ctrl_info.height_m = MIN_L0;


    /** 初始化标志位 **/
    // 底盘初始化标志位
    chassis.init_flag = false;

    // 平衡标志位
    chassis.is_chassis_balance = false;
    // 倒地自救成功标志位
    chassis.recover_finish = false;

    // 离地标志位
    chassis.leg_L.leg_is_offground = false;
    chassis.leg_R.leg_is_offground = false;
    chassis.chassis_is_offground   = false;

    // 跳跃标志位
    chassis.jump_flag = false;

    /**
     * 因为LK电机上电默认使能，之前有过遥控器失能后轮毂电机依然疯转的现象，
     * 因此选择在INIT模式再使能，并且失能后停止轮毂电机，但此时轮毂仍可以接受信息并产生动作
     * **/
    wheel_stop();
}

/*********************** 初始化任务 ***************************/
static void chassis_init_task() {

    joint_enable();
    wheel_enable();

    chassis.init_flag = true;
}

/*********************** 使能任务 ****************************/
static void chassis_enable_task() {

    lqr_ctrl();
    vmc_ctrl();
    chassis_vx_kalman_run();

    chassis_selfhelp();

}

/*********************** 总任务 ******************************/
extern void chassis_task(void const *pvParameters) {

    chassis_init();

    // ???
    TickType_t last_wake_time = xTaskGetTickCount();

    while (1) {

        get_IMU_info();

#if CHASSIS_REMOTE
        set_chassis_mode();

        set_chassis_ctrl_info();

        chassis_device_offline_handle();
#else
        set_chassis_mode_from_gimbal_msg();
        set_chassis_ctrl_info_from_gimbal_msg();
#endif

        switch (chassis.chassis_ctrl_mode) {

            case CHASSIS_DISABLE:
                chassis_disable_task();
                break;

            case CHASSIS_INIT:
                chassis_init_task();
                break;

            case CHASSIS_ENABLE:
            case CHASSIS_SPIN:
                chassis_enable_task();
                break;

            default:break;
        }

        chassis_motor_cmd_send();

        vTaskDelayUntil(&last_wake_time, CHASSIS_PERIOD);
    }
}
/*********************************************************************************************/
