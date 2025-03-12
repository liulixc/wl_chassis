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
#include "ramp.h"

#include "joint.h"
#include "wheel.h"
#include "remote.h"
#include "error.h"
#include "Referee.h"

extern CAN_HandleTypeDef hcan1;
extern CAN_HandleTypeDef hcan2;

//int jump_finish = 0;
extern Referee_info_t Referee;
extern Chassis chassis;

extern KalmanFilter_t vaEstimateKF;


extern float motor_torque_l,motor_w_l;
extern float motor_torque_r,motor_w_r;
float power_nihe;

ramp_function_source_t  chassis_vx_ramp;


/*******************************************************************************
 *                                    Remote                                   *
 *******************************************************************************/


/** ģ�����ߴ��� **/
static void chassis_device_offline_handle() {
    check_is_rc_online(get_rc_ctrl());
    if (get_errors() != 0) {
        chassis.chassis_ctrl_mode = CHASSIS_DISABLE;
    }
}

/** ���̽���ң������Ϣ **/
static void set_chassis_ctrl_info() {
    float temp_v_m_per_s = (float) (get_rc_ctrl()->rc.ch[CHASSIS_SPEED_CHANNEL]) * RC_TO_VX;
    ramp_calc(&chassis_vx_ramp,temp_v_m_per_s);
    chassis.chassis_ctrl_info.v_m_per_s=chassis_vx_ramp.out;
    chassis.chassis_ctrl_info.x = chassis.chassis_ctrl_info.x + CHASSIS_PERIOD * 0.001f * chassis.chassis_ctrl_info.v_m_per_s;

    chassis.chassis_ctrl_info.yaw_angle_rad -= (float) (get_rc_ctrl()->rc.ch[CHASSIS_YAW_CHANNEL]) * (-RC_TO_YAW_INCREMENT);
//    chassis.chassis_ctrl_info.height_m = 0.18f;

    if(switch_is_down(get_rc_ctrl()->rc.s[RC_s_L])){
        chassis.chassis_ctrl_info.height_m = 0.10f;
    }
    else if(switch_is_mid(get_rc_ctrl()->rc.s[RC_s_L])){
        chassis.chassis_ctrl_info.height_m = 0.18f;
    }
    else if(switch_is_up(get_rc_ctrl()->rc.s[RC_s_L])){
        chassis.chassis_ctrl_info.height_m = 0.35f;
    }

//    chassis.chassis_ctrl_info.roll_angle_rad = (float) (get_rc_ctrl()->rc.ch[TEST_CHASSIS_ROLL_CHANNEL]) * 0.001f;

}

/** ���̸���ң��������ģʽ **/
static void set_chassis_mode() {
    if (switch_is_down(get_rc_ctrl()->rc.s[RC_s_R])) { // ʧ��
        chassis.chassis_ctrl_mode_last = chassis.chassis_ctrl_mode;
        chassis.chassis_ctrl_mode = CHASSIS_DISABLE;
    } else if (switch_is_mid(get_rc_ctrl()->rc.s[RC_s_R]) && chassis.init_flag == false) { // ��ʼ��ģʽ
        chassis.chassis_ctrl_mode_last = chassis.chassis_ctrl_mode;
        chassis.chassis_ctrl_mode = CHASSIS_INIT;
    } else if (switch_is_mid(get_rc_ctrl()->rc.s[RC_s_R]) && chassis.init_flag == true) { // ʹ��
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

/** ����ͨ�����ͨ�Ž�����̨����Ϣ **/
static void set_chassis_ctrl_info_from_gimbal_msg() {
    chassis.chassis_ctrl_info.v_m_per_s = get_gimbal_msg()->chassis_ctrl_info.v_m_per_s;
//    if (Referee.GameRobotStat.chassis_power_limit==80)chassis.chassis_ctrl_info.v_m_per_s*=1.25f;
    chassis.chassis_ctrl_info.x = chassis.chassis_ctrl_info.x + CHASSIS_PERIOD * 0.001f * chassis.chassis_ctrl_info.v_m_per_s;
    chassis.chassis_ctrl_info.yaw_angle_rad = get_gimbal_msg()->chassis_ctrl_info.yaw_angle_rad;
    chassis.chassis_ctrl_info.roll_angle_rad = get_gimbal_msg()->chassis_ctrl_info.roll_angle_rad;
    chassis.chassis_ctrl_info.height_m = get_gimbal_msg()->chassis_ctrl_info.height_m;
//    chassis.chassis_ctrl_info.height_m = 0.18f;
    VAL_LIMIT(chassis.chassis_ctrl_info.height_m, MIN_L0, MAX_L0);
}

/** ���̸�����̨��Ϣ����ģʽ **/
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

/*********************** ��ȡ���̴�������Ϣ *************************/
static void get_IMU_info() {

    /** Yaw **/
    chassis.imu_reference.yaw_angle = -*(get_ins_angle() + 0);

    // Ȧ�����
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

    /** ���¸�����ٶȺͽ��ٶ� **/
    chassis.imu_reference.pitch_gyro = -*(get_ins_gyro() + 0);
    chassis.imu_reference.yaw_gyro = -*(get_ins_gyro() + 2);
    chassis.imu_reference.roll_gyro = -*(get_ins_gyro() + 1);

    chassis.imu_reference.ax = -*(get_ins_accel() + 1);
    chassis.imu_reference.ay = *(get_ins_accel() + 0);
    chassis.imu_reference.az = *(get_ins_accel() + 2);

    /** ȥ������Ӱ��ĸ�����ٶ�  ��ת���� **/
    chassis.imu_reference.ax_filtered = chassis.imu_reference.ax - GRAVITY * sinf(chassis.imu_reference.pitch_angle);
    chassis.imu_reference.ay_filtered =  chassis.imu_reference.ay
                                         - GRAVITY * cosf(chassis.imu_reference.pitch_angle) * sinf(chassis.imu_reference.roll_angle);
    chassis.imu_reference.az_filtered =  chassis.imu_reference.az
                                         - GRAVITY * cosf(chassis.imu_reference.pitch_angle) * cosf(chassis.imu_reference.roll_angle);

    /** ������ֱ������ٶ� **/
    float robot_az_raw =  chassis.imu_reference.ax_filtered * sinf(chassis.imu_reference.pitch_angle)
                          + chassis.imu_reference.ay_filtered * sinf(-chassis.imu_reference.roll_angle) * cosf(chassis.imu_reference.pitch_angle)
                          + chassis.imu_reference.az_filtered * cosf(chassis.imu_reference.pitch_angle) * cosf(chassis.imu_reference.roll_angle);

    update_moving_average_filter(&chassis.robot_az_filter, robot_az_raw);
    chassis.imu_reference.robot_az = get_moving_average_filtered_value(&chassis.robot_az_filter);

}

/************************ ����̵���������� **********************/
static void chassis_motor_cmd_send() {

/** DEBUG_MODE: ��1ʱ�������ģʽ���رչؽں������� **/
#if DEBUG_MODE
    set_joint_torque(0, 0, 0, 0);
    osDelay(2);
    set_wheel_torque(0, 0);

#else

//      set_joint_torque(0, 0, 0, 0);


//      set_wheel_torque(0, 0);


        set_joint_torque(-chassis.leg_L.joint_F_torque,
                         -chassis.leg_L.joint_B_torque,
                         chassis.leg_R.joint_F_torque,
                         chassis.leg_R.joint_B_torque);


    set_wheel_torque(chassis.leg_L.wheel_torque, chassis.leg_R.wheel_torque);




#endif


//实测下面这套功率拟合基本能用,我需要用发给电机的力矩重新拟合一下功率看看跟裁判系统反馈功率的不同，然后看看是不是右边轮毂力矩取反的问题


        power_nihe=3.1604*motor_torque_l*motor_torque_l+3.3810*motor_torque_r*motor_torque_r
                +0.0140*motor_torque_l*motor_w_l+0.0142*motor_torque_r*motor_w_r
                +0.6685+0.3708
                +0.0116* fabs(motor_w_l)+0.0102* fabs(motor_w_r);
    if (power_nihe<0)power_nihe=0;

}

/************************ ����pid��ʼ�� **********************/
static void chassis_pid_init() {

    /** ת��PID **/
    pid_init(&chassis.chassis_turn_pid,
             CHASSIS_TURN_PID_OUT_LIMIT,
             CHASSIS_TURN_PID_IOUT_LIMIT,
             CHASSIS_TURN_PID_P,
             CHASSIS_TURN_PID_I,
             CHASSIS_TURN_PID_D);

    /** �ȳ�PID **/
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

    // ��غ���ȳ�PID
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

    /** ������PID **/
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

/************************ ������ز�����ʼ�� **********************/
static void chassis_init() {

    /** ��ʼ������ģʽ **/
    chassis.chassis_ctrl_mode = CHASSIS_DISABLE;

    /** ��챵����ʼ�� **/
    wheel_init();

    /** �ؽڵ����ʼ�� **/
    joint_init();

    /** ����pid��ʼ�� **/
    chassis_pid_init();

    /** �ƶ�ƽ���˲�����ʼ�� **/
    moving_average_filter_init(&chassis.robot_az_filter);
    moving_average_filter_init(&chassis.leg_L.Fn_filter);
    moving_average_filter_init(&chassis.leg_R.Fn_filter);
    moving_average_filter_init(&chassis.leg_L.theta_ddot_filter);
    moving_average_filter_init(&chassis.leg_R.theta_ddot_filter);

    //斜坡函数初始化
    ramp_init(&chassis_vx_ramp,0.002f,0.002f);

//  chassis.chassis_ctrl_info.spin_speed = 5.0f;

    vTaskSuspendAll();

    /** ���-�ٶ��ںϼ��ٶ� �������˲��� ��ʼ�� **/
    xvEstimateKF_Init(&vaEstimateKF);

    xTaskResumeAll();
}

//static bool is_joints_reduced(void)
//{
//    if (ABS(get_joint_motors()[0].pos_r+1.5707963) <= 0.25 &&
//        ABS(get_joint_motors()[1].pos_r-1.5707963) <= 0.25 &&
//        ABS(get_joint_motors()[2].pos_r+1.5707963) <= 0.25 &&
//        ABS(get_joint_motors()[3].pos_r-1.5707963) <= 0.25) {
//        return true;
//    } else {
//        return false;
//    }
//}

// ���غ�������ȳ�����
static void chassis_selfhelp(void)
{
    if(ABS(chassis.imu_reference.pitch_angle) > 0.1395f) // -8��~ 8��
    {
//        if(!is_joints_reduced())//腿部没有收起来
//        {
//            chassis.leg_L.wheel_torque = 0;
//            chassis.leg_R.wheel_torque = 0;
//
//            set_dm8009_MIT(CAN_2, JOINT_LF_SEND, -1.5707963, 0, 20, 5, 0);
//            set_dm8009_MIT(CAN_2, JOINT_LB_SEND, 1.5707963, 0, 20, 5, 0);
//            osDelay(2);
//            set_dm8009_MIT(CAN_2, JOINT_RF_SEND, 1.5707963, 0, 20, 5, 0);
//            set_dm8009_MIT(CAN_2, JOINT_RB_SEND, -1.5707963, 0, 20, 5, 0);
//        }
        chassis.leg_L.joint_F_torque = 0.0f;
        chassis.leg_L.joint_B_torque = 0.0f;
        chassis.leg_R.joint_F_torque = 0.0f;
        chassis.leg_R.joint_B_torque = 0.0f;

        chassis.is_chassis_balance = false;
    }
    else{
        chassis.is_chassis_balance = true;
    }



//    if(ABS(chassis.imu_reference.pitch_angle) > 0.1395f) // -8��~ 8��
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


/************************* ʧ������ ***************************/
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


    /** ��ʼ����־λ **/
    // ���̳�ʼ����־λ
    chassis.init_flag = false;

    // ƽ���־λ
    chassis.is_chassis_balance = false;
    // �����Ծȳɹ���־λ
    chassis.recover_finish = false;

    // ��ر�־λ
    chassis.leg_L.leg_is_offground = false;
    chassis.leg_R.leg_is_offground = false;
    chassis.chassis_is_offground   = false;

    // ��Ծ��־λ
    chassis.jump_flag = false;

    chassis.power_is_dangerous=false;

    /**
     * ��ΪLK����ϵ�Ĭ��ʹ�ܣ�֮ǰ�й�ң����ʧ�ܺ���챵����Ȼ��ת������
     * ���ѡ����INITģʽ��ʹ�ܣ�����ʧ�ܺ�ֹͣ��챵��������ʱ����Կ��Խ�����Ϣ����������
     * **/
    wheel_stop();
}

/*********************** ��ʼ������ ***************************/
static void chassis_init_task() {

    joint_enable();
    wheel_enable();

//
//    if(is_joints_reduced())
//    {
        chassis.init_flag = true;
//    }
//    else
//    {
//        chassis.init_flag=false;
//        chassis.leg_L.wheel_torque = 0;
//        chassis.leg_R.wheel_torque = 0;
//
//        set_dm8009_MIT(CAN_2, JOINT_LF_SEND, -1.5707963, 0, 20, 5, 0);
//        set_dm8009_MIT(CAN_2, JOINT_LB_SEND, 1.5707963, 0, 20, 5, 0);
//        osDelay(2);
//        set_dm8009_MIT(CAN_2, JOINT_RF_SEND, 1.5707963, 0, 20, 5, 0);
//        set_dm8009_MIT(CAN_2, JOINT_RB_SEND, -1.5707963, 0, 20, 5, 0);
//    }

}

/*********************** ʹ������ ****************************/
static void chassis_enable_task() {

    lqr_ctrl();
    vmc_ctrl();
    chassis_vx_kalman_run();

    chassis_selfhelp();

}

/*********************** ������ ******************************/
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
