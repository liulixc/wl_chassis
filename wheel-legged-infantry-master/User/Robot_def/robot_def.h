#ifndef ROBOT_DEF_H
#define ROBOT_DEF_H

#include <stdbool.h>

#include "pid.h"
#include "moving_filter.h"

typedef float fp32; // 表明某个float类型变量是32位浮点数
typedef double fp64;

/** 宏定义 **/
//CHASSIS_REMOTE置1：底盘由遥控器控制
#define CHASSIS_REMOTE 0

//DEBUG_MODE置1：进入调试模式，关闭关节和轮毂输出
#define DEBUG_MODE 0

#define CHASSIS_PERIOD 2 // ms 控制频率: 500Hz

/** 遥控器路径 **/
// x : 2-左手 ; 0-右手
// y : 3-左手 ; 1-右手
#define CHASSIS_SPEED_CHANNEL 1
#define CHASSIS_YAW_CHANNEL 2
#define TEST_CHASSIS_ROLL_CHANNEL 2
#define TEST_CHASSIS_LEG_CHANNEL 3
#define CHASSIS_PIT_CHANNEL 3
#define CHASSIS_ROLL_CHANNEL 0

/** 变量约束 **/
#define MAX_CHASSIS_VX_SPEED 1.0f
#define MAX_PITCH 0.174533f
#define MIN_PITCH (-0.174533f)
#define MAX_ROLL 0.12f
#define MIN_ROLL (-0.12f)
#define MAX_WHEEL_TORQUE 10.f
#define MIN_WHEEL_TORQUE (-10.f)
#define MAX_JOINT_TORQUE 40.f
#define MIN_JOINT_TORQUE (-40.f)

#define MIN_L0 0.10f
#define MAX_L0 0.40f


/** 遥控器值映射 **/
#define RC_TO_VX  (MAX_CHASSIS_VX_SPEED/660)
#define MAX_CHASSIS_YAW_INCREMENT 0.02f
#define RC_TO_YAW_INCREMENT (MAX_CHASSIS_YAW_INCREMENT/660)
#define RC_TO_PITCH ((MAX_PITCH-MIN_PITCH)/660)
#define RC_TO_ROLL ((MAX_ROLL-MIN_ROLL)/660)

/** PID参数 **/
/** 转向PID **/
#define CHASSIS_TURN_PID_P 20.0f
#define CHASSIS_TURN_PID_I 0.0f
#define CHASSIS_TURN_PID_D 3.0f
#define CHASSIS_TURN_PID_IOUT_LIMIT 0.0f
#define CHASSIS_TURN_PID_OUT_LIMIT 100.0f

///** 腿长位置环PID **/
//#define CHASSIS_LEG_L0_POS_PID_P 4.0f
//#define CHASSIS_LEG_L0_POS_PID_I 0.0f
//#define CHASSIS_LEG_L0_POS_PID_D 1.0f
//#define CHASSIS_LEG_L0_POS_PID_IOUT_LIMIT 0.0f
//#define CHASSIS_LEG_L0_POS_PID_OUT_LIMIT 20.0f
//
///** 腿长速度环PID **/
//#define CHASSIS_LEG_L0_SPEED_PID_P 70.0f // 50.0f
//#define CHASSIS_LEG_L0_SPEED_PID_I 5.0f
//#define CHASSIS_LEG_L0_SPEED_PID_D 0.0f
//#define CHASSIS_LEG_L0_SPEED_PID_IOUT_LIMIT 500.0f
//#define CHASSIS_LEG_L0_SPEED_PID_OUT_LIMIT 1000.0f

/** 腿长位置环PID **/
#define CHASSIS_LEG_L0_POS_PID_P 15.0f
#define CHASSIS_LEG_L0_POS_PID_I 0.0f
#define CHASSIS_LEG_L0_POS_PID_D 50.0f
#define CHASSIS_LEG_L0_POS_PID_IOUT_LIMIT 0.0f
#define CHASSIS_LEG_L0_POS_PID_OUT_LIMIT 20.0f

/** 腿长速度环PID **/
#define CHASSIS_LEG_L0_SPEED_PID_P 50.0f // 50.0f
#define CHASSIS_LEG_L0_SPEED_PID_I 0.0f
#define CHASSIS_LEG_L0_SPEED_PID_D 0.0f
#define CHASSIS_LEG_L0_SPEED_PID_IOUT_LIMIT 0.0f
#define CHASSIS_LEG_L0_SPEED_PID_OUT_LIMIT 1000.0f

/** Roll PID **/
#define CHASSIS_ROLL_PID_P 150.0f
#define CHASSIS_ROLL_PID_I 0.0f
#define CHASSIS_ROLL_PID_D 0.0f
#define CHASSIS_ROLL_PID_IOUT_LIMIT 0.0f
#define CHASSIS_ROLL_PID_OUT_LIMIT 2000.0f

/** 离地后的腿长PID **/
#define CHASSIS_OFFGROUND_LO_PID_P 0.0f
#define CHASSIS_OFFGROUND_L0_PID_I 0.0f
#define CHASSIS_OFFGROUND_L0_PID_D 0.0f
#define CHASSIS_OFFGROUND_L0_PID_IOUT_LIMIT 0.0f
#define CHASSIS_OFFGROUND_L0_PID_OUT_LIMIT 0.0f

/** 防劈叉PID **/
#define CHASSIS_LEG_COORDINATION_PID_P 110.0f
#define CHASSIS_LEG_COORDINATION_PID_I 0.0f
#define CHASSIS_LEG_COORDINATION_PID_D 20.0f
#define CHASSIS_LEG_COORDINATION_PID_IOUT_LIMIT 0.0f
#define CHASSIS_LEG_COORDINATION_PID_OUT_LIMIT 50.0f


/*******************************************************************************
 *                                    底盘                                     *
 *******************************************************************************/

/** 底盘物理参数结构体 **/
typedef struct{
    float wheel_radius; // 驱动轮半径
    float body_weight; // 机体质量(有云台要算上云台)
    float wheel_weight; // 驱动轮重量(算上电机)

    float l1, l2, l3, l4, l5; // 五连杆参数
} ChassisPhysicalConfig;

/** 底盘模式结构体 **/
//typedef enum{
//    CHASSIS_DISABLE = 1, // 失能模式
//    CHASSIS_INIT, // 初始化模式
//    CHASSIS_ENABLE, // 使能模式
//    CHASSIS_JUMP, // 跳跃模式
//    CHASSIS_SPIN, // 小陀螺
//} ChassisCtrlMode;

typedef enum{
    CHASSIS_DISABLE = 1, // 失能模式
    CHASSIS_ENABLE, // 使能模式
    CHASSIS_SPIN, // 小陀螺
    CHASSIS_INIT, // 初始化模式
    CHASSIS_JUMP, // 跳跃模式
} ChassisCtrlMode;

/** 跳跃状态结构体 **/
typedef enum{
    NOT_READY,
    READY, // 第一阶段：收腿蓄力
    STRETCHING, // 第二阶段：伸腿蹬地
    SHRINKING, // 第三阶段：空中收腿
    LANDING, // 第四阶段：落地
} JumpState;

/** 传感器结构体 **/
typedef struct{
    // 欧拉角
    float roll_angle;
    float pitch_angle;
    float yaw_angle;
    float yaw_last_angle;
    float yaw_total_angle;
    float yaw_round_count;


    //三轴角速度
    float pitch_gyro;
    float yaw_gyro;
    float roll_gyro;

    //三轴加速度
    float ax;
    float ay;
    float az;

    //去除重力加速度影响后的三轴加速度
    float ax_filtered;
    float ay_filtered;
    float az_filtered;

    // 机体竖直向上的加速度
    float robot_az;

} IMUReference;

typedef struct{
    float v_m_per_s; // 期望速度
    float x; // 期望位移
    float pitch_angle_rad;
    float yaw_angle_rad;
    float roll_angle_rad;
    float height_m; // 期望腿长
    float spin_speed;

} ChassisCtrlInfo;

/** 状态变量结构体 **/
typedef struct{
    float theta; // 状态变量1
    float theta_dot; // 状态变量2
    float theta_last;
    float theta_dot_last;
    float theta_ddot;

    float x; // 状态变量3
    float x_dot; // 状态变量4
    float x_dot_last;
    float x_ddot;

    float phi; // 状态变量5
    float phi_dot; // 状态变量6
} StateVariable;

/** VMC VMC VMC VMC VMC VMC VMC VMC VMC VMC VMC VMC VMC VMC VMC VMC VMC VMC **/

/** 正运动学解算  FK == Forward Kinematics(正运动学) **/
typedef struct{// 腿长
    float L0;
    float L0_last;
    float L0_dot;
    float L0_dot_last;
    float L0_ddot;
} FKL0;

typedef struct{// 五连杆中的角度
    float phi1;
    float phi2;
    float phi3;
    float phi4;

    float phi0; // 腿摆角
    float last_phi0;
    float d_phi0;// 摆角变化速度
    float last_d_phi0;
    float dd_phi0;
} FKPhi;

typedef struct{// 五连杆中的点坐标(Coordinates)
    float a_x, a_y;
    float b_x, b_y;
    float c_x, c_y;
    float d_x, d_y;
    float e_x, e_y;
} FKPointCoordinates;

typedef struct{
    FKL0 fk_L0;
    FKPhi fk_phi;
    FKPointCoordinates fk_point_coordinates;
    float d_alpha; // ?

/** 正动力学解算(Forward Dynamics)：从 末端力(F Tp) 到 末端执行器(T1 T4) **/
    union { // 自行学习联合体的特性: union
        float array[2][2];
        struct {
            float Tp_set_point;
            float Fy_set_point;
        } E;
    } Fxy_set_point;

    union {
        float array[2][2];
        struct {
            float x1_1;
            float x1_2;
            float x2_1;
            float x2_2;
        } E;
    } J_F_to_T;

    union {
        float array[2][1];
        struct {
            float T1_set_point;
            float T4_set_point;
        } E;
    } T1_T4_set_point;

} ForwardKinematics;


/** 逆动力学解算(Inverse Dynamics): 从 末端执行器(T1 T4) 到 末端力(T Tp) **/
typedef struct {
    union {
        float array[2][1];
        struct {
            float T1_fdb;
            float T4_fdb;
        } E;
    } T1_T4_fdb;

    union {
        float array[2][2];
        struct {
            float x1_1;
            float x1_2;
            float x2_1;
            float x2_2;
        } E;
    } J_T_to_F;

    union {
        float array[2][1];
        struct {
            float Tp_fdb;
            float Fy_fdb;
        } E;
    } Fxy_fdb;

 /** 逆运动学解算(Inverse Dynamics): 从 末端执行器(w1 w4) 到 末端编码(d_L0 d_phi0) **/
    union {
        float array[2][1];
        struct {
            float w1_fdb;// 关节电机反馈回来的角速度
            float w4_fdb;
        } E;
    } W_fdb;

    union {
        float array[2][2];
        struct {
            float x1_1;
            float x1_2;
            float x2_1;
            float x2_2;
        } E;
    } J_w_to_v;

    union {
        float array[2][1];
        struct {
            float d_L0_fdb; // 腿长变化速度
            float d_phi0_fdb; // 摆角(phi0)变化速度
        } E;
    } V_fdb;

}InverseKinematics;

/** 腿部VMC结构体 **/
typedef struct{
    ForwardKinematics forward_kinematics;
    InverseKinematics inverse_kinematics;
} VMC;
/*****************************************************************************/



/** 腿部结构体 **/
typedef struct{

    ChassisCtrlInfo chassis_ctrl_info;

    /** 状态变量 **/
    StateVariable state_variable_feedback;  // 反馈状态变量
    StateVariable state_variable_set_point; // 期望状态变量
    StateVariable state_variable_error;     // 误差 = 反馈 - 期望
    StateVariable state_variable_wheel_out; // 各个状态变量通过lqr计算的关于轮毂的输出
    StateVariable state_variable_joint_out; // 各个状态变量通过lqr计算的关于关节的输出

    /** 腿部VMC **/
    VMC vmc;

    /** 腿长串级PID **/
    Pid leg_pos_pid; // 腿长位置环
    Pid leg_speed_pid; // 腿长速度环
    float leg_offset; // “面对结果编程”

    /** 离地后的腿长PID **/
    Pid offground_leg_pid; // 离地后的腿长pid  使腿尽量接近地面，增加缓冲

    float wheel_torque; // 轮毂力矩
    float joint_F_torque; // 关节力矩
    float joint_B_torque;


    /** 竖直方向支持力 **/
    MovingAverageFilter theta_ddot_filter; // dd_theta的移动平均滤波器, 用于计算竖直方向支持力Fn
    MovingAverageFilter Fn_filter; // 竖直方向支持力Fn的移动平均滤波器
    float Fn; // 竖直方向支持力

    bool leg_is_offground;

} Leg;

/** 底盘结构体 **/
typedef struct{

    /** 传感器 **/
    IMUReference imu_reference;
    MovingAverageFilter robot_az_filter;

    /** 遥控器信息 **/
    ChassisCtrlMode chassis_ctrl_mode;
    ChassisCtrlMode chassis_ctrl_mode_last;
    ChassisCtrlInfo chassis_ctrl_info;

    /** 腿部 **/
    Leg leg_L;
    Leg leg_R;

    /** 跳跃 **/
    JumpState jump_state;

    /** PID **/
    Pid chassis_turn_pid;             // 转向pid
    Pid chassis_leg_coordination_pid; // 防劈叉pid
    Pid chassis_roll_pid;             // roll补偿pid
    float wheel_turn_torque;          // 转向力矩
    float steer_compensatory_torque;  // 防劈叉力矩
    float theta_error;                // 两条腿之间theta的误差
    Pid chassis_vw_speed_pid;
    Pid chassis_vw_pos_pid;
    Pid chassis_spin_pid;


    /** flag **/
    bool init_flag;            // 底盘初始化完成标志位

    bool is_chassis_balance;   // 平衡标志位
    bool recover_finish;       // 倒地自起完成标志位

    bool chassis_is_offground; // 离地标志位
    bool jump_flag;            // 跳跃标志位

} Chassis;



/*******************************************************************************
 *                                    云台                                     *
 *******************************************************************************/


////云台电机的反馈量
//typedef struct
//{
//    uint16_t ecd; // 编码器值
//    int16_t last_ecd;
//    int16_t speed_rpm; // 反馈回来的转速
//    int16_t feedback_current; // 施加在电机上的电流
//    uint8_t temperate; // 温度
//
//    int32_t round_cnt;   //电机旋转的总圈数
//    int32_t total_ecd;   //电机旋转的总编码器值
//
//    uint16_t offset_ecd;//电机的校准编码值
//} motor_measure_t;
//
//typedef struct
//{
//    const motor_measure_t *motor_measure;
//
//    fp32 speed;
//
//    fp32 rpm_set;
//
//    pid_t speed_p;
//
//    int16_t give_current;
//
//
//}motor_3508_t;
//
//typedef struct
//{
//    motor_measure_t *motor_measure;
//
//    pid_t angle_pos_pid; // 云台角度位置环pid
//    pid_t angle_speed_pid; // 云台角度速度环pid
//
//    fp32 relative_angle_get;
//    fp32 relative_angle_set; //°
//
//    fp32 absolute_angle_get;
//    fp32 absolute_angle_set;//rad
//
//    fp32 gyro_set;  //转速设置
//    int16_t give_current; //最终电流值
//
//}motor_6020_t;
//
//
//typedef struct
//{
//    motor_measure_t *motor_measure;
//
//    pid_t angle_p;//角度环pid
//
//    pid_t speed_p;//速度环pid
//
//    fp32 speed;//转速期望值
//
//    int16_t give_current;
//
//}motor_2006_t;



#endif
