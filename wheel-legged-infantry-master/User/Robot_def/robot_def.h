#ifndef ROBOT_DEF_H
#define ROBOT_DEF_H

#include <stdbool.h>

#include "pid.h"
#include "moving_filter.h"

typedef float fp32; // ����ĳ��float���ͱ�����32λ������
typedef double fp64;

/** �궨�� **/
//CHASSIS_REMOTE��1��������ң��������
#define CHASSIS_REMOTE 1

//DEBUG_MODE��1���������ģʽ���رչؽں�������
#define DEBUG_MODE 0

#define CHASSIS_PERIOD 2 // ms ����Ƶ��: 500Hz

/** ң����·�� **/
// x : 2-���� ; 0-����
// y : 3-���� ; 1-����
#define CHASSIS_SPEED_CHANNEL 1
#define CHASSIS_YAW_CHANNEL 2
#define TEST_CHASSIS_ROLL_CHANNEL 2
#define TEST_CHASSIS_LEG_CHANNEL 3
#define CHASSIS_PIT_CHANNEL 3
#define CHASSIS_ROLL_CHANNEL 0

/** ����Լ�� **/
#define MAX_CHASSIS_VX_SPEED 2.0f
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


/** ң����ֵӳ�� **/
#define RC_TO_VX  (MAX_CHASSIS_VX_SPEED/660)
#define MAX_CHASSIS_YAW_INCREMENT 0.02f
#define RC_TO_YAW_INCREMENT (MAX_CHASSIS_YAW_INCREMENT/660)
#define RC_TO_PITCH ((MAX_PITCH-MIN_PITCH)/660)
#define RC_TO_ROLL ((MAX_ROLL-MIN_ROLL)/660)

/** PID���� **/
/** PID���� **/
/** ת��PID **/
#define CHASSIS_TURN_PID_P 20.0f
#define CHASSIS_TURN_PID_I 0.0f
#define CHASSIS_TURN_PID_D 3.0f
#define CHASSIS_TURN_PID_IOUT_LIMIT 0.0f
#define CHASSIS_TURN_PID_OUT_LIMIT 4.0f

/** 腿长位置环PID **/
#define CHASSIS_LEG_L0_POS_PID_P 15.0f
#define CHASSIS_LEG_L0_POS_PID_I 0.0f
#define CHASSIS_LEG_L0_POS_PID_D 15.0f
#define CHASSIS_LEG_L0_POS_PID_IOUT_LIMIT 0.0f
#define CHASSIS_LEG_L0_POS_PID_OUT_LIMIT 2.0f

/** 腿长速度环PID **/
#define CHASSIS_LEG_L0_SPEED_PID_P 30.0f // 50.0f
#define CHASSIS_LEG_L0_SPEED_PID_I 0.0f
#define CHASSIS_LEG_L0_SPEED_PID_D 0.0f
#define CHASSIS_LEG_L0_SPEED_PID_IOUT_LIMIT 0.0f
#define CHASSIS_LEG_L0_SPEED_PID_OUT_LIMIT 60.0f

/** Roll PID **/
#define CHASSIS_ROLL_PID_P 40.0f
#define CHASSIS_ROLL_PID_I 0.0f
#define CHASSIS_ROLL_PID_D 0.0f
#define CHASSIS_ROLL_PID_IOUT_LIMIT 0.0f
#define CHASSIS_ROLL_PID_OUT_LIMIT 20.0f

/** 离地后的腿长PID **/
#define CHASSIS_OFFGROUND_LO_PID_P 0.0f
#define CHASSIS_OFFGROUND_L0_PID_I 0.0f
#define CHASSIS_OFFGROUND_L0_PID_D 0.0f
#define CHASSIS_OFFGROUND_L0_PID_IOUT_LIMIT 0.0f
#define CHASSIS_OFFGROUND_L0_PID_OUT_LIMIT 0.0f

/** 防劈叉PID **/
#define CHASSIS_LEG_COORDINATION_PID_P 15.0f
#define CHASSIS_LEG_COORDINATION_PID_I 0.0f
#define CHASSIS_LEG_COORDINATION_PID_D 2.0f
#define CHASSIS_LEG_COORDINATION_PID_IOUT_LIMIT 0.0f
#define CHASSIS_LEG_COORDINATION_PID_OUT_LIMIT 10.0f


/*******************************************************************************
 *                                    ����                                     *
 *******************************************************************************/

/** ������������ṹ�� **/
typedef struct{
    float wheel_radius; // �����ְ뾶
    float body_weight; // ��������(����̨Ҫ������̨)
    float wheel_weight; // ����������(���ϵ��)

    float l1, l2, l3, l4, l5; // �����˲���
} ChassisPhysicalConfig;

/** ����ģʽ�ṹ�� **/
//typedef enum{
//    CHASSIS_DISABLE = 1, // ʧ��ģʽ
//    CHASSIS_INIT, // ��ʼ��ģʽ
//    CHASSIS_ENABLE, // ʹ��ģʽ
//    CHASSIS_JUMP, // ��Ծģʽ
//    CHASSIS_SPIN, // С����
//} ChassisCtrlMode;

typedef enum{
    CHASSIS_DISABLE = 1, // ʧ��ģʽ
    CHASSIS_ENABLE, // ʹ��ģʽ
    CHASSIS_SPIN, // С����
    CHASSIS_INIT, // ��ʼ��ģʽ
    CHASSIS_JUMP, // ��Ծģʽ
} ChassisCtrlMode;

typedef enum{
    GIMBAL_RELAX=0,//云台失能
    GIMBAL_BACK,//云台回中
    GIMBAL_ACTIVE,
    GIMBAL_AUTO,
    GIMBAL_BUFF, //大符
    GIMBAL_SBUFF, //小符
    GIMBAL_FORECAST,//预测
} GimbalCtrlMode;

typedef struct {
    int16_t fire_l_speed_rpm;
    int16_t fire_r_speed_rpm;
} LauncherCtrlInfo;

/** ��Ծ״̬�ṹ�� **/
typedef enum{
    NOT_READY,
    READY, // ��һ�׶Σ���������
    STRETCHING, // �ڶ��׶Σ����ȵŵ�
    SHRINKING, // �����׶Σ���������
    LANDING, // ���Ľ׶Σ����
} JumpState;

/** �������ṹ�� **/
typedef struct{
    // ŷ����
    float roll_angle;
    float pitch_angle;
    float yaw_angle;
    float yaw_last_angle;
    float yaw_total_angle;
    float yaw_round_count;


    //������ٶ�
    float pitch_gyro;
    float yaw_gyro;
    float roll_gyro;

    //������ٶ�
    float ax;
    float ay;
    float az;

    //ȥ���������ٶ�Ӱ����������ٶ�
    float ax_filtered;
    float ay_filtered;
    float az_filtered;

    // ������ֱ���ϵļ��ٶ�
    float robot_az;

} IMUReference;

typedef struct{
    float v_m_per_s; // �����ٶ�
    float x; // ����λ��
    float pitch_angle_rad;
    float yaw_angle_rad;
    float roll_angle_rad;
    float height_m; // �����ȳ�
    float spin_speed;

} ChassisCtrlInfo;

/** ״̬�����ṹ�� **/
typedef struct{
    float theta; // ״̬����1
    float theta_dot; // ״̬����2
    float theta_last;
    float theta_dot_last;
    float theta_ddot;

    float x; // ״̬����3
    float x_dot; // ״̬����4
    float x_dot_last;
    float x_ddot;

    float phi; // ״̬����5
    float phi_dot; // ״̬����6
} StateVariable;

/** VMC VMC VMC VMC VMC VMC VMC VMC VMC VMC VMC VMC VMC VMC VMC VMC VMC VMC **/

/** ���˶�ѧ����  FK == Forward Kinematics(���˶�ѧ) **/
typedef struct{// �ȳ�
    float L0;
    float L0_last;
    float L0_dot;
    float L0_dot_last;
    float L0_ddot;
} FKL0;

typedef struct{// �������еĽǶ�
    float phi1;
    float phi2;
    float phi3;
    float phi4;

    float phi0; // �Ȱڽ�
    float last_phi0;
    float d_phi0;// �ڽǱ仯�ٶ�
    float last_d_phi0;
    float dd_phi0;
} FKPhi;

typedef struct{// �������еĵ�����(Coordinates)
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

/** ������ѧ����(Forward Dynamics)���� ĩ����(F Tp) �� ĩ��ִ����(T1 T4) **/
    union { // ����ѧϰ�����������: union
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


/** �涯��ѧ����(Inverse Dynamics): �� ĩ��ִ����(T1 T4) �� ĩ����(T Tp) **/
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

 /** ���˶�ѧ����(Inverse Dynamics): �� ĩ��ִ����(w1 w4) �� ĩ�˱���(d_L0 d_phi0) **/
    union {
        float array[2][1];
        struct {
            float w1_fdb;// �ؽڵ�����������Ľ��ٶ�
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
            float d_L0_fdb; // �ȳ��仯�ٶ�
            float d_phi0_fdb; // �ڽ�(phi0)�仯�ٶ�
        } E;
    } V_fdb;

}InverseKinematics;

/** �Ȳ�VMC�ṹ�� **/
typedef struct{
    ForwardKinematics forward_kinematics;
    InverseKinematics inverse_kinematics;
} VMC;
/*****************************************************************************/



/** �Ȳ��ṹ�� **/
typedef struct{

    ChassisCtrlInfo chassis_ctrl_info;

    /** ״̬���� **/
    StateVariable state_variable_feedback;  // ����״̬����
    StateVariable state_variable_set_point; // ����״̬����
    StateVariable state_variable_error;     // ��� = ���� - ����
    StateVariable state_variable_wheel_out; // ����״̬����ͨ��lqr����Ĺ�����챵����
    StateVariable state_variable_joint_out; // ����״̬����ͨ��lqr����Ĺ��ڹؽڵ����

    /** �Ȳ�VMC **/
    VMC vmc;

    /** �ȳ�����PID **/
    Pid leg_pos_pid; // �ȳ�λ�û�
    Pid leg_speed_pid; // �ȳ��ٶȻ�
    float leg_offset; // ����Խ����̡�

    /** ��غ���ȳ�PID **/
    Pid offground_leg_pid; // ��غ���ȳ�pid  ʹ�Ⱦ����ӽ����棬���ӻ���

    float wheel_torque; // �������
    float joint_F_torque; // �ؽ�����
    float joint_B_torque;


    /** ��ֱ����֧���� **/
    MovingAverageFilter theta_ddot_filter; // dd_theta���ƶ�ƽ���˲���, ���ڼ�����ֱ����֧����Fn
    MovingAverageFilter Fn_filter; // ��ֱ����֧����Fn���ƶ�ƽ���˲���
    float Fn; // ��ֱ����֧����

    bool leg_is_offground;

} Leg;

/** ���̽ṹ�� **/
typedef struct{

    /** ������ **/
    IMUReference imu_reference;
    MovingAverageFilter robot_az_filter;

    /** ң������Ϣ **/
    ChassisCtrlMode chassis_ctrl_mode;
    ChassisCtrlMode chassis_ctrl_mode_last;
    ChassisCtrlInfo chassis_ctrl_info;

    /** �Ȳ� **/
    Leg leg_L;
    Leg leg_R;

    /** ��Ծ **/
    JumpState jump_state;

    /** PID **/
    Pid chassis_turn_pid;             // ת��pid
    Pid chassis_leg_coordination_pid; // ������pid
    Pid chassis_roll_pid;             // roll����pid
    float wheel_turn_torque;          // ת������
    float steer_compensatory_torque;  // ����������
    float theta_error;                // ������֮��theta�����
    Pid chassis_vw_speed_pid;
    Pid chassis_vw_pos_pid;
    Pid chassis_spin_pid;


    /** flag **/
    bool init_flag;            // ���̳�ʼ����ɱ�־λ

    bool is_chassis_balance;   // ƽ���־λ
    bool recover_finish;       // ����������ɱ�־λ

    bool chassis_is_offground; // ��ر�־λ
    bool jump_flag;            // ��Ծ��־λ
    bool power_is_dangerous;

    float chassis_power_K;
    float last_chassis_power_K;
    float power_nihe;
    float power_nihe2;

} Chassis;



/*******************************************************************************
 *                                    ��̨                                     *
 *******************************************************************************/


////��̨����ķ�����
//typedef struct
//{
//    uint16_t ecd; // ������ֵ
//    int16_t last_ecd;
//    int16_t speed_rpm; // ����������ת��
//    int16_t feedback_current; // ʩ���ڵ���ϵĵ���
//    uint8_t temperate; // �¶�
//
//    int32_t round_cnt;   //�����ת����Ȧ��
//    int32_t total_ecd;   //�����ת���ܱ�����ֵ
//
//    uint16_t offset_ecd;//�����У׼����ֵ
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
//    pid_t angle_pos_pid; // ��̨�Ƕ�λ�û�pid
//    pid_t angle_speed_pid; // ��̨�Ƕ��ٶȻ�pid
//
//    fp32 relative_angle_get;
//    fp32 relative_angle_set; //��
//
//    fp32 absolute_angle_get;
//    fp32 absolute_angle_set;//rad
//
//    fp32 gyro_set;  //ת������
//    int16_t give_current; //���յ���ֵ
//
//}motor_6020_t;
//
//
//typedef struct
//{
//    motor_measure_t *motor_measure;
//
//    pid_t angle_p;//�ǶȻ�pid
//
//    pid_t speed_p;//�ٶȻ�pid
//
//    fp32 speed;//ת������ֵ
//
//    int16_t give_current;
//
//}motor_2006_t;



#endif
