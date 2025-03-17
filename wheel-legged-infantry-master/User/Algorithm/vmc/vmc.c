#include <stdbool.h>
#include <math.h>

#include "robot_def.h"
#include "vmc.h"
#include "user_lib.h"
#include "joint.h"
#include "moving_filter.h"
#include "Referee.h"
#include "arm_math.h"
#include "math.h"

extern float motor_torque_l,motor_w_l;
extern float motor_torque_r,motor_w_r;
extern Referee_info_t Referee;
extern Chassis chassis;
extern ChassisPhysicalConfig chassis_physical_config;

float wheel_L_balance_torque,wheel_R_balance_torque;
float wheel_L_move_torque,wheel_R_move_torque;

float test_delta,test_root1,test_root2;

float L_L0_dot_test, R_L0_dot_test;

/*               ������
 *    phi4                      phi4
 *
 *    phi1                      phi1
 */


static void vmc_phi_update(Leg *leg_L, Leg *leg_R) {

    float RF_joint_pos = (get_joint_motors() + 3)->pos_r;
    float RB_joint_pos = (get_joint_motors() + 2)->pos_r;
    float LF_joint_pos = (get_joint_motors() + 0)->pos_r;
    float LB_joint_pos = get_joint_motors()[1].pos_r;

    leg_L->vmc.forward_kinematics.fk_phi.phi4 = PI/2 + LF_joint_pos;
    leg_L->vmc.forward_kinematics.fk_phi.phi1 = PI/2 + LB_joint_pos;
    leg_R->vmc.forward_kinematics.fk_phi.phi4 = PI/2 - RF_joint_pos;
    leg_R->vmc.forward_kinematics.fk_phi.phi1 = PI/2 - RB_joint_pos;
}

// vmc���˶�ѧ����
static void forward_kinematics(Leg* leg_L, Leg* leg_R, ChassisPhysicalConfig *physical_config) {
    /***LEG_L LEG_L LEG_L LEG_L LEG_L LEG_L LEG_L LEG_L LEG_L LEG_L LEG_L LEG_L LEG_L LEG_L LEG_L LEG_L LEG_L LEG_L LEG_L LEG_L***/

    leg_L->vmc.forward_kinematics.fk_point_coordinates.b_x = physical_config->l1 * cosf(leg_L->vmc.forward_kinematics.fk_phi.phi1);
    leg_L->vmc.forward_kinematics.fk_point_coordinates.b_y = physical_config->l1 * sinf(leg_L->vmc.forward_kinematics.fk_phi.phi1);
    leg_L->vmc.forward_kinematics.fk_point_coordinates.d_x = physical_config->l5 + physical_config->l4 * cosf(leg_L->vmc.forward_kinematics.fk_phi.phi4);
    leg_L->vmc.forward_kinematics.fk_point_coordinates.d_y = physical_config->l4 * sinf(leg_L->vmc.forward_kinematics.fk_phi.phi4); // l5 �ĳ��� l4

    float L_A0 = 2.0f * physical_config->l2 * (leg_L->vmc.forward_kinematics.fk_point_coordinates.d_x - leg_L->vmc.forward_kinematics.fk_point_coordinates.b_x);
    float L_B0 = 2.0f * physical_config->l2 * (leg_L->vmc.forward_kinematics.fk_point_coordinates.d_y - leg_L->vmc.forward_kinematics.fk_point_coordinates.b_y);
    float L_BD_sq =  (leg_L->vmc.forward_kinematics.fk_point_coordinates.d_x - leg_L->vmc.forward_kinematics.fk_point_coordinates.b_x) * (leg_L->vmc.forward_kinematics.fk_point_coordinates.d_x - leg_L->vmc.forward_kinematics.fk_point_coordinates.b_x)
                     + (leg_L->vmc.forward_kinematics.fk_point_coordinates.d_y - leg_L->vmc.forward_kinematics.fk_point_coordinates.b_y) * (leg_L->vmc.forward_kinematics.fk_point_coordinates.d_y - leg_L->vmc.forward_kinematics.fk_point_coordinates.b_y);
    float L_C0 = physical_config->l2 * physical_config->l2 + L_BD_sq - physical_config->l3 * physical_config->l3;

    float temp = L_A0 * L_A0 + L_B0 * L_B0 - L_C0 * L_C0;
    float y = L_B0 + sqrtf(ABS(temp));
    float x = L_A0 + L_C0;
    leg_L->vmc.forward_kinematics.fk_phi.phi2 = 2.0f * atan2f(y, x);

    leg_L->vmc.forward_kinematics.fk_point_coordinates.c_x = physical_config->l1 * cosf(leg_L->vmc.forward_kinematics.fk_phi.phi1) + physical_config->l2 * cosf(leg_L->vmc.forward_kinematics.fk_phi.phi2);
    leg_L->vmc.forward_kinematics.fk_point_coordinates.c_y = physical_config->l1 * sinf(leg_L->vmc.forward_kinematics.fk_phi.phi1) + physical_config->l2 * sinf(leg_L->vmc.forward_kinematics.fk_phi.phi2);
    y = leg_L->vmc.forward_kinematics.fk_point_coordinates.c_y - leg_L->vmc.forward_kinematics.fk_point_coordinates.d_y;
    x = leg_L->vmc.forward_kinematics.fk_point_coordinates.c_x - leg_L->vmc.forward_kinematics.fk_point_coordinates.d_x;
    leg_L->vmc.forward_kinematics.fk_phi.phi3 = atan2f(y, x);

    temp =  (leg_L->vmc.forward_kinematics.fk_point_coordinates.c_x - physical_config->l5 * 0.5f) * (leg_L->vmc.forward_kinematics.fk_point_coordinates.c_x - physical_config->l5 * 0.5f)
            + leg_L->vmc.forward_kinematics.fk_point_coordinates.c_y * leg_L->vmc.forward_kinematics.fk_point_coordinates.c_y;
    leg_L->vmc.forward_kinematics.fk_L0.L0_last = leg_L->vmc.forward_kinematics.fk_L0.L0;
    leg_L->vmc.forward_kinematics.fk_L0.L0 = sqrtf(ABS(temp));
    leg_L->vmc.forward_kinematics.fk_L0.L0_dot_last = leg_L->vmc.forward_kinematics.fk_L0.L0_dot;
    leg_L->vmc.forward_kinematics.fk_L0.L0_dot =
            (leg_L->vmc.forward_kinematics.fk_L0.L0 - leg_L->vmc.forward_kinematics.fk_L0.L0_last)
            / (CHASSIS_PERIOD * 0.001f);
    leg_L->vmc.forward_kinematics.fk_L0.L0_ddot =
            (leg_L->vmc.forward_kinematics.fk_L0.L0_dot - leg_L->vmc.forward_kinematics.fk_L0.L0_dot_last)
            / (CHASSIS_PERIOD * 0.001f);

    leg_L->vmc.forward_kinematics.fk_phi.last_phi0 = leg_L->vmc.forward_kinematics.fk_phi.phi0;
    y = leg_L->vmc.forward_kinematics.fk_point_coordinates.c_y;
    x = leg_L->vmc.forward_kinematics.fk_point_coordinates.c_x - physical_config->l5 * 0.5f;
    leg_L->vmc.forward_kinematics.fk_phi.phi0 = atan2f(y, x);
    leg_L->vmc.forward_kinematics.fk_phi.last_d_phi0 = leg_L->vmc.forward_kinematics.fk_phi.d_phi0;
    leg_L->vmc.forward_kinematics.fk_phi.d_phi0 =  (leg_L->vmc.forward_kinematics.fk_phi.phi0 - leg_L->vmc.forward_kinematics.fk_phi.last_phi0)
                                                   / (CHASSIS_PERIOD * 0.001f);
    leg_L->vmc.forward_kinematics.fk_phi.dd_phi0 =  (leg_L->vmc.forward_kinematics.fk_phi.d_phi0 - leg_L->vmc.forward_kinematics.fk_phi.last_d_phi0)
                                                    / (CHASSIS_PERIOD * 0.001f);

    leg_L->vmc.forward_kinematics.d_alpha = 0.0f - leg_L->vmc.forward_kinematics.fk_phi.d_phi0;

    /***LEG_R LEG_R LEG_R LEG_R LEG_R LEG_R LEG_R LEG_R LEG_R LEG_R LEG_R LEG_R LEG_R LEG_R LEG_R LEG_R LEG_R LEG_R LEG_R LEG_R***/

    leg_R->vmc.forward_kinematics.fk_point_coordinates.b_x = physical_config->l1 * cosf(leg_R->vmc.forward_kinematics.fk_phi.phi1);
    leg_R->vmc.forward_kinematics.fk_point_coordinates.b_y = physical_config->l1 * sinf(leg_R->vmc.forward_kinematics.fk_phi.phi1);
    leg_R->vmc.forward_kinematics.fk_point_coordinates.d_x = physical_config->l5 + physical_config->l4 * cosf(leg_R->vmc.forward_kinematics.fk_phi.phi4);
    leg_R->vmc.forward_kinematics.fk_point_coordinates.d_y = physical_config->l4 * sinf(leg_R->vmc.forward_kinematics.fk_phi.phi4);

    float R_A0 = 2.0f * physical_config->l2 * (leg_R->vmc.forward_kinematics.fk_point_coordinates.d_x - leg_R->vmc.forward_kinematics.fk_point_coordinates.b_x);
    float R_B0 = 2.0f * physical_config->l2 * (leg_R->vmc.forward_kinematics.fk_point_coordinates.d_y - leg_R->vmc.forward_kinematics.fk_point_coordinates.b_y);
    float R_BD_sq =  (leg_R->vmc.forward_kinematics.fk_point_coordinates.d_x - leg_R->vmc.forward_kinematics.fk_point_coordinates.b_x) * (leg_R->vmc.forward_kinematics.fk_point_coordinates.d_x - leg_R->vmc.forward_kinematics.fk_point_coordinates.b_x)
                     + (leg_R->vmc.forward_kinematics.fk_point_coordinates.d_y - leg_R->vmc.forward_kinematics.fk_point_coordinates.b_y) * (leg_R->vmc.forward_kinematics.fk_point_coordinates.d_y - leg_R->vmc.forward_kinematics.fk_point_coordinates.b_y);
    float R_C0 = physical_config->l2 * physical_config->l2 + R_BD_sq - physical_config->l3 * physical_config->l3;

    temp = R_A0 * R_A0 + R_B0 * R_B0 - R_C0 * R_C0;
    y = R_B0 + sqrtf(ABS(temp));
    x = R_A0 + R_C0;
    leg_R->vmc.forward_kinematics.fk_phi.phi2 = 2.0f * atan2f(y, x);

    leg_R->vmc.forward_kinematics.fk_point_coordinates.c_x = physical_config->l1 * cosf(leg_R->vmc.forward_kinematics.fk_phi.phi1) + physical_config->l2 * cosf(leg_R->vmc.forward_kinematics.fk_phi.phi2);
    leg_R->vmc.forward_kinematics.fk_point_coordinates.c_y = physical_config->l1 * sinf(leg_R->vmc.forward_kinematics.fk_phi.phi1) + physical_config->l2 * sinf(leg_R->vmc.forward_kinematics.fk_phi.phi2);
    y = leg_R->vmc.forward_kinematics.fk_point_coordinates.c_y - leg_R->vmc.forward_kinematics.fk_point_coordinates.d_y;
    x = leg_R->vmc.forward_kinematics.fk_point_coordinates.c_x - leg_R->vmc.forward_kinematics.fk_point_coordinates.d_x;
    leg_R->vmc.forward_kinematics.fk_phi.phi3 = atan2f(y, x);

    temp =  (leg_R->vmc.forward_kinematics.fk_point_coordinates.c_x - physical_config->l5 * 0.5f) * (leg_R->vmc.forward_kinematics.fk_point_coordinates.c_x - physical_config->l5 * 0.5f)
            + leg_R->vmc.forward_kinematics.fk_point_coordinates.c_y * leg_R->vmc.forward_kinematics.fk_point_coordinates.c_y;
    leg_R->vmc.forward_kinematics.fk_L0.L0_last = leg_R->vmc.forward_kinematics.fk_L0.L0;
    leg_R->vmc.forward_kinematics.fk_L0.L0 = sqrtf(ABS(temp));
    leg_R->vmc.forward_kinematics.fk_L0.L0_dot_last = leg_R->vmc.forward_kinematics.fk_L0.L0_dot;
    leg_R->vmc.forward_kinematics.fk_L0.L0_dot = (leg_R->vmc.forward_kinematics.fk_L0.L0 - leg_R->vmc.forward_kinematics.fk_L0.L0_last) / (CHASSIS_PERIOD * 0.001f);
    leg_R->vmc.forward_kinematics.fk_L0.L0_ddot = (leg_R->vmc.forward_kinematics.fk_L0.L0_dot - leg_R->vmc.forward_kinematics.fk_L0.L0_dot_last) / (CHASSIS_PERIOD * 0.001f);

    leg_R->vmc.forward_kinematics.fk_phi.last_phi0 = leg_R->vmc.forward_kinematics.fk_phi.phi0;
    y = leg_R->vmc.forward_kinematics.fk_point_coordinates.c_y;
    x = leg_R->vmc.forward_kinematics.fk_point_coordinates.c_x - physical_config->l5 * 0.5f;


    leg_R->vmc.forward_kinematics.fk_phi.phi0 = atan2f(y, x);
    leg_R->vmc.forward_kinematics.fk_phi.last_d_phi0 = leg_R->vmc.forward_kinematics.fk_phi.d_phi0;
    leg_R->vmc.forward_kinematics.fk_phi.d_phi0 =  (leg_R->vmc.forward_kinematics.fk_phi.phi0 - leg_R->vmc.forward_kinematics.fk_phi.last_phi0) / (CHASSIS_PERIOD * 0.001f);
    leg_R->vmc.forward_kinematics.fk_phi.dd_phi0 =  (leg_R->vmc.forward_kinematics.fk_phi.d_phi0 - leg_R->vmc.forward_kinematics.fk_phi.last_d_phi0) / (CHASSIS_PERIOD * 0.001f);

    leg_R->vmc.forward_kinematics.d_alpha = 0.0f - leg_R->vmc.forward_kinematics.fk_phi.d_phi0;



}

// vmc������ѧ����
static void forward_dynamics(VMC *vmc, const ChassisPhysicalConfig *physical_config) {
    if (vmc == NULL)
    {
        return;
    }

    vmc->forward_kinematics.J_F_to_T.E.x1_1 =
            physical_config->l1 * cosf(vmc->forward_kinematics.fk_phi.phi0 - vmc->forward_kinematics.fk_phi.phi3) * sinf(vmc->forward_kinematics.fk_phi.phi1 - vmc->forward_kinematics.fk_phi.phi2)
            / (vmc->forward_kinematics.fk_L0.L0 * sinf(vmc->forward_kinematics.fk_phi.phi3 - vmc->forward_kinematics.fk_phi.phi2));

    vmc->forward_kinematics.J_F_to_T.E.x1_2 =
            physical_config->l1 * sinf(vmc->forward_kinematics.fk_phi.phi0 - vmc->forward_kinematics.fk_phi.phi3) * sinf(vmc->forward_kinematics.fk_phi.phi1 - vmc->forward_kinematics.fk_phi.phi2)
            / sinf(vmc->forward_kinematics.fk_phi.phi3 - vmc->forward_kinematics.fk_phi.phi2);

    vmc->forward_kinematics.J_F_to_T.E.x2_1 =
            physical_config->l4 * cosf(vmc->forward_kinematics.fk_phi.phi0 - vmc->forward_kinematics.fk_phi.phi2) * sinf(vmc->forward_kinematics.fk_phi.phi3 - vmc->forward_kinematics.fk_phi.phi4)
            / (vmc->forward_kinematics.fk_L0.L0 * sinf(vmc->forward_kinematics.fk_phi.phi3 - vmc->forward_kinematics.fk_phi.phi2));

    vmc->forward_kinematics.J_F_to_T.E.x2_2 = physical_config->l4 * sinf(vmc->forward_kinematics.fk_phi.phi0 - vmc->forward_kinematics.fk_phi.phi2) * sinf(vmc->forward_kinematics.fk_phi.phi3 - vmc->forward_kinematics.fk_phi.phi4)
                                              / sinf(vmc->forward_kinematics.fk_phi.phi3 - vmc->forward_kinematics.fk_phi.phi2);

    Matrix_multiply(2, 2, vmc->forward_kinematics.J_F_to_T.array,
                    2, 1, vmc->forward_kinematics.Fxy_set_point.array,
                    vmc->forward_kinematics.T1_T4_set_point.array);
}


static void wheel_motors_torque_set(Chassis *chassis) {

    if (chassis == NULL) {
        return;
    }

    if (chassis->chassis_ctrl_mode != CHASSIS_SPIN) {

#if CHASSIS_REMOTE
        //
        chassis->wheel_turn_torque =  CHASSIS_TURN_PID_P * (chassis->imu_reference.yaw_total_angle - chassis->chassis_ctrl_info.yaw_angle_rad)
                                      + CHASSIS_TURN_PID_D * chassis->imu_reference.yaw_gyro;
#else
        if (chassis->is_chassis_balance)
        {
            chassis->chassis_ctrl_info.spin_speed= pid_calc(&chassis->chassis_vw_pos_pid,chassis->chassis_ctrl_info.yaw_angle_rad,0);
            chassis->wheel_turn_torque = pid_calc(&chassis->chassis_vw_speed_pid,chassis->imu_reference.yaw_gyro,chassis->chassis_ctrl_info.spin_speed);
        }
        else{
            chassis->wheel_turn_torque=0;
        }
#endif
    }else {
        chassis->wheel_turn_torque = pid_calc(&chassis->chassis_spin_pid,
                                              chassis->imu_reference.yaw_gyro,
                                              4);
    }




    wheel_L_balance_torque  = -chassis->leg_L.state_variable_wheel_out.theta
                              - chassis->leg_L.state_variable_wheel_out.theta_dot // ��
                              - chassis->leg_L.state_variable_wheel_out.phi
                              - chassis->leg_L.state_variable_wheel_out.phi_dot;

    wheel_R_balance_torque = chassis->leg_R.state_variable_wheel_out.theta
                             + chassis->leg_R.state_variable_wheel_out.theta_dot // ��
                             + chassis->leg_R.state_variable_wheel_out.phi
                             + chassis->leg_R.state_variable_wheel_out.phi_dot;

    wheel_L_move_torque = -chassis->leg_L.state_variable_wheel_out.x
                          - chassis->leg_L.state_variable_wheel_out.x_dot;

    wheel_R_move_torque =chassis->leg_R.state_variable_wheel_out.x
                         + chassis->leg_R.state_variable_wheel_out.x_dot;

    //��������ӽ���
//    if(Referee.PowerHeatData.chassis_power_buffer<10){
//        is_dangerous=true;
//        //ϵ��cԽ�����ƵĹ���ֵԽ�ͣ����������80w�����ƵĹ��ʽ���60w���ǵ��̹��ʱ����ƺ�,�����ɴﵽ60w���£��������ᳬ��80w
//    }

    chassis->leg_L.wheel_torque = 0;
    chassis->leg_R.wheel_torque = 0;

    chassis->leg_L.wheel_torque -= chassis->leg_L.state_variable_wheel_out.theta;//
    chassis->leg_L.wheel_torque -= chassis->leg_L.state_variable_wheel_out.theta_dot;// ��
    chassis->leg_L.wheel_torque -= chassis->leg_L.state_variable_wheel_out.x;
    chassis->leg_L.wheel_torque -= chassis->leg_L.state_variable_wheel_out.x_dot;
    chassis->leg_L.wheel_torque -= chassis->leg_L.state_variable_wheel_out.phi;//
    chassis->leg_L.wheel_torque -= chassis->leg_L.state_variable_wheel_out.phi_dot;
    chassis->leg_L.wheel_torque -= chassis->wheel_turn_torque;

    chassis->leg_R.wheel_torque += chassis->leg_R.state_variable_wheel_out.theta;
    chassis->leg_R.wheel_torque += chassis->leg_R.state_variable_wheel_out.theta_dot; // ��
    chassis->leg_R.wheel_torque += chassis->leg_R.state_variable_wheel_out.x;
    chassis->leg_R.wheel_torque += chassis->leg_R.state_variable_wheel_out.x_dot;
    chassis->leg_R.wheel_torque += chassis->leg_R.state_variable_wheel_out.phi;
    chassis->leg_R.wheel_torque += chassis->leg_R.state_variable_wheel_out.phi_dot;
    chassis->leg_R.wheel_torque -= chassis->wheel_turn_torque;



    chassis->power_nihe=3.1604*chassis->leg_L.wheel_torque*chassis->leg_L.wheel_torque+3.3810*chassis->leg_R.wheel_torque*chassis->leg_R.wheel_torque
                        +0.0140*chassis->leg_L.wheel_torque*motor_w_l+0.0142*chassis->leg_R.wheel_torque*motor_w_r
                        +0.6685+0.3708
                        +0.0116* fabs(motor_w_l)+0.0102* fabs(motor_w_r);
    chassis->power_nihe = fmaxf(chassis->power_nihe, 0);

    if(chassis->power_nihe>Referee.GameRobotStat.chassis_power_limit-10)
    {
        //�������һ������˥��ϵ�� K ��һԪ���η��� k1_l * (wheel_L_balance_torque+ wheel_L_move_torque- K*wheel_turn_torque)^2 + k1_r * (wheel_R_balance_torque + wheel_R_move_torque - K*wheel_turn_torque)^2
        //                  + k2_l * (wheel_L_balance_torque + wheel_L_move_torque -K*wheel_turn_torque)*motor_w_l + k2_r *(wheel_R_balance_torque+ wheel_R_move_torque - K*wheel_turn_torque)*motor_w_r)
        //                  + k3_l+k3_r+k4_l*absf(motor_w_l)+k4_r*absf(motor_w_r) - power_limit=0
        //����     const float k1_l = 3.1604f;
        //        const float k2_l = 0.0140f;
        //        const float k3_l = 0.6685f;
        //        const float k4_l = 0.0116f;
        //
        //        const float k1_r = 3.3810f;
        //        const float k2_r = 0.0142f;
        //        const float k3_r = 0.3708f;
        //        const float k4_r = 0.0102f;
        //�����淽��չ��������Ϊ a*K^2+b*K+c=0��
        const float k1_l = 3.1604f;
        const float k2_l = 0.0140f;
        const float k3_l = 0.6685f;
        const float k4_l = 0.0116f;

        const float k1_r = 3.3810f;
        const float k2_r = 0.0142f;
        const float k3_r = 0.3708f;
        const float k4_r = 0.0102f;

        // �����м����
        float T_turn = chassis->wheel_turn_torque;
        float A_L = wheel_L_balance_torque + wheel_L_move_torque;
        float A_R = wheel_R_balance_torque + wheel_R_move_torque;
        float W_L = motor_w_l;
        float W_R = motor_w_r;

        // ����ϵ�� a, b, c
        float a = T_turn * T_turn * (k1_l + k1_r);
        float b = -T_turn * (2 * k1_l * A_L + 2 * k1_r * A_R + k2_l * W_L + k2_r * W_R);
        float c = k1_l * A_L * A_L + k1_r * A_R * A_R
                  + k2_l * A_L * W_L + k2_r * A_R * W_R
                  + (k3_l + k3_r + k4_l * fabs(W_L) + k4_r * fabs(W_R) - (Referee.GameRobotStat.chassis_power_limit-10));

        chassis->last_chassis_power_K=chassis->chassis_power_K;

        float discriminant = b*b - 4.0f*a*c;
        float sqrt_result;
        test_delta =discriminant;
        // ���Ƚ��ĸ�ѡ�����
        if(discriminant >= 0.0f) {
            arm_sqrt_f32(discriminant, &sqrt_result);
             float root1 = (-b + sqrt_result) / (2.0f*a);
             float root2 = (-b - sqrt_result) / (2.0f*a);

            test_root1=root1;
            test_root2=root2;
            // ����ѡ�������������ĸ�
            if (root1>=0 && root1<=1) chassis->chassis_power_K=root1;
            else if (root2>=0 && root2<=1) chassis->chassis_power_K=root2;
            else chassis->chassis_power_K=0;
        } else {
            chassis->chassis_power_K = -b/(2*a); // ȡ��ֵ��
            chassis->chassis_power_K = fminf(fmaxf(chassis->chassis_power_K, 0), 1);
        }
    }
    else chassis->last_chassis_power_K=chassis->chassis_power_K=1;


    chassis->leg_L.wheel_torque = 0;
    chassis->leg_R.wheel_torque = 0;

    chassis->leg_L.wheel_torque -= chassis->leg_L.state_variable_wheel_out.theta;//
    chassis->leg_L.wheel_torque -= chassis->leg_L.state_variable_wheel_out.theta_dot;// ��
    chassis->leg_L.wheel_torque -= chassis->leg_L.state_variable_wheel_out.x;
    chassis->leg_L.wheel_torque -= chassis->leg_L.state_variable_wheel_out.x_dot;
    chassis->leg_L.wheel_torque -= chassis->leg_L.state_variable_wheel_out.phi;//
    chassis->leg_L.wheel_torque -= chassis->leg_L.state_variable_wheel_out.phi_dot;
    chassis->leg_L.wheel_torque -= chassis->chassis_power_K *chassis->wheel_turn_torque;




    chassis->leg_R.wheel_torque += chassis->leg_R.state_variable_wheel_out.theta;
    chassis->leg_R.wheel_torque += chassis->leg_R.state_variable_wheel_out.theta_dot; // ��
    chassis->leg_R.wheel_torque += chassis->leg_R.state_variable_wheel_out.x;
    chassis->leg_R.wheel_torque += chassis->leg_R.state_variable_wheel_out.x_dot;
    chassis->leg_R.wheel_torque += chassis->leg_R.state_variable_wheel_out.phi;
    chassis->leg_R.wheel_torque += chassis->leg_R.state_variable_wheel_out.phi_dot;
    chassis->leg_R.wheel_torque -= chassis->chassis_power_K *chassis->wheel_turn_torque;

//    chassis->leg_L.wheel_torque = 0;
//    chassis->leg_R.wheel_torque = 0;
//
//    chassis->leg_L.wheel_torque -= chassis->leg_L.state_variable_wheel_out.theta;//
//    chassis->leg_L.wheel_torque -= chassis->leg_L.state_variable_wheel_out.theta_dot;// ��
//    chassis->leg_L.wheel_torque -= chassis->leg_L.state_variable_wheel_out.x;
//    chassis->leg_L.wheel_torque -= chassis->leg_L.state_variable_wheel_out.x_dot;
//    chassis->leg_L.wheel_torque -= chassis->leg_L.state_variable_wheel_out.phi;//
//    chassis->leg_L.wheel_torque -= chassis->leg_L.state_variable_wheel_out.phi_dot;
//    chassis->leg_L.wheel_torque -= chassis->wheel_turn_torque;
//
//    chassis->leg_R.wheel_torque += chassis->leg_R.state_variable_wheel_out.theta;
//    chassis->leg_R.wheel_torque += chassis->leg_R.state_variable_wheel_out.theta_dot; // ��
//    chassis->leg_R.wheel_torque += chassis->leg_R.state_variable_wheel_out.x;
//    chassis->leg_R.wheel_torque += chassis->leg_R.state_variable_wheel_out.x_dot;
//    chassis->leg_R.wheel_torque += chassis->leg_R.state_variable_wheel_out.phi;
//    chassis->leg_R.wheel_torque += chassis->leg_R.state_variable_wheel_out.phi_dot;
//    chassis->leg_R.wheel_torque -= chassis->wheel_turn_torque;


    chassis->power_nihe2=3.1604f*chassis->leg_L.wheel_torque*chassis->leg_L.wheel_torque+3.3810*chassis->leg_R.wheel_torque*chassis->leg_R.wheel_torque
                         +0.0140*chassis->leg_L.wheel_torque*motor_w_l+0.0142*chassis->leg_R.wheel_torque*motor_w_r
                         +0.6685+0.3708
                         +0.0116* fabs(motor_w_l)+0.0102* fabs(motor_w_r);
    chassis->power_nihe2 = fmaxf(chassis->power_nihe2, 0);


    // ��ش���
    if (chassis->leg_L.leg_is_offground || chassis->leg_R.leg_is_offground) {

        if(chassis->leg_L.leg_is_offground)
        {
            chassis->leg_L.wheel_torque = 0.0f;
        }

        if(chassis->leg_R.leg_is_offground)
        {
            chassis->leg_R.wheel_torque = 0.0f;
        }
    }

    VAL_LIMIT(chassis->leg_L.wheel_torque, MIN_WHEEL_TORQUE, MAX_WHEEL_TORQUE);
    VAL_LIMIT(chassis->leg_R.wheel_torque, MIN_WHEEL_TORQUE, MAX_WHEEL_TORQUE);
}

static void joint_motors_torque_set(Chassis *chassis,
                                    ChassisPhysicalConfig *chassis_physical_config) {

    /** Tp Tp Tp Tp Tp Tp Tp Tp Tp Tp Tp Tp Tp Tp Tp Tp Tp Tp Tp Tp Tp Tp **/

    chassis->leg_L.vmc.forward_kinematics.Fxy_set_point.E.Tp_set_point = 0;
    chassis->leg_R.vmc.forward_kinematics.Fxy_set_point.E.Tp_set_point = 0;

    /****** ������pid ******/
    // ����νtheta����phi0 ���ù�Ӳ����
    chassis->steer_compensatory_torque = pid_calc(&chassis->chassis_leg_coordination_pid,
                                                  chassis->theta_error,
                                                  0);



//    //Left
//    chassis->leg_L.vmc.forward_kinematics.Fxy_set_point.E.Tp_set_point += chassis->leg_L.state_variable_joint_out.theta; // ��
//    chassis->leg_L.vmc.forward_kinematics.Fxy_set_point.E.Tp_set_point += chassis->leg_L.state_variable_joint_out.theta_dot; // ��
//    chassis->leg_L.vmc.forward_kinematics.Fxy_set_point.E.Tp_set_point += chassis->chassis_power_K*chassis->leg_L.state_variable_joint_out.x; // ��
//    chassis->leg_L.vmc.forward_kinematics.Fxy_set_point.E.Tp_set_point += chassis->chassis_power_K*chassis->leg_L.state_variable_joint_out.x_dot; // ��
//    chassis->leg_L.vmc.forward_kinematics.Fxy_set_point.E.Tp_set_point += chassis->leg_L.state_variable_joint_out.phi; // ! Ӧ��ûɶ����� ����������˸��Ÿ���
//    chassis->leg_L.vmc.forward_kinematics.Fxy_set_point.E.Tp_set_point += chassis->leg_L.state_variable_joint_out.phi_dot;
//    chassis->leg_L.vmc.forward_kinematics.Fxy_set_point.E.Tp_set_point -= chassis->steer_compensatory_torque;
//
//    //Right
//    chassis->leg_R.vmc.forward_kinematics.Fxy_set_point.E.Tp_set_point += chassis->leg_R.state_variable_joint_out.theta; // ��
//    chassis->leg_R.vmc.forward_kinematics.Fxy_set_point.E.Tp_set_point += chassis->leg_R.state_variable_joint_out.theta_dot; // ��
//    chassis->leg_R.vmc.forward_kinematics.Fxy_set_point.E.Tp_set_point += chassis->chassis_power_K*chassis->leg_R.state_variable_joint_out.x; // ��
//    chassis->leg_R.vmc.forward_kinematics.Fxy_set_point.E.Tp_set_point += chassis->chassis_power_K*chassis->leg_R.state_variable_joint_out.x_dot; // ��
//    chassis->leg_R.vmc.forward_kinematics.Fxy_set_point.E.Tp_set_point += chassis->leg_R.state_variable_joint_out.phi; // ! Ӧ��ûɶ����� ����������˸��Ÿ���
//    chassis->leg_R.vmc.forward_kinematics.Fxy_set_point.E.Tp_set_point += chassis->leg_R.state_variable_joint_out.phi_dot;
//    chassis->leg_R.vmc.forward_kinematics.Fxy_set_point.E.Tp_set_point += chassis->steer_compensatory_torque;

    //Left
    chassis->leg_L.vmc.forward_kinematics.Fxy_set_point.E.Tp_set_point += chassis->leg_L.state_variable_joint_out.theta; // ��
    chassis->leg_L.vmc.forward_kinematics.Fxy_set_point.E.Tp_set_point += chassis->leg_L.state_variable_joint_out.theta_dot; // ��
    chassis->leg_L.vmc.forward_kinematics.Fxy_set_point.E.Tp_set_point += chassis->leg_L.state_variable_joint_out.x; // ��
    chassis->leg_L.vmc.forward_kinematics.Fxy_set_point.E.Tp_set_point += chassis->leg_L.state_variable_joint_out.x_dot; // ��
    chassis->leg_L.vmc.forward_kinematics.Fxy_set_point.E.Tp_set_point += chassis->leg_L.state_variable_joint_out.phi; // ! Ӧ��ûɶ����� ����������˸��Ÿ���
    chassis->leg_L.vmc.forward_kinematics.Fxy_set_point.E.Tp_set_point += chassis->leg_L.state_variable_joint_out.phi_dot;
    chassis->leg_L.vmc.forward_kinematics.Fxy_set_point.E.Tp_set_point -= chassis->steer_compensatory_torque;

    //Right
    chassis->leg_R.vmc.forward_kinematics.Fxy_set_point.E.Tp_set_point += chassis->leg_R.state_variable_joint_out.theta; // ��
    chassis->leg_R.vmc.forward_kinematics.Fxy_set_point.E.Tp_set_point += chassis->leg_R.state_variable_joint_out.theta_dot; // ��
    chassis->leg_R.vmc.forward_kinematics.Fxy_set_point.E.Tp_set_point += chassis->leg_R.state_variable_joint_out.x; // ��
    chassis->leg_R.vmc.forward_kinematics.Fxy_set_point.E.Tp_set_point += chassis->leg_R.state_variable_joint_out.x_dot; // ��
    chassis->leg_R.vmc.forward_kinematics.Fxy_set_point.E.Tp_set_point += chassis->leg_R.state_variable_joint_out.phi; // ! Ӧ��ûɶ����� ����������˸��Ÿ���
    chassis->leg_R.vmc.forward_kinematics.Fxy_set_point.E.Tp_set_point += chassis->leg_R.state_variable_joint_out.phi_dot;
    chassis->leg_R.vmc.forward_kinematics.Fxy_set_point.E.Tp_set_point += chassis->steer_compensatory_torque;

    /** Fy Fy Fy Fy Fy Fy Fy Fy Fy Fy Fy Fy Fy Fy Fy Fy Fy Fy Fy Fy Fy Fy **/

    chassis->leg_L.vmc.forward_kinematics.Fxy_set_point.E.Fy_set_point = 0.0f;
    chassis->leg_R.vmc.forward_kinematics.Fxy_set_point.E.Fy_set_point = 0.0f;

    /****** Leg pid ******/
    // �����˶� -- ����pid
    // ��ª�ġ���Խ����̡�

    if(chassis->chassis_ctrl_info.height_m == 0.10f)
    {
        chassis->leg_L.leg_offset = -0.07f;
        chassis->leg_R.leg_offset = -0.07f;
    }else if(chassis->chassis_ctrl_info.height_m == 0.18f)
    {
        chassis->leg_L.leg_offset = -0.033f;
        chassis->leg_R.leg_offset = -0.038f;
    }else if(chassis->chassis_ctrl_info.height_m == 0.35f)
    {
        chassis->leg_L.leg_offset = -0.02f;
        chassis->leg_R.leg_offset = -0.03f;
    }

    float L_L0_dot_set = pid_calc(&chassis->leg_L.leg_pos_pid,
                                  chassis->leg_L.vmc.forward_kinematics.fk_L0.L0,
                                  chassis->chassis_ctrl_info.height_m + chassis->leg_L.leg_offset);

    float R_L0_dot_set = pid_calc(&chassis->leg_R.leg_pos_pid,
                                  chassis->leg_R.vmc.forward_kinematics.fk_L0.L0,
                                  chassis->chassis_ctrl_info.height_m + chassis->leg_R.leg_offset);

    L_L0_dot_test = L_L0_dot_set;
    R_L0_dot_test = R_L0_dot_set;

    pid_calc(&chassis->leg_L.leg_speed_pid,
             chassis->leg_L.vmc.forward_kinematics.fk_L0.L0_dot,
             L_L0_dot_set);

    pid_calc(&chassis->leg_R.leg_speed_pid,
             chassis->leg_R.vmc.forward_kinematics.fk_L0.L0_dot,
             R_L0_dot_set);

//    float L_L0_dot_set = pid_calc(&chassis->leg_L.leg_pos_pid,
//                                  chassis->leg_L.vmc.forward_kinematics.fk_L0.L0,
//                                  chassis->chassis_ctrl_info.height_m);
//
//    float R_L0_dot_set = pid_calc(&chassis->leg_R.leg_pos_pid,
//                                  chassis->leg_R.vmc.forward_kinematics.fk_L0.L0,
//                                  chassis->chassis_ctrl_info.height_m);
//
//    L_L0_dot_test = L_L0_dot_set;
//    R_L0_dot_test = R_L0_dot_set;
//
//    pid_calc(&chassis->leg_L.leg_speed_pid,
//             chassis->leg_L.vmc.forward_kinematics.fk_L0.L0_dot,
//             L_L0_dot_test);
//
//    pid_calc(&chassis->leg_R.leg_speed_pid,
//             chassis->leg_R.vmc.forward_kinematics.fk_L0.L0_dot,
//             R_L0_dot_test);

    // ����ȳ�pid
    pid_calc(&chassis->leg_L.offground_leg_pid,
             chassis->leg_L.vmc.forward_kinematics.fk_L0.L0,
             chassis->chassis_ctrl_info.height_m);

    pid_calc(&chassis->leg_R.offground_leg_pid,
             chassis->leg_R.vmc.forward_kinematics.fk_L0.L0,
             chassis->chassis_ctrl_info.height_m);

    /****** Roll pid ******/
    pid_calc(&chassis->chassis_roll_pid,
             chassis->imu_reference.roll_angle,
             chassis->chassis_ctrl_info.roll_angle_rad);


    chassis->leg_L.vmc.forward_kinematics.Fxy_set_point.E.Fy_set_point =  chassis_physical_config->body_weight * GRAVITY * cosf(chassis->leg_L.state_variable_feedback.theta)
                                                                        + chassis->leg_L.leg_speed_pid.out
                                                                        + chassis->chassis_roll_pid.out;

    chassis->leg_R.vmc.forward_kinematics.Fxy_set_point.E.Fy_set_point =  chassis_physical_config->body_weight * GRAVITY * cosf(chassis->leg_R.state_variable_feedback.theta)
                                                                        + chassis->leg_R.leg_speed_pid.out
                                                                        - chassis->chassis_roll_pid.out;

    /***********************************************************************/

    // ��ش���
    if (chassis->leg_L.leg_is_offground || chassis->leg_R.leg_is_offground) {

        if(chassis->leg_L.leg_is_offground)
        {
            chassis->leg_L.vmc.forward_kinematics.Fxy_set_point.E.Tp_set_point = 0.0f;
            chassis->leg_L.vmc.forward_kinematics.Fxy_set_point.E.Tp_set_point += chassis->leg_L.state_variable_joint_out.theta; // ��
            chassis->leg_L.vmc.forward_kinematics.Fxy_set_point.E.Tp_set_point += chassis->leg_L.state_variable_joint_out.theta_dot;
            chassis->leg_L.vmc.forward_kinematics.Fxy_set_point.E.Tp_set_point -= chassis->steer_compensatory_torque;

            chassis->leg_L.vmc.forward_kinematics.Fxy_set_point.E.Fy_set_point = 0.0f;
            chassis->leg_L.vmc.forward_kinematics.Fxy_set_point.E.Fy_set_point = chassis->leg_L.offground_leg_pid.out;
        }

        if(chassis->leg_R.leg_is_offground)
        {
            chassis->leg_R.vmc.forward_kinematics.Fxy_set_point.E.Tp_set_point = 0.0f;
            chassis->leg_R.vmc.forward_kinematics.Fxy_set_point.E.Tp_set_point += chassis->leg_R.state_variable_joint_out.theta; // ��
            chassis->leg_R.vmc.forward_kinematics.Fxy_set_point.E.Tp_set_point += chassis->leg_R.state_variable_joint_out.theta_dot;
            chassis->leg_R.vmc.forward_kinematics.Fxy_set_point.E.Tp_set_point += chassis->steer_compensatory_torque;

            chassis->leg_R.vmc.forward_kinematics.Fxy_set_point.E.Fy_set_point = 0.0f;
            chassis->leg_R.vmc.forward_kinematics.Fxy_set_point.E.Fy_set_point = chassis->leg_R.offground_leg_pid.out;
        }
    }


// ����ؽڵ������
        forward_dynamics(&chassis->leg_L.vmc, chassis_physical_config);
        forward_dynamics(&chassis->leg_R.vmc, chassis_physical_config);

        chassis->leg_L.joint_F_torque = chassis->leg_L.vmc.forward_kinematics.T1_T4_set_point.E.T1_set_point;//F
        chassis->leg_L.joint_B_torque = chassis->leg_L.vmc.forward_kinematics.T1_T4_set_point.E.T4_set_point;//B

        chassis->leg_R.joint_F_torque = chassis->leg_R.vmc.forward_kinematics.T1_T4_set_point.E.T1_set_point;//F
        chassis->leg_R.joint_B_torque = chassis->leg_R.vmc.forward_kinematics.T1_T4_set_point.E.T4_set_point;//B

        // ����޷�
        VAL_LIMIT(chassis->leg_R.joint_F_torque, MIN_JOINT_TORQUE, MAX_JOINT_TORQUE);
        VAL_LIMIT(chassis->leg_R.joint_B_torque, MIN_JOINT_TORQUE, MAX_JOINT_TORQUE);
        VAL_LIMIT(chassis->leg_L.joint_F_torque, MIN_JOINT_TORQUE, MAX_JOINT_TORQUE);
        VAL_LIMIT(chassis->leg_L.joint_B_torque, MIN_JOINT_TORQUE, MAX_JOINT_TORQUE);

}

// ������ȳ��仯�ٶȡ��ڽǱ仯�ٶ�
static void Inverse_Kinematics(VMC *vmc,
                               float w1,
                               float w4,
                               ChassisPhysicalConfig *chassis_physical_config) {
    if (vmc == NULL) {
        return;
    }
    vmc->inverse_kinematics.W_fdb.E.w1_fdb = w1;
    vmc->inverse_kinematics.W_fdb.E.w4_fdb = w4;

    vmc->inverse_kinematics.J_w_to_v.E.x1_1 =  -chassis_physical_config->l1 * sinf(vmc->forward_kinematics.fk_phi.phi0 - vmc->forward_kinematics.fk_phi.phi3) * sinf(vmc->forward_kinematics.fk_phi.phi1 - vmc->forward_kinematics.fk_phi.phi2)
                                               / sinf(vmc->forward_kinematics.fk_phi.phi2 - vmc->forward_kinematics.fk_phi.phi3);


    vmc->inverse_kinematics.J_w_to_v.E.x1_2 = -chassis_physical_config->l4 * sinf(vmc->forward_kinematics.fk_phi.phi0 - vmc->forward_kinematics.fk_phi.phi2) * sinf(vmc->forward_kinematics.fk_phi.phi3 - vmc->forward_kinematics.fk_phi.phi4)
                                              / sinf(vmc->forward_kinematics.fk_phi.phi2 - vmc->forward_kinematics.fk_phi.phi3);


    vmc->inverse_kinematics.J_w_to_v.E.x2_1 = -chassis_physical_config->l1 * cosf(vmc->forward_kinematics.fk_phi.phi0 - vmc->forward_kinematics.fk_phi.phi3) * sinf(vmc->forward_kinematics.fk_phi.phi1 - vmc->forward_kinematics.fk_phi.phi2)
                                              / (vmc->forward_kinematics.fk_L0.L0 * sinf(vmc->forward_kinematics.fk_phi.phi2 - vmc->forward_kinematics.fk_phi.phi3));


    vmc->inverse_kinematics.J_w_to_v.E.x2_2 = -chassis_physical_config->l4 * cosf(vmc->forward_kinematics.fk_phi.phi0 - vmc->forward_kinematics.fk_phi.phi2) * sinf(vmc->forward_kinematics.fk_phi.phi3 - vmc->forward_kinematics.fk_phi.phi4)
                                              / (vmc->forward_kinematics.fk_L0.L0 * sinf(vmc->forward_kinematics.fk_phi.phi2 - vmc->forward_kinematics.fk_phi.phi3));

    Matrix_multiply(2, 2, vmc->inverse_kinematics.J_w_to_v.array,
                    2, 1, vmc->inverse_kinematics.W_fdb.array,
                    vmc->inverse_kinematics.V_fdb.array);
}

// �������������غ����ȷ���֧����
static void Inverse_Dynamics(VMC *vmc,
                             float T1, // phi1
                             float T4, // phi4
                             ChassisPhysicalConfig *chassis_physical_config) {
    if (vmc == NULL) {
        return;
    }
    vmc->inverse_kinematics.T1_T4_fdb.E.T1_fdb = T1;
    vmc->inverse_kinematics.T1_T4_fdb.E.T4_fdb = T4;

    vmc->inverse_kinematics.J_T_to_F.E.x1_1 =
            vmc->forward_kinematics.fk_L0.L0 * sinf(vmc->forward_kinematics.fk_phi.phi0 - vmc->forward_kinematics.fk_phi.phi2)
            / (chassis_physical_config->l1
               * sinf(vmc->forward_kinematics.fk_phi.phi1 - vmc->forward_kinematics.fk_phi.phi2));

    vmc->inverse_kinematics.J_T_to_F.E.x1_2 =
            vmc->forward_kinematics.fk_L0.L0 * sinf(vmc->forward_kinematics.fk_phi.phi0 - vmc->forward_kinematics.fk_phi.phi3)
            / (chassis_physical_config->l4
               * sinf(vmc->forward_kinematics.fk_phi.phi4 - vmc->forward_kinematics.fk_phi.phi3));

    vmc->inverse_kinematics.J_T_to_F.E.x2_1 = cosf(vmc->forward_kinematics.fk_phi.phi0 - vmc->forward_kinematics.fk_phi.phi2)
                                              / (chassis_physical_config->l1 * sinf(vmc->forward_kinematics.fk_phi.phi2 - vmc->forward_kinematics.fk_phi.phi1));

    vmc->inverse_kinematics.J_T_to_F.E.x2_2 = cosf(vmc->forward_kinematics.fk_phi.phi0 - vmc->forward_kinematics.fk_phi.phi3)
                                              / (chassis_physical_config->l4 * sinf(vmc->forward_kinematics.fk_phi.phi3 - vmc->forward_kinematics.fk_phi.phi4));


    Matrix_multiply(2, 2, vmc->inverse_kinematics.J_T_to_F.array,
                    2, 1, vmc->inverse_kinematics.T1_T4_fdb.array,
                    vmc->inverse_kinematics.Fxy_fdb.array);
}

// ������ֱ����֧����
static void fn_cal(Leg *leg, float az, ChassisPhysicalConfig *chassis_physical_config) {

    if (leg == NULL) {
        return;
    }

    // �����������ݼ���
    float P = leg->vmc.inverse_kinematics.Fxy_fdb.E.Fy_fdb * cosf(leg->state_variable_feedback.theta)
              + leg->vmc.inverse_kinematics.Fxy_fdb.E.Tp_fdb * sinf(leg->state_variable_feedback.theta) / leg->vmc.forward_kinematics.fk_L0.L0;

    float leg_az = az - leg->vmc.forward_kinematics.fk_L0.L0_ddot * cosf(leg->state_variable_feedback.theta)
             + 2.0f * leg->vmc.forward_kinematics.fk_L0.L0_dot * leg->state_variable_feedback.theta_dot
               * sinf(leg->state_variable_feedback.theta)
             + leg->vmc.forward_kinematics.fk_L0.L0 * leg->state_variable_feedback.theta_ddot
               * sinf(leg->state_variable_feedback.theta)
             + leg->vmc.forward_kinematics.fk_L0.L0 * leg->state_variable_feedback.theta_dot
               * leg->state_variable_feedback.theta_dot * cosf(leg->state_variable_feedback.theta);

    float Fn_raw = P + chassis_physical_config->wheel_weight * 9.8f + chassis_physical_config->wheel_weight * leg_az;

    update_moving_average_filter(&leg->Fn_filter, Fn_raw);
    leg->Fn = get_moving_average_filtered_value(&leg->Fn_filter);

}

static void leg_is_offground(Chassis* chassis)
{
    if(chassis->leg_L.Fn < 233.0f)
    {
        chassis->leg_L.leg_is_offground = true;
    }else{
        chassis->leg_L.leg_is_offground = false;
    }

    if(chassis->leg_R.Fn < 233.0f)
    {
        chassis->leg_R.leg_is_offground = true;
    }else{
        chassis->leg_R.leg_is_offground = false;
    }
}

/*******************************************************************************
 *                                     VMC                                     *
 *******************************************************************************/
void vmc_ctrl(void) {

    // ����phi1 phi4
    vmc_phi_update(&chassis.leg_L, &chassis.leg_R);

    // VMC ���˶�ѧ����
    forward_kinematics(&chassis.leg_L, &chassis.leg_R, &chassis_physical_config);

    // �����������
    wheel_motors_torque_set(&chassis);
    joint_motors_torque_set(&chassis, &chassis_physical_config);

    // VMC �涯��ѧ����
    // ���ݹؽڷ�������������������(F Tp)
  Inverse_Dynamics(&chassis.leg_L.vmc,
                   (get_joint_motors() + 1)->torque,
                   get_joint_motors()->torque,
                   &chassis_physical_config);
  Inverse_Dynamics(&chassis.leg_R.vmc,
                   -(get_joint_motors() + 2)->torque,
                   -(get_joint_motors() + 3)->torque,
                   &chassis_physical_config);

  // ��ֱ����֧��������
  fn_cal(&chassis.leg_L, chassis.imu_reference.robot_az, &chassis_physical_config);
  fn_cal(&chassis.leg_R, chassis.imu_reference.robot_az, &chassis_physical_config);

//  // �Ȳ���ؼ��
//  leg_is_offground(&chassis);
}