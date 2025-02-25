#include <stdbool.h>
#include <math.h>

#include "robot_def.h"
#include "vmc.h"
#include "user_lib.h"
#include "joint.h"
#include "moving_filter.h"
#include "Referee.h"

extern double power_nihe;
extern Chassis chassis;
extern ChassisPhysicalConfig chassis_physical_config;

float L_L0_dot_test, R_L0_dot_test;

/*               正方向
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

// vmc正运动学解算
static void forward_kinematics(Leg* leg_L, Leg* leg_R, ChassisPhysicalConfig *physical_config) {
    /***LEG_L LEG_L LEG_L LEG_L LEG_L LEG_L LEG_L LEG_L LEG_L LEG_L LEG_L LEG_L LEG_L LEG_L LEG_L LEG_L LEG_L LEG_L LEG_L LEG_L***/

    leg_L->vmc.forward_kinematics.fk_point_coordinates.b_x = physical_config->l1 * cosf(leg_L->vmc.forward_kinematics.fk_phi.phi1);
    leg_L->vmc.forward_kinematics.fk_point_coordinates.b_y = physical_config->l1 * sinf(leg_L->vmc.forward_kinematics.fk_phi.phi1);
    leg_L->vmc.forward_kinematics.fk_point_coordinates.d_x = physical_config->l5 + physical_config->l4 * cosf(leg_L->vmc.forward_kinematics.fk_phi.phi4);
    leg_L->vmc.forward_kinematics.fk_point_coordinates.d_y = physical_config->l4 * sinf(leg_L->vmc.forward_kinematics.fk_phi.phi4); // l5 改成了 l4

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

// vmc正动力学解算
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


    double wheel_L_balance_torque,wheel_R_balance_torque;
    double wheel_L_move_torque,wheel_R_move_torque;

    wheel_L_balance_torque  = chassis->leg_L.state_variable_wheel_out.theta
                              + chassis->leg_L.state_variable_wheel_out.theta_dot // √
                              + chassis->leg_L.state_variable_wheel_out.phi
                              + chassis->leg_L.state_variable_wheel_out.phi_dot;

    wheel_R_balance_torque = chassis->leg_R.state_variable_wheel_out.theta
                             + chassis->leg_R.state_variable_wheel_out.theta_dot // √
                             + chassis->leg_R.state_variable_wheel_out.phi
                             + chassis->leg_R.state_variable_wheel_out.phi_dot;

    wheel_L_move_torque = chassis->leg_L.state_variable_wheel_out.x
                          + chassis->leg_L.state_variable_wheel_out.x_dot;

    wheel_R_move_torque =chassis->leg_R.state_variable_wheel_out.x
                         + chassis->leg_R.state_variable_wheel_out.x_dot;
    if(power_nihe>Referee.GameRobotStat.chassis_power_limit)
    {
        //求解这样一个一元二次方程 a*（wheel_L_balance_torque+k*wheel_L_move_torque)^2 + b*（wheel_L_balance_torque+k*wheel_L_move_torque)*motor_w_l + c - power_limit/2=0
        //其中a=0.1843，b=0.0087，c=2.1

    }


    chassis->leg_L.wheel_torque = 0;
    chassis->leg_R.wheel_torque = 0;

    chassis->leg_L.wheel_torque += chassis->leg_L.state_variable_wheel_out.theta;//
    chassis->leg_L.wheel_torque += chassis->leg_L.state_variable_wheel_out.theta_dot;// √
    chassis->leg_L.wheel_torque += chassis->leg_L.state_variable_wheel_out.x;
    chassis->leg_L.wheel_torque += chassis->leg_L.state_variable_wheel_out.x_dot;
    chassis->leg_L.wheel_torque += chassis->leg_L.state_variable_wheel_out.phi;//
    chassis->leg_L.wheel_torque += chassis->leg_L.state_variable_wheel_out.phi_dot;
    chassis->leg_L.wheel_torque += chassis->wheel_turn_torque;

    chassis->leg_R.wheel_torque += chassis->leg_R.state_variable_wheel_out.theta;
    chassis->leg_R.wheel_torque += chassis->leg_R.state_variable_wheel_out.theta_dot; // √
    chassis->leg_R.wheel_torque += chassis->leg_R.state_variable_wheel_out.x;
    chassis->leg_R.wheel_torque += chassis->leg_R.state_variable_wheel_out.x_dot;
    chassis->leg_R.wheel_torque += chassis->leg_R.state_variable_wheel_out.phi;
    chassis->leg_R.wheel_torque += chassis->leg_R.state_variable_wheel_out.phi_dot;
    chassis->leg_R.wheel_torque -= chassis->wheel_turn_torque;
    chassis->leg_R.wheel_torque *= -1;



    // 离地处理
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

    /****** 防劈叉pid ******/
    // 无所谓theta或者phi0 调得够硬就行
    chassis->steer_compensatory_torque = pid_calc(&chassis->chassis_leg_coordination_pid,
                                                  chassis->theta_error,
                                                  0);

    //Left
    chassis->leg_L.vmc.forward_kinematics.Fxy_set_point.E.Tp_set_point += chassis->leg_L.state_variable_joint_out.theta; // √
    chassis->leg_L.vmc.forward_kinematics.Fxy_set_point.E.Tp_set_point += chassis->leg_L.state_variable_joint_out.theta_dot; // √
    chassis->leg_L.vmc.forward_kinematics.Fxy_set_point.E.Tp_set_point += chassis->leg_L.state_variable_joint_out.x; // √
    chassis->leg_L.vmc.forward_kinematics.Fxy_set_point.E.Tp_set_point += chassis->leg_L.state_variable_joint_out.x_dot; // √
    chassis->leg_L.vmc.forward_kinematics.Fxy_set_point.E.Tp_set_point += chassis->leg_L.state_variable_joint_out.phi; // ! 应该没啥问题吧 它的输出加了负号更爆
    chassis->leg_L.vmc.forward_kinematics.Fxy_set_point.E.Tp_set_point += chassis->leg_L.state_variable_joint_out.phi_dot;
    chassis->leg_L.vmc.forward_kinematics.Fxy_set_point.E.Tp_set_point -= chassis->steer_compensatory_torque;

    //Right
    chassis->leg_R.vmc.forward_kinematics.Fxy_set_point.E.Tp_set_point += chassis->leg_R.state_variable_joint_out.theta; // √
    chassis->leg_R.vmc.forward_kinematics.Fxy_set_point.E.Tp_set_point += chassis->leg_R.state_variable_joint_out.theta_dot; // √
    chassis->leg_R.vmc.forward_kinematics.Fxy_set_point.E.Tp_set_point += chassis->leg_R.state_variable_joint_out.x; // √
    chassis->leg_R.vmc.forward_kinematics.Fxy_set_point.E.Tp_set_point += chassis->leg_R.state_variable_joint_out.x_dot; // √
    chassis->leg_R.vmc.forward_kinematics.Fxy_set_point.E.Tp_set_point += chassis->leg_R.state_variable_joint_out.phi; // ! 应该没啥问题吧 它的输出加了负号更爆
    chassis->leg_R.vmc.forward_kinematics.Fxy_set_point.E.Tp_set_point += chassis->leg_R.state_variable_joint_out.phi_dot;
    chassis->leg_R.vmc.forward_kinematics.Fxy_set_point.E.Tp_set_point += chassis->steer_compensatory_torque;

    /** Fy Fy Fy Fy Fy Fy Fy Fy Fy Fy Fy Fy Fy Fy Fy Fy Fy Fy Fy Fy Fy Fy **/

    chassis->leg_L.vmc.forward_kinematics.Fxy_set_point.E.Fy_set_point = 0.0f;
    chassis->leg_R.vmc.forward_kinematics.Fxy_set_point.E.Fy_set_point = 0.0f;

    /****** Leg pid ******/
    // 正常运动 -- 串级pid
    // 丑陋的“面对结果编程”

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

    // 离地腿长pid
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

    // 离地处理
    if (chassis->leg_L.leg_is_offground || chassis->leg_R.leg_is_offground) {

        if(chassis->leg_L.leg_is_offground)
        {
            chassis->leg_L.vmc.forward_kinematics.Fxy_set_point.E.Tp_set_point = 0.0f;
            chassis->leg_L.vmc.forward_kinematics.Fxy_set_point.E.Tp_set_point += chassis->leg_L.state_variable_joint_out.theta; // √
            chassis->leg_L.vmc.forward_kinematics.Fxy_set_point.E.Tp_set_point += chassis->leg_L.state_variable_joint_out.theta_dot;
            chassis->leg_L.vmc.forward_kinematics.Fxy_set_point.E.Tp_set_point -= chassis->steer_compensatory_torque;

            chassis->leg_L.vmc.forward_kinematics.Fxy_set_point.E.Fy_set_point = 0.0f;
            chassis->leg_L.vmc.forward_kinematics.Fxy_set_point.E.Fy_set_point = chassis->leg_L.offground_leg_pid.out;
        }

        if(chassis->leg_R.leg_is_offground)
        {
            chassis->leg_R.vmc.forward_kinematics.Fxy_set_point.E.Tp_set_point = 0.0f;
            chassis->leg_R.vmc.forward_kinematics.Fxy_set_point.E.Tp_set_point += chassis->leg_R.state_variable_joint_out.theta; // √
            chassis->leg_R.vmc.forward_kinematics.Fxy_set_point.E.Tp_set_point += chassis->leg_R.state_variable_joint_out.theta_dot;
            chassis->leg_R.vmc.forward_kinematics.Fxy_set_point.E.Tp_set_point += chassis->steer_compensatory_torque;

            chassis->leg_R.vmc.forward_kinematics.Fxy_set_point.E.Fy_set_point = 0.0f;
            chassis->leg_R.vmc.forward_kinematics.Fxy_set_point.E.Fy_set_point = chassis->leg_R.offground_leg_pid.out;
        }
    }


// 计算关节电机力矩
        forward_dynamics(&chassis->leg_L.vmc, chassis_physical_config);
        forward_dynamics(&chassis->leg_R.vmc, chassis_physical_config);

        chassis->leg_L.joint_F_torque = chassis->leg_L.vmc.forward_kinematics.T1_T4_set_point.E.T1_set_point;//F
        chassis->leg_L.joint_B_torque = chassis->leg_L.vmc.forward_kinematics.T1_T4_set_point.E.T4_set_point;//B

        chassis->leg_R.joint_F_torque = chassis->leg_R.vmc.forward_kinematics.T1_T4_set_point.E.T1_set_point;//F
        chassis->leg_R.joint_B_torque = chassis->leg_R.vmc.forward_kinematics.T1_T4_set_point.E.T4_set_point;//B

        // 输出限幅
        VAL_LIMIT(chassis->leg_R.joint_F_torque, MIN_JOINT_TORQUE, MAX_JOINT_TORQUE);
        VAL_LIMIT(chassis->leg_R.joint_B_torque, MIN_JOINT_TORQUE, MAX_JOINT_TORQUE);
        VAL_LIMIT(chassis->leg_L.joint_F_torque, MIN_JOINT_TORQUE, MAX_JOINT_TORQUE);
        VAL_LIMIT(chassis->leg_L.joint_B_torque, MIN_JOINT_TORQUE, MAX_JOINT_TORQUE);

}

// 逆解算腿长变化速度、摆角变化速度
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

// 逆解算出虚拟力矩和沿腿方向支持力
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

// 计算竖直方向支持力
static void fn_cal(Leg *leg, float az, ChassisPhysicalConfig *chassis_physical_config) {

    if (leg == NULL) {
        return;
    }

    // 用逆解算的数据计算
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

    // 更新phi1 phi4
    vmc_phi_update(&chassis.leg_L, &chassis.leg_R);

    // VMC 正运动学解算
    forward_kinematics(&chassis.leg_L, &chassis.leg_R, &chassis_physical_config);

    // 配置输出力矩
    wheel_motors_torque_set(&chassis);
    joint_motors_torque_set(&chassis, &chassis_physical_config);

    // VMC 逆动力学解算
    // 根据关节反馈力矩逆解算出虚拟力(F Tp)
  Inverse_Dynamics(&chassis.leg_L.vmc,
                   (get_joint_motors() + 1)->torque,
                   get_joint_motors()->torque,
                   &chassis_physical_config);
  Inverse_Dynamics(&chassis.leg_R.vmc,
                   -(get_joint_motors() + 2)->torque,
                   -(get_joint_motors() + 3)->torque,
                   &chassis_physical_config);

  // 竖直方向支持力解算
  fn_cal(&chassis.leg_L, chassis.imu_reference.robot_az, &chassis_physical_config);
  fn_cal(&chassis.leg_R, chassis.imu_reference.robot_az, &chassis_physical_config);

//  // 腿部离地检测
//  leg_is_offground(&chassis);
}