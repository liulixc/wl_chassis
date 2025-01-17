#include "vx_kalman_filter.h"
#include "robot_def.h"
#include "wheel.h"
#include "joint.h"
#include "cmsis_os.h"

extern Chassis chassis;
extern ChassisPhysicalConfig chassis_physical_config;

KalmanFilter_t vaEstimateKF;	   // �������˲����ṹ��

float vel_acc[2]; // ����ٶ�����ٶ��ںϺ�Ľ��

float vaEstimateKF_F[4] = {1.0f, 0.005f,
                           0.0f, 1.0f};	   // ״̬ת�ƾ��󣬿�������Ϊ0.001s

float vaEstimateKF_P[4] = {1.0f, 0.0f,
                           0.0f, 1.0f};    // �������Э�����ʼֵ

float vaEstimateKF_Q[4] = {0.000025f, 0.005f,
                           0.005f, 1.0f};    // Q�����ʼֵ���������ֵ��������

float vaEstimateKF_R[4] = {0.0025f, 0.0f,
                           0.0f,  1.0f}; 	//200��200Ϊ������������

const float vaEstimateKF_H[4] = {1.0f, 0.0f,
                                 0.0f, 1.0f};	// ���þ���HΪ����


void xvEstimateKF_Init(KalmanFilter_t *EstimateKF)//��ʼ���������ṹ�壬���Ѹÿ�ͷ����ľ����Ƶ��ṹ���еľ���
{
    Kalman_Filter_Init(EstimateKF, 2, 0, 2);	// ״̬����2ά û�п����� ��������2ά

    memcpy(EstimateKF->F_data, vaEstimateKF_F, sizeof(vaEstimateKF_F));
    memcpy(EstimateKF->P_data, vaEstimateKF_P, sizeof(vaEstimateKF_P));
    memcpy(EstimateKF->Q_data, vaEstimateKF_Q, sizeof(vaEstimateKF_Q));
    memcpy(EstimateKF->R_data, vaEstimateKF_R, sizeof(vaEstimateKF_R));
    memcpy(EstimateKF->H_data, vaEstimateKF_H, sizeof(vaEstimateKF_H));

}

static void xvEstimateKF_Update(KalmanFilter_t *EstimateKF ,float acc,float vel)
{
    //��Q��R�����Ƶ��ṹ���еľ���
    memcpy(EstimateKF->Q_data, vaEstimateKF_Q, sizeof(vaEstimateKF_Q));
    memcpy(EstimateKF->R_data, vaEstimateKF_R, sizeof(vaEstimateKF_R));

    //�������˲�������ֵ����
    EstimateKF->MeasuredVector[0] =	vel;//�����ٶ�
    EstimateKF->MeasuredVector[1] = acc;//�������ٶ�

    //�������˲������º���
    Kalman_Filter_Update(EstimateKF);

    // ��ȡ����ֵ
    for (uint8_t i = 0; i < 2; i++)
    {
        vel_acc[i] = EstimateKF->FilteredValue[i];
    }
}

void chassis_vx_kalman_run(void)
{

    static float w_l,w_r=0.0f;//���������ֵĽ��ٶ�
    static float v_lb,v_rb=0.0f;//ͨ����������������Ļ����ٶ�
    static float aver_v=0.0f;//ͨ��ȡƽ������������ٶ�

    // ���������ת����Դ�ؽ��ٶȣ����ﶨ�����˳ʱ��Ϊ��
    w_l = -get_wheel_motors()->angular_vel + chassis.imu_reference.pitch_gyro + chassis.leg_L.vmc.forward_kinematics.d_alpha;
    // �������ڻ���(bϵ)���ٶ�
    v_lb = w_l * chassis_physical_config.wheel_radius + chassis.leg_L.vmc.forward_kinematics.fk_L0.L0 * chassis.leg_L.state_variable_feedback.theta_dot * arm_cos_f32(chassis.leg_L.state_variable_feedback.theta) + chassis.leg_L.vmc.forward_kinematics.fk_L0.L0_dot * arm_sin_f32(chassis.leg_L.state_variable_feedback.theta);

    // �ұ�������ת����Դ�ؽ��ٶȣ����ﶨ�����˳ʱ��Ϊ��
    w_r = -(get_wheel_motors() + 1)->angular_vel - chassis.imu_reference.pitch_gyro + chassis.leg_R.vmc.forward_kinematics.d_alpha;
    // �������ڻ���(bϵ)���ٶ�
    v_rb = w_r * chassis_physical_config.wheel_radius + chassis.leg_R.vmc.forward_kinematics.fk_L0.L0 * chassis.leg_R.state_variable_feedback.theta_dot * arm_cos_f32(chassis.leg_R.state_variable_feedback.theta) + chassis.leg_R.vmc.forward_kinematics.fk_L0.L0_dot * arm_sin_f32(chassis.leg_R.state_variable_feedback.theta);

    aver_v = (v_rb - v_lb) / 2.0f;//ȡƽ��

    xvEstimateKF_Update(&vaEstimateKF,chassis.imu_reference.ax_filtered,aver_v);//���ϸ��¿������˲��еĸ������

}


