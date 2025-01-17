#include "Referee.h"
#include "string.h"
#include "bsp_usart.h"
#include "cmsis_os.h"
#include "Gimbal.h"
#include "Chassis.h"
#include "stm32f4xx_hal_uart.h"
#include "CRC8_CRC16.h"

extern UART_HandleTypeDef huart6;
extern UART_HandleTypeDef huart1;


uint8_t usart6_buf[REFEREE_BUFFER_SIZE]={0};  //缓存从串口接受的数据
uint8_t usart1_buf[REFEREE_BUFFER_SIZE]={0};

Referee_info_t Referee;
uint8_t test_usart;


/*函数和声明*/
static bool Referee_read_data(uint8_t *ReadFromUsart);
/*裁判系统主任务*/

//串口中断函数
void USART6_IRQHandler(void)
{
    static volatile uint8_t res;
    if(USART6->SR & UART_FLAG_IDLE)
    {
        __HAL_UART_CLEAR_PEFLAG(&huart6);//读取UART6-SR 和UART6-DR; 清除中断标志位

        __HAL_DMA_DISABLE(huart6.hdmarx); //使能dma_rx

        Referee_read_data(&usart6_buf[0]);
        test_usart++;

        memset(&usart6_buf[0],0,REFEREE_BUFFER_SIZE);//置0

        __HAL_DMA_CLEAR_FLAG(huart6.hdmarx,DMA_LISR_TCIF1); //清除传输完成标志位

        __HAL_DMA_SET_COUNTER(huart6.hdmarx, REFEREE_BUFFER_SIZE);//设置DMA 搬运数据大小 单位为字节

        __HAL_DMA_ENABLE(huart6.hdmarx); //使能DMARx

    }
}

//图传串口中断函数
void USART1_IRQHandler(void)
{
    static volatile uint8_t res;
    if(USART1->SR & UART_FLAG_IDLE)
    {
        __HAL_UART_CLEAR_PEFLAG(&huart1);//读取UART1-SR 和UART1-DR; 清除中断标志位

        __HAL_DMA_DISABLE(huart1.hdmarx); //使能dma_rx

        Referee_read_data(&usart1_buf[0]);
        test_usart++;
        memset(&usart1_buf[0],0,REFEREE_BUFFER_SIZE);//置0

        __HAL_DMA_CLEAR_FLAG(huart1.hdmarx,DMA_LISR_TCIF1); //清除传输完成标志位

        __HAL_DMA_SET_COUNTER(huart1.hdmarx, REFEREE_BUFFER_SIZE);//设置DMA 搬运数据大小 单位为字节

        __HAL_DMA_ENABLE(huart1.hdmarx); //使能DMARx


    }
}

//根据裁判系统信息判断机器人的ID和对应客户端的ID
void judge_team_client(){
    //本机器人为红方
    if(Referee.GameRobotStat.robot_id<10)
    {
        Referee.ids.teammate_hero 	   = 1;
        Referee.ids.teammate_engineer  = 2;
        Referee.ids.teammate_infantry3 = 3;
        Referee.ids.teammate_infantry4 = 4;
        Referee.ids.teammate_infantry5 = 5;
        Referee.ids.teammate_plane	   = 6;
        Referee.ids.teammate_sentry		= 7;

        Referee.ids.client_hero 	 = 0x0101;
        Referee.ids.client_engineer  = 0x0102;
        Referee.ids.client_infantry3 = 0x0103;
        Referee.ids.client_infantry4 = 0x0104;
        Referee.ids.client_infantry5 = 0x0105;
        Referee.ids.client_plane	 = 0x0106;

        switch (Referee.GameRobotStat.robot_id) {
            case Referee_hero_red:{
                Referee.SelfClient=Referee.ids.client_hero;
            }break;

            case Referee_engineer_red:{
                Referee.SelfClient=Referee.ids.client_engineer;
            }break;

            case Referee_infantry3_red:{
                Referee.SelfClient=Referee.ids.client_infantry3;
            }break;

            case Referee_infantry4_red:{
                Referee.SelfClient=Referee.ids.client_infantry4;
            }break;

            case Referee_infantry5_red:{
                Referee.SelfClient=Referee.ids.client_infantry5;
            }break;

            case Referee_plane_red:{
                Referee.SelfClient=Referee.ids.client_plane;
            }break;

            default:{

            }break;
        }

    }//本机器人为蓝方
    else{
        Referee.ids.teammate_hero 		 	= 101;
        Referee.ids.teammate_engineer  = 102;
        Referee.ids.teammate_infantry3 = 103;
        Referee.ids.teammate_infantry4 = 104;
        Referee.ids.teammate_infantry5 = 105;
        Referee.ids.teammate_plane		 = 106;
        Referee.ids.teammate_sentry		= 107;

        Referee.ids.client_hero 	 = 0x0165;
        Referee.ids.client_engineer  = 0x0166;
        Referee.ids.client_infantry3 = 0x0167;
        Referee.ids.client_infantry4 = 0x0168;
        Referee.ids.client_infantry5 = 0x0169;
        Referee.ids.client_plane	 = 0x016A;

        switch (Referee.GameRobotStat.robot_id) {
            case Referee_hero_blue:{
                Referee.SelfClient=Referee.ids.client_hero;
            }break;

            case Referee_engineer_blue:{
                Referee.SelfClient=Referee.ids.client_engineer;
            }break;

            case Referee_infantry3_blue:{
                Referee.SelfClient=Referee.ids.client_infantry3;
            }break;

            case Referee_infantry4_blue:{
                Referee.SelfClient=Referee.ids.client_infantry4;
            }break;

            case Referee_infantry5_blue:{
                Referee.SelfClient=Referee.ids.client_infantry5;
            }break;

            case Referee_plane_blue:{
                Referee.SelfClient=Referee.ids.client_plane;
            }break;

            default:{

            }break;
        }

    }
}
float all_rpm_mul_current = 0;
float all_current_pingfang = 0;
float power_nihe = 0;
bool Referee_read_data(uint8_t *ReadFromUsart)
{
    int CmdID=0;//数据命令码解析

    uint16_t judge_length;
    Referee.RobotHurt.being_hurt = false;
    if(ReadFromUsart==NULL)
        return 0 ;

    memcpy(&Referee.FrameHeader,ReadFromUsart,Referee_LEN_FRAME_HEAD);

    if(ReadFromUsart[SOF]==REFREE_HEADER_SOF) //判断帧头是否为0xA5
    {
        if(verify_CRC8_check_sum(ReadFromUsart,LEN_HEADER)) //CRC 帧头校验
        {
            judge_length=ReadFromUsart[DATA_LENGTH]+LEN_HEADER+Referee_LEN_CMD_ID+Referee_LEN_FRAME_TAIL;
            if(verify_CRC16_check_sum(ReadFromUsart,judge_length))  //帧尾校验
            {
//                retval_tf=1;//表示数据可用
                CmdID = (ReadFromUsart[6] << 8 | ReadFromUsart[5]);//解析数据命令码,将数据拷贝到相应结构体中(注意拷贝数据的长度)

                switch (CmdID)
                {

                    case Referee_ID_game_state://0x0001 比赛状态 1HZ
                        memcpy(&Referee.GameState,ReadFromUsart+DATA,Referee_LEN_game_state);
                        break;

                    case Referee_ID_game_result://0x0002 比赛结果   比赛结束后发送
                        memcpy(&Referee.GameResult,ReadFromUsart+DATA,Referee_LEN_game_result);
                        Referee.GameResult.game_over = true;
                        break;

                    case Referee_ID_game_robot_HP://0x0003 机器人状态HP   1HZ
                        memcpy(&Referee.GameRobotHP,ReadFromUsart+DATA,Referee_LEN_game_robot_HP);
                        break;

//V1.6.1删除
//                    case Referee_ID_game_dart_state: //0x0004 飞镖发射状态
//                        memcpy(&Referee.GameDartStatus,ReadFromUsart+DATA,Referee_LED_game_missile_state);
//                        break;
//
//                    case Referee_ID_game_buff: //0x0005 ICRA_BUFF状态     1HZ
//                        memcpy(&Referee.GameICRABuff,ReadFromUsart+DATA,Referee_LED_game_buff);
//                        break;

                    case Referee_ID_event_data://0x0101 场地事件数据      1HZ
                        memcpy(&Referee.EventData,ReadFromUsart+DATA,Referee_LEN_event_data);
                        break;

                    case Referee_ID_supply_projectile_action://0x0102 场地补给站动作标识数据   动作改变之后发送
                        memcpy(&Referee.SupplyProjectileAction,ReadFromUsart+DATA,Referee_LEN_supply_projectile_action);
                        break;

                    case Referee_ID_supply_warm://0x0104    裁判系统警告数据    己方警告之后发送
                        memcpy(&Referee.RefereeWarning,ReadFromUsart+DATA,Referee_LEN_supply_warm);
                        break;

                    case Referee_ID_dart_info://0x0105    飞镖发射口倒计时    1HZ
                        memcpy(&Referee.DartRemainingTime,ReadFromUsart+DATA,Referee_LEN_dart_info);
                        break;

                    case Referee_ID_game_robot_state://0x0201   机器人状态数据     10HZ
                        memcpy(&Referee.GameRobotStat,ReadFromUsart+DATA,Referee_LEN_game_robot_state);
                        judge_team_client();//判断一下机器人所属的队伍和类型 以及对应的机械人id和客户端id
                        break;

                    case Referee_ID_power_heat_data://0x0202    实时功率热量数据    50HZ
                        memcpy(&Referee.PowerHeatData,ReadFromUsart+DATA,Referee_LEN_power_heat_data);
                        //测量功率模型用，为使测量值测量频率和裁判系统回报的功率频率一致
                        //收集数据
//                        float tmp1= 0,tmp2 = 0;
//                        for (int i = 0; i < 4; ++i) {
////                            float filtercurrent= first_Kalman_Filter(&chassis_filter[i],chassis.motor_chassis[i].motor_measure->given_current);
//                            tmp1 += chassis.motor_chassis[i].motor_measure->given_current*chassis.motor_chassis[i].motor_measure->given_current;
//                            tmp2 += chassis.motor_chassis[i].motor_measure->speed_rpm*chassis.motor_chassis[i].motor_measure->given_current;
//                            //   tmp1+=pow(filtercurrent*20/16384.0,2);
//                            //  tmp2+=filtercurrent*20/16384.0*chassis.motor_chassis[i].motor_measure->speed_rpm;
//
//                        }
//                        all_current_pingfang = tmp1*20.0/16384*20/16384;//反馈电流值转国际单位/A
//                        all_rpm_mul_current = tmp2*20.0/16384;
//                        power_nihe = CHASSIS_POWER_R0*tmp1 + CHASSIS_POWER_K0*tmp2 + CHASSIS_POWER_P0;
//                        //          power_nihe = 0.000002623f*tmp2 + 0.0000001025f*tmp1 + 3.067f;
//
//                        if(power_nihe < 0)
//                            power_nihe = 0;
                        break;

                    case Referee_ID_game_robot_pos://0x0203     机器人位置数据     10HZ
                        memcpy(&Referee.GameRobotPos,ReadFromUsart+DATA,Referee_LEN_game_robot_pos);
                        break;

                    case Referee_ID_buff_musk://0x0204  机器人增益数据     1HZ
                        memcpy(&Referee.Buff,ReadFromUsart+DATA,Referee_LEN_buff_musk);
                        break;

                    case Referee_ID_aerial_robot_energy://0x0205    空中机器人能量状态数据 10HZ
                        memcpy(&Referee.AerialRobotEnergy,ReadFromUsart+DATA,Referee_LEN_aerial_robot_energy);
                        break;

                    case Referee_ID_robot_hurt://0x0206     伤害状态数据  伤害发生后发送
                        memcpy(&Referee.RobotHurt,ReadFromUsart+DATA,Referee_LEN_robot_hurt);
                        Referee.RobotHurt.being_hurt = true;//受击判断
                        break;

                    case Referee_ID_shoot_data://0x0207     实时射击数据  射击后发送
                        memcpy(&Referee.ShootData,ReadFromUsart+DATA,Referee_LEN_shoot_data);
                        break;

                    case Referee_ID_bullet_remaining://0x0208   剩余发射数   10HZ周期发送
                        memcpy(&Referee.BulletRemaining,ReadFromUsart+DATA,Referee_LEN_bullet_remaining);
                        break;

                    case Referee_ID_rfid_status://0x0209    机器人RFID状态，1Hz
                        memcpy(&Referee.RfidStatus,ReadFromUsart+DATA,Referee_LEN_rfid_status);
                        break;

                    case Referee_ID_dart_client_directive://0x020A  飞镖机器人客户端指令书, 10Hz
                        memcpy(&Referee.DartClient,ReadFromUsart+DATA,Referee_LEN_dart_client_directive);
                        break;

                    case Referee_ID_dart_all_robot_position://0x020B
                        memcpy(&Referee.RobotPosition,ReadFromUsart+DATA,Referee_LEN_dart_all_robot_position);
                        break;

                    case Referee_ID_radar_mark://0x020C
                        memcpy(&Referee.RadarMark,ReadFromUsart+DATA,Referee_LEN_radar_mark);
                        break;

                    case Referee_ID_entry_info://0x020D
                        memcpy(&Referee.SentryInfo,ReadFromUsart+DATA,Referee_LEN_entry_info);
                        break;

                    case Referee_ID_radar_info://0x020E
                        memcpy(&Referee.RadarInfo,ReadFromUsart+DATA,Referee_LEN_radar_info);
                        break;

                    case Referee_ID_robot_interactive_header_data://0x0301
                        memcpy(&Referee.StudentInteractive,ReadFromUsart+DATA,Referee_LEN_robot_interactive_header_data);
                        break;

                    case Referee_ID_map_command://0x0303
                        memcpy(&Referee.MapCommand,ReadFromUsart+DATA,Referee_LEN_map_command);
                        break;

                    case Referee_ID_keyboard_information://0x0304
                        memcpy(&Referee.keyboard,ReadFromUsart+DATA,Referee_LEN_keyboard_information);
                        break;

                    case Referee_ID_robot_map_robot_data://0x0305
                        memcpy(&Referee.EnemyPosition,ReadFromUsart+DATA,Referee_LEN_robot_map_robot_data);
                        break;

                    case Referee_ID_robot_custom_client://0x0306
                        memcpy(&Referee.Custom,ReadFromUsart+DATA,Referee_LEN_robot_custom_client);
                        break;

                    case Referee_ID_robot_entry_info_receive://0x0307
                        memcpy(&Referee.SentryMapData,ReadFromUsart+DATA,Referee_LEN_robot_entry_info_receive);
                        break;

                    case Referee_ID_robot_custom_info_receive://0x0308
                        memcpy(&Referee.SendData,ReadFromUsart+DATA,Referee_LEN_robot_custom_info_receive);
                        break;

                    default:
                        break;
                }
            }
        }
        if(*(ReadFromUsart + sizeof(frame_header_struct_t) + Referee_LEN_CMD_ID + Referee.FrameHeader.data_length +Referee_LEN_FRAME_TAIL) == 0xA5)
        {
            //如果一个数据包出现了多帧数据,则再次读取
            Referee_read_data(ReadFromUsart + sizeof(frame_header_struct_t) + Referee_LEN_CMD_ID + Referee.FrameHeader.data_length+ Referee_LEN_FRAME_TAIL);
        }
    }
}
