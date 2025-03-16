#include "cap.h"
#include "can.h"
#include "cmsis_os.h"

cap_data_t Cap;
static CAN_TxHeaderTypeDef tx_message;

uint8_t cap_update_flag = 0;
uint8_t cap_mode = CAP_MODE_SILENT;
int32_t cap_percentage;

//This is the initialization frame, which is received once after the controller is powered on
void cap_init(uint8_t cap_mode_)
{
    uint8_t can_send_data[8];
    tx_message.StdId = CAP_INIT_ID;
    tx_message.IDE = CAN_ID_STD;
    tx_message.RTR = CAN_RTR_DATA;
    tx_message.DLC = 0x01;
    can_send_data[0] = cap_mode_;

    if(HAL_CAN_AddTxMessage(&hcan2, &tx_message, can_send_data, (uint32_t*)CAN_TX_MAILBOX0) != HAL_OK)
    {
        if(HAL_CAN_AddTxMessage(&hcan2, &tx_message, can_send_data, (uint32_t*)CAN_TX_MAILBOX1) != HAL_OK)
        {
            HAL_CAN_AddTxMessage(&hcan2, &tx_message, can_send_data, (uint32_t*)CAN_TX_MAILBOX2);
        }
    }
}

//Control frames,Exceed模式ON
void cap_control_ExceedOn(uint8_t pb_set, uint8_t cap_mode)
{
    uint8_t can_send_data[8];
    tx_message.StdId = CAP_CONTROL_ID;
    tx_message.IDE = CAN_ID_STD;
    tx_message.RTR = CAN_RTR_DATA;
    tx_message.DLC = 0x03;
    can_send_data[0] = pb_set;
    can_send_data[1] = CAP_EXCEED_MODE_ENABLE;
    can_send_data[2] = cap_mode;

    if(HAL_CAN_AddTxMessage(&hcan2, &tx_message, can_send_data, (uint32_t*)CAN_TX_MAILBOX0) != HAL_OK)
    {
        if(HAL_CAN_AddTxMessage(&hcan2, &tx_message, can_send_data, (uint32_t*)CAN_TX_MAILBOX1) != HAL_OK)
        {
            HAL_CAN_AddTxMessage(&hcan2, &tx_message, can_send_data, (uint32_t*)CAN_TX_MAILBOX2);
        }
    }
}

//Control frames，Exceed模式off
void cap_control_ExceedOff(uint8_t pb_set, uint8_t cap_mode)
{
    uint8_t can_send_data[8];
    tx_message.StdId = CAP_CONTROL_ID;
    tx_message.IDE = CAN_ID_STD;
    tx_message.RTR = CAN_RTR_DATA;
    tx_message.DLC = 0x03;
    can_send_data[0] = pb_set;
    can_send_data[1] = CAP_EXCEED_MODE_DISABLE;
    can_send_data[2] = cap_mode;

    if(HAL_CAN_AddTxMessage(&hcan2, &tx_message, can_send_data, (uint32_t*)CAN_TX_MAILBOX0) != HAL_OK)
    {
        if(HAL_CAN_AddTxMessage(&hcan2, &tx_message, can_send_data, (uint32_t*)CAN_TX_MAILBOX1) != HAL_OK)
        {
            HAL_CAN_AddTxMessage(&hcan2, &tx_message, can_send_data, (uint32_t*)CAN_TX_MAILBOX2);
        }
    }
}

void cap_test_loading()
{
    uint8_t can_send_data[8];
    tx_message.StdId = 0x06;
    tx_message.IDE = CAN_ID_STD;
    tx_message.RTR = CAN_RTR_DATA;
    tx_message.DLC = 0x08;
    can_send_data[0] = 0;
    can_send_data[1] = 0;
    can_send_data[2] = 0;
    can_send_data[3] = 0;
    can_send_data[4] = 0;
    can_send_data[5] = 0;
    can_send_data[6] = 0;
    can_send_data[7] = 0;

    if(HAL_CAN_AddTxMessage(&hcan2, &tx_message, can_send_data, (uint32_t*)CAN_TX_MAILBOX0) != HAL_OK)
    {
        if(HAL_CAN_AddTxMessage(&hcan2, &tx_message, can_send_data, (uint32_t*)CAN_TX_MAILBOX1) != HAL_OK)
        {
            HAL_CAN_AddTxMessage(&hcan2, &tx_message, can_send_data, (uint32_t*)CAN_TX_MAILBOX2);
        }
    }
}

void cap_can_msg_unpack(uint32_t id, uint8_t data[]) {
    switch (id) {
        case CAP_ERR_FEEDBACK_ID:
        {
            Cap.err_state[CAP_FIRMWARE_ERR] = data[CAP_FIRMWARE_ERR];
            Cap.err_state[CAP_CAN_ERR] = data[CAP_CAN_ERR];
            Cap.err_state[CAP_TEMP_ERR] = data[CAP_TEMP_ERR];
            Cap.err_state[CAP_CALI_ERR] = data[CAP_CALI_ERR];
            Cap.err_state[CAP_VOLT_ERR] = data[CAP_VOLT_ERR];
            Cap.err_state[CAP_CURR_ERR] = data[CAP_CURR_ERR];
            Cap.err_state[CAP_POWER_ERR] = data[CAP_POWER_ERR];
            Cap.err_state[CAP_SAMP_ERR] = data[CAP_SAMP_ERR];
            if(Cap.can_init_state != CAP_INIT_FINISHED)
            {
                Cap.can_init_state = CAP_INIT_FINISHED;
            }
            break;
        }
        case CAP_INFO_FEEDBACK_ID://feedback frames
        {
            Cap.capFeedback.esr_v = (uint16_t)((data[0] << 8) | data[1]);//ESR修正后的电容组电压
            Cap.capFeedback.work_s1 = data[2];      //工作强度1
            Cap.capFeedback.work_s2 = data[3];      //工作强度2
            Cap.capFeedback.input_power = (uint16_t)((data[4] << 8) | data[5]);//电源输入功率
            cap_percentage = Cap.capFeedback.esr_v/2800;
            if(Cap.can_init_state != CAP_INIT_FINISHED)
            {
                Cap.can_init_state = CAP_INIT_FINISHED;
            }
            break;
        }
        case CAP_INIT_FEEDBACK_ID://ready frames
        {
            Cap.can_init_state = data[0];
//          hcan->ErrorCode = HAL_CAN_ERROR_NONE; //don't understand
            break;
        }
        default:{
            break;
        }
    }
}
