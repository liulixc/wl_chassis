#include <stm32f4xx_hal.h>
#include <stm32f4xx_hal_can.h>
#include "can_device.h"
#include "joint.h"
#include "wheel.h"
#include "gimbal.h"

extern CAN_HandleTypeDef hcan1;
extern CAN_HandleTypeDef hcan2;

void can_filter_init(void) {
  CAN_FilterTypeDef can_filter_st;
  can_filter_st.FilterActivation = ENABLE;
  can_filter_st.FilterMode = CAN_FILTERMODE_IDMASK;
  can_filter_st.FilterScale = CAN_FILTERSCALE_32BIT;
  can_filter_st.FilterIdHigh = 0x0000;
  can_filter_st.FilterIdLow = 0x0000;
  can_filter_st.FilterMaskIdHigh = 0x0000;
  can_filter_st.FilterMaskIdLow = 0x0000;
  can_filter_st.FilterBank = 0;
  can_filter_st.FilterFIFOAssignment = CAN_RX_FIFO0;
  HAL_CAN_ConfigFilter(&hcan1, &can_filter_st);
  HAL_CAN_Start(&hcan1);
  HAL_CAN_ActivateNotification(&hcan1, CAN_IT_RX_FIFO0_MSG_PENDING);

  can_filter_st.SlaveStartFilterBank = 14;
  can_filter_st.FilterBank = 14;
  HAL_CAN_ConfigFilter(&hcan2, &can_filter_st);
  HAL_CAN_Start(&hcan2);
  HAL_CAN_ActivateNotification(&hcan2, CAN_IT_RX_FIFO0_MSG_PENDING);
}

//**对于STM32的HAL库和CAN外设，邮箱不为空的条件通常是通过检查传输状态寄存器（TSR）中的特定位来确定的。
// 具体来说，每个邮箱都有一个对应的“传输邮箱空”（Transmit Mailbox Empty，TME）位，当该位为0时，表示邮箱不为空；当该位为1时，表示邮箱为空且可用。

/************* 寻找不为空的邮箱 ***************/
uint32_t get_can1_free_mailbox() {
  if ((hcan1.Instance->TSR & CAN_TSR_TME0) != RESET){
    return CAN_TX_MAILBOX0;
  }
  else if ((hcan1.Instance->TSR & CAN_TSR_TME1) != RESET){
      return CAN_TX_MAILBOX1;
  }
  else if ((hcan1.Instance->TSR & CAN_TSR_TME2) != RESET){
      return CAN_TX_MAILBOX2;
  }
  else{
      return 0;
  }
}

uint32_t get_can2_free_mailbox() {
  if ((hcan2.Instance->TSR & CAN_TSR_TME0) != RESET) {
    return CAN_TX_MAILBOX0;
  }
  else if ((hcan2.Instance->TSR & CAN_TSR_TME1) != RESET) {
      return CAN_TX_MAILBOX1;
  }
  else if ((hcan2.Instance->TSR & CAN_TSR_TME2) != RESET) {
      return CAN_TX_MAILBOX2;
  }
  else {
      return 0;
  }
}

/**********************************************/

void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan) {
  CAN_RxHeaderTypeDef rx_header;
  uint8_t rx_data[8]={0};

  if (hcan == &hcan1) {
    if (HAL_CAN_GetRxMessage(hcan, CAN_RX_FIFO0, &rx_header, rx_data) == HAL_OK) {
      lk9025_can_msg_unpack(rx_header.StdId, rx_data);
      gimbal_msg_unpack(rx_header.StdId, rx_data);
    }
  } else if (hcan == &hcan2) {
    if (HAL_CAN_GetRxMessage(hcan, CAN_RX_FIFO0, &rx_header, rx_data) == HAL_OK) {
      dm8009_can_msg_unpack(rx_header.StdId, rx_data);

    }
  }
}