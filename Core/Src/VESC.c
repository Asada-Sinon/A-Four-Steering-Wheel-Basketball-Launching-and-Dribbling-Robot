#include "fdcan.h"
#include "stm32h7xx_hal_fdcan.h"
#include "VESC.h"
/**
 * @brief: 进行VESC配置
 */
void VESC_CAN_Send(uint32_t id, const uint8_t *data, uint8_t len, FDCAN_HandleTypeDef *hcan)
{
	int i;
	FDCAN_TxHeaderTypeDef TxMessage;
	if (len > 8)
		len = 8;

	TxMessage.IdType = FDCAN_EXTENDED_ID;
	TxMessage.DataLength = FDCAN_DLC_BYTES_4;
	TxMessage.Identifier = id;
	TxMessage.TxFrameType = FDCAN_DATA_FRAME;
	TxMessage.ErrorStateIndicator = FDCAN_ESI_ACTIVE;
	TxMessage.BitRateSwitch = FDCAN_BRS_OFF;
	TxMessage.FDFormat = FDCAN_CLASSIC_CAN;
	TxMessage.TxEventFifoControl = FDCAN_NO_TX_EVENTS;
	TxMessage.MessageMarker = 0;

	uint8_t mbox[8];
	for (i = 0; i < len; i++)
		mbox[i] = data[i];

	i = 0;
	while ((HAL_FDCAN_AddMessageToTxFifoQ(hcan, &TxMessage, mbox) != HAL_OK) && (i < 0XFF))
		i++;
}

// 设置转速
void VESC_Set_RPM(uint32_t id, int32_t RPM, FDCAN_HandleTypeDef *hcan)
{
	uint8_t mes[4];
	int i = 0;
	unsigned char *hex = (unsigned char *)&RPM;
	for (i = 0; i < 4; i++)
	{
		mes[i] = hex[3 - i];
	}
	VESC_CAN_Send(id | ((uint32_t)CAN_PACKET_SET_RPM << 8), mes, 4, hcan);
}

// 制动
void VESC_Set_Break_Current(uint32_t id, float current, FDCAN_HandleTypeDef *hcan)
{
	uint8_t mes[4];
	int i = 0;
	uint32_t current_int = current * 1000;
	unsigned char *hex = (unsigned char *)&current_int;
	for (i = 0; i < 4; i++)
	{
		mes[i] = hex[3 - i];
	}
	VESC_CAN_Send(id | ((uint32_t)CAN_PACKET_SET_CURRENT_HANDBRAKE << 8), mes, 4, hcan);
}

VESC_STA VESC[4];
// void VESC_CAN_Receive(FDCAN_RxHeaderTypeDef RxHeader, uint8_t *CanReceiveData)
// {
// 	int id;
// 	int cmd;
// 	id = (RxHeader.Identifier & 0xFF); // 截取ID
// 	cmd = (RxHeader.Identifier >> 8);  // 截取命令
// 	if (RxHeader.IdType == FDCAN_EXTENDED_ID && (id == 1 || id == 2 || id == 3 || id == 4))
// 	{
// 		switch (cmd)
// 		{ // cmd判断
// 		case CAN_PACKET_STATUS_1:
// 			VESC[id].vesc_msg_1.rx_time = 0;
// 			VESC[id].vesc_msg_1.rpm = (CanReceiveData[0] << 24 | CanReceiveData[1] << 16 | CanReceiveData[2] << 8 | CanReceiveData[3]);
// 			VESC[id].vesc_msg_1.current = ((int16_t)(CanReceiveData[4] << 8 | CanReceiveData[5])) / 10.0f;
// 			VESC[id].vesc_msg_1.duty = ((int16_t)(CanReceiveData[6] << 8 | CanReceiveData[7])) / 1000.0f;
// 			break;
// 		case CAN_PACKET_STATUS_2:
// 			VESC[id].vesc_msg_2.rx_time = 0;
// 			VESC[id].vesc_msg_2.amp_hours = ((int32_t)(CanReceiveData[0] << 24 | CanReceiveData[1] << 16 | CanReceiveData[2] << 8 | CanReceiveData[3])) / 10000.0f;
// 			VESC[id].vesc_msg_2.amp_hours_charged = ((int32_t)(CanReceiveData[4] << 24 | CanReceiveData[5] << 16 | CanReceiveData[6] << 8 | CanReceiveData[7])) / 10000.0f;
// 			break;
// 		case CAN_PACKET_STATUS_3:
// 			VESC[id].vesc_msg_3.rx_time = 0;
// 			VESC[id].vesc_msg_3.watt_hours = ((int32_t)(CanReceiveData[0] << 24 | CanReceiveData[1] << 16 | CanReceiveData[2] << 8 | CanReceiveData[3])) / 10000.0f;
// 			VESC[id].vesc_msg_3.watt_hours_charged = ((int32_t)(CanReceiveData[4] << 24 | CanReceiveData[5] << 16 | CanReceiveData[6] << 8 | CanReceiveData[7])) / 10000.0f;
// 			break;
// 		case CAN_PACKET_STATUS_4:
// 			VESC[id].vesc_msg_4.rx_time = 0;
// 			VESC[id].vesc_msg_4.temp_fet = ((int16_t)(CanReceiveData[0] << 8 | CanReceiveData[1])) / 10.0f;
// 			VESC[id].vesc_msg_4.temp_motor = ((int16_t)(CanReceiveData[2] << 8 | CanReceiveData[3])) / 10.0f;
// 			VESC[id].vesc_msg_4.current_in = ((int16_t)(CanReceiveData[4] << 8 | CanReceiveData[5])) / 10.0f;
// 			VESC[id].vesc_msg_4.pid_pos_now = ((int16_t)(CanReceiveData[6] << 8 | CanReceiveData[7])) / 50.0f;
// 			break;
// 		case CAN_PACKET_STATUS_5:
// 			VESC[id].vesc_msg_5.rx_time = 0;
// 			VESC[id].vesc_msg_5.tacho_value = (CanReceiveData[0] << 24 | CanReceiveData[1] << 16 | CanReceiveData[2] << 8 | CanReceiveData[3]);
// 			VESC[id].vesc_msg_5.v_input = ((int16_t)(CanReceiveData[4] << 8 | CanReceiveData[5])) / 10.0f;
// 			VESC[id].vesc_msg_5.reserved = (CanReceiveData[6] << 8 | CanReceiveData[7]);
// 			break;
// 		default:
// 			break;
// 		}
// 	}
// }
