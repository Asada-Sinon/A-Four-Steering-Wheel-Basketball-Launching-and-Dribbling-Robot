#include <stdint.h>
#include "fdcan.h"
#include "stm32h7xx_hal_fdcan.h"
#include "VESC_NEW.h"

// Implementation for sending extended ID CAN-frames
/**
 ***************************************************************************
 ** -------------------------------------------------------------------- **
 ** @name          : can_transmit_eid
 ** @brief         : ����һ����չID��CAN��Ϣ
 ** @param      id : CAN��Ϣ����չID
 ** @param    data : ָ����������ݵ�ָ��
 ** @param     len : ���������ݵĳ���
 ** @param    hcan : FDCAN���ָ��
 ** @retval        : ��
 ** @author        : Ru_yl
 ** -------------------------------------------------------------------- **
 ***************************************************************************
 **/
void can_transmit_eid(uint32_t id, const uint8_t *data, uint8_t len, FDCAN_HandleTypeDef *hcan)
{
	int i;
	FDCAN_TxHeaderTypeDef TxMessage;
	if (len > 8) len = 8;

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
	for (i = 0; i < len; i++) mbox[i] = data[i];

	i = 0;
	while ((HAL_FDCAN_AddMessageToTxFifoQ(hcan, &TxMessage, mbox) != HAL_OK) && (i < 0XFF)) i++;
}

/**
 ***************************************************************************
 ** -------------------------------------------------------------------- **
 ** @name          : buffer_append_int16
 ** @brief         : ��һ��16λ������ӵ�������
 ** @param  buffer : ָ�򻺳�����ָ��
 ** @param  number : Ҫ��ӵ�16λ����
 ** @retval        : None
 ** @author        : Ru_yl
 ** -------------------------------------------------------------------- **
 ***************************************************************************
 **/
void buffer_append_int16(uint8_t *buffer, int16_t number, int32_t *index)
{
	buffer[(*index)++] = number >> 8;
	buffer[(*index)++] = number;
}

/**
***************************************************************************
** -------------------------------------------------------------------- **
** @name          : buffer_append_int32
** @brief         : ���32λ����
** @param  buffer : ������
** @param  number : Ҫ��ӵ�32λ����
** @param  index  : ����������
** @retval        : None
** @author        : Ru_yl
** -------------------------------------------------------------------- **
***************************************************************************
**/
void buffer_append_int32(uint8_t *buffer, int32_t number, int32_t *index)
{
	buffer[(*index)++] = number >> 24;
	buffer[(*index)++] = number >> 16;
	buffer[(*index)++] = number >> 8;
	buffer[(*index)++] = number;
}

/**
 ***************************************************************************
 ** -------------------------------------------------------------------- **
 ** @name          : buffer_append_float16
 ** @brief         : ��һ��16λ��������ӵ�������
 ** @param  buffer : ָ�򻺳�����ָ��
 ** @param  number : Ҫ��ӵ�16λ������
 ** @param  scale  : ��������
 ** @retval        : None
 ** @author        : Ru_yl
 ** -------------------------------------------------------------------- **
 ***************************************************************************
 **/
void buffer_append_float16(uint8_t *buffer, float number, float scale, int32_t *index)
{
	buffer_append_int16(buffer, (int16_t)(number * scale), index);
}

/**
 ***************************************************************************
 ** -------------------------------------------------------------------- **
 ** @name          : buffer_append_float32
 ** @brief         : ��һ��32λ��������ӵ�������
 ** @param  buffer : ָ�򻺳�����ָ��
 ** @param  number : Ҫ��ӵ�32λ������
 ** @param  scale  : ��������
 ** @retval        : None
 ** @author        : Ru_yl
 ** -------------------------------------------------------------------- **
 ***************************************************************************
 **/
void buffer_append_float32(uint8_t *buffer, float number, float scale, int32_t *index)
{
	buffer_append_int32(buffer, (int32_t)(number * scale), index);
}

/**
 ***************************************************************************
 ** -------------------------------------------------------------------- **
 ** @name          		  : vesc_can_set_duty
 ** @brief         		  : ���õ����ռ�ձ�
 ** @param  controller_id : ������ID
 ** @param  duty          : ռ�ձȣ�0-100��
 ** @param  hcan          : FDCAN���ָ��
 ** @retval        		  : None
 ** @author        		  : Ru_yl
 ** -------------------------------------------------------------------- **
 ***************************************************************************
 **/
void vesc_can_set_duty(uint8_t controller_id, float duty, FDCAN_HandleTypeDef *hcan)
{
	int32_t send_index = 0;
	uint8_t buffer[4];
	buffer_append_int32(buffer, (int32_t)(duty * 100000.0f), &send_index);
	can_transmit_eid(controller_id | ((uint32_t)CAN_PACKET_SET_DUTY << 8), buffer, send_index, hcan);
}

/**
 ***************************************************************************
 ** -------------------------------------------------------------------- **
 ** @name          		  : vesc_can_set_current
 ** @brief         		  : ���õ���ĵ���
 ** @param  controller_id : ������ID
 ** @param  current       : ��������λ��A��
 ** @param  hcan          : FDCAN���ָ��
 ** @retval        		  : None
 ** @author        		  : Ru_yl
 ** -------------------------------------------------------------------- **
 ***************************************************************************
 **/
void vesc_can_set_current(uint8_t controller_id, float current, FDCAN_HandleTypeDef *hcan)
{
	int32_t send_index = 0;
	uint8_t buffer[4];
	buffer_append_int32(buffer, (int32_t)(current * 1000.0f), &send_index);
	can_transmit_eid(controller_id | ((uint32_t)CAN_PACKET_SET_CURRENT << 8), buffer, send_index, hcan);
}


/**
 ***************************************************************************
 ** -------------------------------------------------------------------- **
 ** @name          		  : vesc_can_set_current_off_delay
 ** @brief         		  : ���õ���ĵ����͹ر��ӳ�
 ** @param  controller_id : ������ID
 ** @param  current       : ��������λ��A��
 ** @param  off_delay     : �ر��ӳ٣���λ���룩
 ** @param  hcan          : FDCAN���ָ��
 ** @retval        		  : None
 ** @author        		  : Ru_yl
 ** -------------------------------------------------------------------- **
 ***************************************************************************
 **/
void vesc_can_set_current_off_delay(uint8_t controller_id, float current, float off_delay, FDCAN_HandleTypeDef *hcan)
{
	int32_t send_index = 0;
	uint8_t buffer[6];
	buffer_append_int32(buffer, (int32_t)(current * 1000.0f), &send_index);
	buffer_append_float16(buffer, off_delay, 1e3, &send_index);
	can_transmit_eid(controller_id | ((uint32_t)CAN_PACKET_SET_CURRENT << 8), buffer, send_index, hcan);
}

/**
 ***************************************************************************
 ** -------------------------------------------------------------------- **
 ** @name          		  : vesc_can_set_current_brake
 ** @brief         		  : ���õ����ɲ������
 ** @param  controller_id : ������ID
 ** @param  current       : ɲ����������λ��A��
 ** @param  hcan          : FDCAN���ָ��
 ** @retval        		  : None
 ** @author        		  : Ru_yl
 ** -------------------------------------------------------------------- **
 ***************************************************************************
 **/
void vesc_can_set_current_brake(uint8_t controller_id, float current, FDCAN_HandleTypeDef *hcan)
{
	int32_t send_index = 0;
	uint8_t buffer[4];
	buffer_append_float32(buffer, current, 1e3, &send_index);
	can_transmit_eid(controller_id | ((uint32_t)CAN_PACKET_SET_CURRENT_BRAKE << 8), buffer, send_index, hcan);
}

/**
 ***************************************************************************
 ** -------------------------------------------------------------------- **
 ** @name          		  : vesc_can_set_current_brake_rel
 ** @brief         		  : ���õ����ɲ���������ֵ
 ** @param  controller_id : ������ID
 ** @param  current_rel   : ɲ���������ֵ����λ��A��
 ** @param  hcan          : FDCAN���ָ��
 ** @retval        		  : None
 ** @author        		  : Ru_yl
 ** -------------------------------------------------------------------- **
 ***************************************************************************
 **/
void vesc_can_set_current_brake_rel(uint8_t controller_id, float current_rel, FDCAN_HandleTypeDef *hcan)
{
	int32_t send_index = 0;
	uint8_t buffer[4];
	buffer_append_float32(buffer, current_rel, 1e5, &send_index);
	can_transmit_eid(controller_id | ((uint32_t)CAN_PACKET_SET_CURRENT_BRAKE_REL << 8), buffer, send_index, hcan);
}


/**
 ***************************************************************************
 ** -------------------------------------------------------------------- **
 ** @name          		  : vesc_can_set_rpm
 ** @brief         		  : ���õ����ת��
 ** @param  controller_id : ������ID
 ** @param  rpm           : ת�٣���λ��RPM��
 ** @param  hcan          : FDCAN���ָ��
 ** @retval        		  : None
 ** @author        		  : Ru_yl
 ** -------------------------------------------------------------------- **
 ***************************************************************************
 **/
void vesc_can_set_rpm(uint8_t controller_id, float rpm, FDCAN_HandleTypeDef *hcan)
{
	int32_t send_index = 0;
	uint8_t buffer[4];
	buffer_append_int32(buffer, (int32_t)rpm, &send_index);
	can_transmit_eid(controller_id | ((uint32_t)CAN_PACKET_SET_RPM << 8), buffer, send_index, hcan);
}


/**
 ***************************************************************************
 ** -------------------------------------------------------------------- **
 ** @name          		  : vesc_can_set_pos
 ** @brief         		  : ���õ����λ��
 ** @param  controller_id : ������ID
 ** @param  pos           : λ�ã���λ�����ȣ�
 ** @param  hcan          : FDCAN���ָ��
 ** @retval        		  : None
 ** @author        		  : Ru_yl
 ** -------------------------------------------------------------------- **
 ***************************************************************************
 **/
void vesc_can_set_pos(uint8_t controller_id, float pos, FDCAN_HandleTypeDef *hcan)
{
	int32_t send_index = 0;
	uint8_t buffer[4];
	buffer_append_int32(buffer, (int32_t)(pos * 1000000.0f), &send_index);
	can_transmit_eid(controller_id | ((uint32_t)CAN_PACKET_SET_POS << 8), buffer, send_index, hcan);
}

/**
 ***************************************************************************
 ** -------------------------------------------------------------------- **
 ** @name                  : vesc_can_set_current_rel
 ** @brief         		   : ��������ڿ������������ĵ���ֵ��
 ** @param  controller_id  : ������ID��
 ** @param  current_rel    : ��Ե���ֵ����Χ -1.0 �� 1.0����
 ** @param  hcan           : FDCAN���ָ�롣
 ** @retval                : ��
 ** @author                : Ru_yl
 ** -------------------------------------------------------------------- **
 ** @attention     : ��������ڿ������������ĵ���ֵ��
 ** @attention     : ��ֵ��ΧΪ -1.0 �� 1.0������ -1.0 ��ʾȫ����1.0 ��ʾȫ����
 ** -------------------------------------------------------------------- **
 ***************************************************************************
 **/

void vesc_can_set_current_rel(uint8_t controller_id, float current_rel, FDCAN_HandleTypeDef *hcan)
{
	int32_t send_index = 0;
	uint8_t buffer[4];
	buffer_append_float32(buffer, current_rel, 1e5, &send_index);
	can_transmit_eid(controller_id | ((uint32_t)CAN_PACKET_SET_CURRENT_REL << 8), buffer, send_index, hcan);
}


/**
 ***************************************************************************
 ** -------------------------------------------------------------------- **
 ** @name          			: vesc_can_set_current_rel_off_delay
 ** @brief         			: ��������ڿ������������ĵ���ֵ�͹ر��ӳ١�
 ** @param  controller_id   : ������ID��
 ** @param  current_rel     : ��Ե���ֵ����Χ -1.0 �� 1.0����
 ** @param  off_delay       : �ر��ӳ٣���λ�����룩��
 ** @param  hcan            : FDCAN���ָ�롣
 ** @retval                 : ��
 ** @author                 : Ru_yl
 ** @note				    : ���������ƣ����������˹ر��ӳ١�ע�����������ʹ��6�ֽڡ��ر��ӳ������ڽ���������Ϊ������С�����󣬱��ֵ�ǰ��������������һ��ʱ�䡣
 ** -------------------------------------------------------------------- **
 ***************************************************************************
 **/
void vesc_can_set_current_rel_off_delay(uint8_t controller_id, float current_rel, float off_delay, FDCAN_HandleTypeDef *hcan)
{
	int32_t send_index = 0;
	uint8_t buffer[6];
	buffer_append_float32(buffer, current_rel, 1e5, &send_index);
	buffer_append_float16(buffer, off_delay, 1e3, &send_index);
	can_transmit_eid(controller_id | ((uint32_t)CAN_PACKET_SET_CURRENT_REL << 8), buffer, send_index, hcan);
}


/**
 ***************************************************************************
 ** -------------------------------------------------------------------- **
 ** @name          		  : vesc_can_set_handbrake
 ** @brief         		  : ������ɲ����
 ** @param  controller_id : ������ID
 ** @param  current       : ��ɲ��������λ��A��
 ** @param  hcan          : FDCAN���ָ��
 ** @retval        		  : None
 ** @author        		  : Ru_yl
 ** -------------------------------------------------------------------- **
 ***************************************************************************
 **/
void vesc_can_set_handbrake(uint8_t controller_id, float current, FDCAN_HandleTypeDef *hcan)
{
	int32_t send_index = 0;
	uint8_t buffer[4];
	buffer_append_float32(buffer, current, 1e3, &send_index);
	can_transmit_eid(controller_id | ((uint32_t)CAN_PACKET_SET_CURRENT_HANDBRAKE << 8), buffer, send_index, hcan);
}

/**
 ***************************************************************************
 ** -------------------------------------------------------------------- **
 ** @name          		  : vesc_can_set_handbrake_rel
 ** @brief         		  : ������ɲ�������ֵ
 ** @param  controller_id : ������ID
 ** @param  current_rel   : ��ɲ�������ֵ����λ��A��
 ** @param  hcan          : FDCAN���ָ��
 ** @retval        		  : None
 ** @author        		  : Ru_yl
 ** -------------------------------------------------------------------- **
 ***************************************************************************
 **/
void vesc_can_set_handbrake_rel(uint8_t controller_id, float current_rel, FDCAN_HandleTypeDef *hcan)
{
	int32_t send_index = 0;
	uint8_t buffer[4];
	buffer_append_float32(buffer, current_rel, 1e5, &send_index);
	can_transmit_eid(controller_id | ((uint32_t)CAN_PACKET_SET_CURRENT_HANDBRAKE_REL << 8), buffer, send_index, hcan);
}

/**
 ***************************************************************************
 ** -------------------------------------------------------------------- **
 ** @name          		   : VESC_CAN_Receive
 ** @brief         		   : ����VESC��CAN��Ϣ������
 ** @param  RxHeader       : ���յ���CAN��Ϣͷ
 ** @param  CanReceiveData : ���յ�������
 ** @param  VESC           : VESC״̬�ṹ��ָ��
 ** @retval        		   : None
 ** @author        		   : Ru_yl
 ** -------------------------------------------------------------------- **
 ***************************************************************************
 **/
void VESC_CAN_Receive(FDCAN_RxHeaderTypeDef RxHeader, uint8_t *CanReceiveData, VESC_STA *VESC)
{
	int id;
	int cmd;
	id = (RxHeader.Identifier & 0xFF); // ��ȡID
	cmd = (RxHeader.Identifier >> 8);  // ��ȡ����
	if (RxHeader.IdType == FDCAN_EXTENDED_ID && (id == 1 || id == 2 || id == 3 || id == 4))
	{
		switch (cmd)
		{ // cmd�ж�
		case CAN_PACKET_STATUS_1:
			VESC[id].vesc_msg_1.rx_time = 0;
			VESC[id].vesc_msg_1.rpm = (CanReceiveData[0] << 24 | CanReceiveData[1] << 16 | CanReceiveData[2] << 8 | CanReceiveData[3]);
			VESC[id].vesc_msg_1.current = ((int16_t)(CanReceiveData[4] << 8 | CanReceiveData[5])) / 10.0f;
			VESC[id].vesc_msg_1.duty = ((int16_t)(CanReceiveData[6] << 8 | CanReceiveData[7])) / 1000.0f;
			break;
		case CAN_PACKET_STATUS_2:
			VESC[id].vesc_msg_2.rx_time = 0;
			VESC[id].vesc_msg_2.amp_hours = ((int32_t)(CanReceiveData[0] << 24 | CanReceiveData[1] << 16 | CanReceiveData[2] << 8 | CanReceiveData[3])) / 10000.0f;
			VESC[id].vesc_msg_2.amp_hours_charged = ((int32_t)(CanReceiveData[4] << 24 | CanReceiveData[5] << 16 | CanReceiveData[6] << 8 | CanReceiveData[7])) / 10000.0f;
			break;
		case CAN_PACKET_STATUS_3:
			VESC[id].vesc_msg_3.rx_time = 0;
			VESC[id].vesc_msg_3.watt_hours = ((int32_t)(CanReceiveData[0] << 24 | CanReceiveData[1] << 16 | CanReceiveData[2] << 8 | CanReceiveData[3])) / 10000.0f;
			VESC[id].vesc_msg_3.watt_hours_charged = ((int32_t)(CanReceiveData[4] << 24 | CanReceiveData[5] << 16 | CanReceiveData[6] << 8 | CanReceiveData[7])) / 10000.0f;
			break;
		case CAN_PACKET_STATUS_4:
			VESC[id].vesc_msg_4.rx_time = 0;
			VESC[id].vesc_msg_4.temp_fet = ((int16_t)(CanReceiveData[0] << 8 | CanReceiveData[1])) / 10.0f;
			VESC[id].vesc_msg_4.temp_motor = ((int16_t)(CanReceiveData[2] << 8 | CanReceiveData[3])) / 10.0f;
			VESC[id].vesc_msg_4.current_in = ((int16_t)(CanReceiveData[4] << 8 | CanReceiveData[5])) / 10.0f;
			VESC[id].vesc_msg_4.pid_pos_now = ((int16_t)(CanReceiveData[6] << 8 | CanReceiveData[7])) / 50.0f;
			break;
		case CAN_PACKET_STATUS_5:
			VESC[id].vesc_msg_5.rx_time = 0;
			VESC[id].vesc_msg_5.tacho_value = (CanReceiveData[0] << 24 | CanReceiveData[1] << 16 | CanReceiveData[2] << 8 | CanReceiveData[3]);
			VESC[id].vesc_msg_5.v_input = ((int16_t)(CanReceiveData[4] << 8 | CanReceiveData[5])) / 10.0f;
			VESC[id].vesc_msg_5.reserved = (CanReceiveData[6] << 8 | CanReceiveData[7]);
			break;
		case CAN_PACKET_STATUS_6:
			VESC[id].vesc_msg_6.rx_time = 0;
			VESC[id].vesc_msg_6.ADC_1 = (CanReceiveData[0] << 8 | CanReceiveData[1]);
			VESC[id].vesc_msg_6.ADC_2 = (CanReceiveData[2] << 8 | CanReceiveData[3]);
			VESC[id].vesc_msg_6.ADC_3 = (CanReceiveData[4] << 8 | CanReceiveData[5]);
			VESC[id].vesc_msg_6.PPM = (CanReceiveData[6] << 8 | CanReceiveData[7]);
			break;
		default:
			break;
		}
	}
}

