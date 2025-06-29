#include <stdint.h>
#include "fdcan.h"
#include "stm32h7xx_hal_fdcan.h"
#include "VESC_NEW.h"

// Implementation for sending extended ID CAN-frames
/**
 ***************************************************************************
 ** -------------------------------------------------------------------- **
 ** @name          : can_transmit_eid
 ** @brief         : 发送一个扩展ID的CAN消息
 ** @param      id : CAN消息的扩展ID
 ** @param    data : 指向待发送数据的指针
 ** @param     len : 待发送数据的长度
 ** @param    hcan : FDCAN句柄指针
 ** @retval        : 无
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
 ** @brief         : 将一个16位整数添加到缓冲区
 ** @param  buffer : 指向缓冲区的指针
 ** @param  number : 要添加的16位整数
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
** @brief         : 添加32位整数
** @param  buffer : 缓存区
** @param  number : 要添加的32位整数
** @param  index  : 缓存区索引
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
 ** @brief         : 将一个16位浮点数添加到缓冲区
 ** @param  buffer : 指向缓冲区的指针
 ** @param  number : 要添加的16位浮点数
 ** @param  scale  : 缩放因子
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
 ** @brief         : 将一个32位浮点数添加到缓冲区
 ** @param  buffer : 指向缓冲区的指针
 ** @param  number : 要添加的32位浮点数
 ** @param  scale  : 缩放因子
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
 ** @brief         		  : 设置电机的占空比
 ** @param  controller_id : 控制器ID
 ** @param  duty          : 占空比（0-100）
 ** @param  hcan          : FDCAN句柄指针
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
 ** @brief         		  : 设置电机的电流
 ** @param  controller_id : 控制器ID
 ** @param  current       : 电流（单位：A）
 ** @param  hcan          : FDCAN句柄指针
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
 ** @brief         		  : 设置电机的电流和关闭延迟
 ** @param  controller_id : 控制器ID
 ** @param  current       : 电流（单位：A）
 ** @param  off_delay     : 关闭延迟（单位：秒）
 ** @param  hcan          : FDCAN句柄指针
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
 ** @brief         		  : 设置电机的刹车电流
 ** @param  controller_id : 控制器ID
 ** @param  current       : 刹车电流（单位：A）
 ** @param  hcan          : FDCAN句柄指针
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
 ** @brief         		  : 设置电机的刹车电流相对值
 ** @param  controller_id : 控制器ID
 ** @param  current_rel   : 刹车电流相对值（单位：A）
 ** @param  hcan          : FDCAN句柄指针
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
 ** @brief         		  : 设置电机的转速
 ** @param  controller_id : 控制器ID
 ** @param  rpm           : 转速（单位：RPM）
 ** @param  hcan          : FDCAN句柄指针
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
 ** @brief         		  : 设置电机的位置
 ** @param  controller_id : 控制器ID
 ** @param  pos           : 位置（单位：弧度）
 ** @param  hcan          : FDCAN句柄指针
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
 ** @brief         		   : 设置相对于控制器最大电流的电流值。
 ** @param  controller_id  : 控制器ID。
 ** @param  current_rel    : 相对电流值（范围 -1.0 到 1.0）。
 ** @param  hcan           : FDCAN句柄指针。
 ** @retval                : 无
 ** @author                : Ru_yl
 ** -------------------------------------------------------------------- **
 ** @attention     : 设置相对于控制器最大电流的电流值。
 ** @attention     : 该值范围为 -1.0 到 1.0，其中 -1.0 表示全反向，1.0 表示全正向。
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
 ** @brief         			: 设置相对于控制器最大电流的电流值和关闭延迟。
 ** @param  controller_id   : 控制器ID。
 ** @param  current_rel     : 相对电流值（范围 -1.0 到 1.0）。
 ** @param  off_delay       : 关闭延迟（单位：毫秒）。
 ** @param  hcan            : FDCAN句柄指针。
 ** @retval                 : 无
 ** @author                 : Ru_yl
 ** @note				    : 与上述类似，但还设置了关闭延迟。注意该命令现在使用6字节。关闭延迟用于在将电流设置为低于最小电流后，保持当前控制器继续运行一段时间。
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
 ** @brief         		  : 设置手刹电流
 ** @param  controller_id : 控制器ID
 ** @param  current       : 手刹电流（单位：A）
 ** @param  hcan          : FDCAN句柄指针
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
 ** @brief         		  : 设置手刹电流相对值
 ** @param  controller_id : 控制器ID
 ** @param  current_rel   : 手刹电流相对值（单位：A）
 ** @param  hcan          : FDCAN句柄指针
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
 ** @brief         		   : 接收VESC的CAN消息并解析
 ** @param  RxHeader       : 接收到的CAN消息头
 ** @param  CanReceiveData : 接收到的数据
 ** @param  VESC           : VESC状态结构体指针
 ** @retval        		   : None
 ** @author        		   : Ru_yl
 ** -------------------------------------------------------------------- **
 ***************************************************************************
 **/
void VESC_CAN_Receive(FDCAN_RxHeaderTypeDef RxHeader, uint8_t *CanReceiveData, VESC_STA *VESC)
{
	int id;
	int cmd;
	id = (RxHeader.Identifier & 0xFF); // 截取ID
	cmd = (RxHeader.Identifier >> 8);  // 截取命令
	if (RxHeader.IdType == FDCAN_EXTENDED_ID && (id == 1 || id == 2 || id == 3 || id == 4))
	{
		switch (cmd)
		{ // cmd判断
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

