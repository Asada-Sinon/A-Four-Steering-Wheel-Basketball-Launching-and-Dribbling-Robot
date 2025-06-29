#ifndef _VESC_H_
#define _VESC_H_
#include "stm32h7xx.h"
#include "main.h"
typedef enum
{
	CAN_PACKET_SET_DUTY 					= 0,
	CAN_PACKET_SET_CURRENT 					= 1,
	CAN_PACKET_SET_CURRENT_BRAKE 			= 2,
	CAN_PACKET_SET_RPM 						= 3,
	CAN_PACKET_SET_POS 						= 4,
	CAN_PACKET_FILL_RX_BUFFER 				= 5,
	CAN_PACKET_FILL_RX_BUFFER_LONG 			= 6,
	CAN_PACKET_PROCESS_RX_BUFFER 			= 7,
	CAN_PACKET_PROCESS_SHORT_BUFFER 		= 8,
	CAN_PACKET_STATUS_1 					= 9,
	CAN_PACKET_SET_CURRENT_REL 				= 10,
	CAN_PACKET_SET_CURRENT_BRAKE_REL 		= 11,
	CAN_PACKET_SET_CURRENT_HANDBRAKE 		= 12,
	CAN_PACKET_SET_CURRENT_HANDBRAKE_REL 	= 13,
	CAN_PACKET_STATUS_2 					= 14,
	CAN_PACKET_STATUS_3 					= 15,
	CAN_PACKET_STATUS_4 					= 16,
	CAN_PACKET_PING 						= 17,
	CAN_PACKET_PONG 						= 18,
	CAN_PACKET_DETECT_APPLY_ALL_FOC 		= 19,
	CAN_PACKET_DETECT_APPLY_ALL_FOC_RES 	= 20,
	CAN_PACKET_CONF_CURRENT_LIMITS 			= 21,
	CAN_PACKET_CONF_STORE_CURRENT_LIMITS 	= 22,
	CAN_PACKET_CONF_CURRENT_LIMITS_IN 		= 23,
	CAN_PACKET_CONF_STORE_CURRENT_LIMITS_IN = 24,
	CAN_PACKET_CONF_FOC_ERPMS 				= 25,
	CAN_PACKET_CONF_STORE_FOC_ERPMS 		= 26,
	CAN_PACKET_STATUS_5 					= 27,
	CAN_PACKET_POLL_TS5700N8501_STATUS 		= 28,
	CAN_PACKET_CONF_BATTERY_CUT 			= 29,
	CAN_PACKET_CONF_STORE_BATTERY_CUT 		= 30,
	CAN_PACKET_SHUTDOWN 					= 31,
	CAN_PACKET_IO_BOARD_ADC_1_TO_4 			= 32,
	CAN_PACKET_IO_BOARD_ADC_5_TO_8 			= 33,
	CAN_PACKET_IO_BOARD_ADC_9_TO_12 		= 34,
	CAN_PACKET_IO_BOARD_DIGITAL_IN 			= 35,
	CAN_PACKET_IO_BOARD_SET_OUTPUT_DIGITAL 	= 36,
	CAN_PACKET_IO_BOARD_SET_OUTPUT_PWM 		= 37,
	CAN_PACKET_BMS_V_TOT 					= 38,
	CAN_PACKET_BMS_I 						= 39,
	CAN_PACKET_BMS_AH_WH 					= 40,
	CAN_PACKET_BMS_V_CELL 					= 41,
	CAN_PACKET_BMS_BAL 						= 42,
	CAN_PACKET_BMS_TEMPS 					= 43,
	CAN_PACKET_BMS_HUM 						= 44,
	CAN_PACKET_BMS_SOC_SOH_TEMP_STAT 		= 45,
	CAN_PACKET_PSW_STAT 					= 46,
	CAN_PACKET_PSW_SWITCH 					= 47,
	CAN_PACKET_BMS_HW_DATA_1 				= 48,
	CAN_PACKET_BMS_HW_DATA_2 				= 49,
	CAN_PACKET_BMS_HW_DATA_3 				= 50,
	CAN_PACKET_BMS_HW_DATA_4 				= 51,
	CAN_PACKET_BMS_HW_DATA_5 				= 52,
	CAN_PACKET_BMS_AH_WH_CHG_TOTAL 			= 53,
	CAN_PACKET_BMS_AH_WH_DIS_TOTAL 			= 54,
	CAN_PACKET_UPDATE_PID_POS_OFFSET 		= 55,
	CAN_PACKET_POLL_ROTOR_POS 				= 56,
	CAN_PACKET_NOTIFY_BOOT 					= 57,
	CAN_PACKET_STATUS_6 					= 58,
	CAN_PACKET_GNSS_TIME 					= 59,
	CAN_PACKET_GNSS_LAT 					= 60,
	CAN_PACKET_GNSS_LON 					= 61,
	CAN_PACKET_GNSS_ALT_SPEED_HDOP 			= 62,
} CAN_PACKET_ID;

// ���ڽ���vesc������can���ݵĽṹ�壺
typedef struct
{
	uint32_t rx_time;
	int32_t rpm;
	float current;
	float duty;
} can_status_msg_1;

typedef struct
{
	uint32_t rx_time;
	float amp_hours;
	float amp_hours_charged;
} can_status_msg_2;

typedef struct
{
	uint32_t rx_time;
	float watt_hours;
	float watt_hours_charged;
} can_status_msg_3;

typedef struct
{
	uint32_t rx_time;
	float temp_fet;
	float temp_motor;
	float current_in;
	float pid_pos_now;
} can_status_msg_4;

typedef struct
{
	uint32_t rx_time;
	int32_t tacho_value;
	float v_input;
	int16_t reserved;
} can_status_msg_5;

typedef struct
{
	uint32_t rx_time;
	uint16_t ADC_1;
	uint16_t ADC_2;
	uint16_t ADC_3;
	uint16_t PPM;
} can_status_msg_6;

typedef struct
{
	can_status_msg_1 vesc_msg_1;
	can_status_msg_2 vesc_msg_2;
	can_status_msg_3 vesc_msg_3;
	can_status_msg_4 vesc_msg_4;
	can_status_msg_5 vesc_msg_5;
	can_status_msg_6 vesc_msg_6;
} VESC_STA;

void VESC_CAN_Send(uint32_t id, const uint8_t *data, uint8_t len, FDCAN_HandleTypeDef *hcan);
void VESC_Set_RPM(uint32_t id, int32_t RPM, FDCAN_HandleTypeDef *hcan); // ���õ��ת�٣���ת�٣�
void VESC_CAN_Receive(FDCAN_RxHeaderTypeDef RxHeader, uint8_t *CanReceiveData, VESC_STA *VESC);
void VESC_Set_Break_Current(uint32_t id, float current, FDCAN_HandleTypeDef *hcan);
// VESC��������

// ������չID CAN֡
void can_transmit_eid(uint32_t id, const uint8_t *data, uint8_t len, FDCAN_HandleTypeDef *hcan);

// �򻺳������16λ����
void buffer_append_int16(uint8_t *buffer, int16_t number, int32_t *index);

// �򻺳������32λ����
void buffer_append_int32(uint8_t *buffer, int32_t number, int32_t *index);

// �򻺳������16λ�����������ţ�
void buffer_append_float16(uint8_t *buffer, float number, float scale, int32_t *index);

// �򻺳������32λ�����������ţ�
void buffer_append_float32(uint8_t *buffer, float number, float scale, int32_t *index);

// ���õ��ռ�ձ�
void vesc_can_set_duty(uint8_t controller_id, float duty, FDCAN_HandleTypeDef *hcan);

// ���õ������
void vesc_can_set_current(uint8_t controller_id, float current, FDCAN_HandleTypeDef *hcan);

// ���õ�������͹ر��ӳ�
void vesc_can_set_current_off_delay(uint8_t controller_id, float current, float off_delay, FDCAN_HandleTypeDef *hcan);

// ���õ��ɲ������
void vesc_can_set_current_brake(uint8_t controller_id, float current, FDCAN_HandleTypeDef *hcan);

// ���õ��ɲ���������ֵ
void vesc_can_set_current_brake_rel(uint8_t controller_id, float current_rel, FDCAN_HandleTypeDef *hcan);

// ���õ��ת��
void vesc_can_set_rpm(uint8_t controller_id, float rpm, FDCAN_HandleTypeDef *hcan);

// ���õ��λ��
void vesc_can_set_pos(uint8_t controller_id, float pos, FDCAN_HandleTypeDef *hcan);

// �������������
void vesc_can_set_current_rel(uint8_t controller_id, float current_rel, FDCAN_HandleTypeDef *hcan);

// ��������������͹ر��ӳ�
void vesc_can_set_current_rel_off_delay(uint8_t controller_id, float current_rel, float off_delay, FDCAN_HandleTypeDef *hcan);

// ������ɲ����
void vesc_can_set_handbrake(uint8_t controller_id, float current, FDCAN_HandleTypeDef *hcan);

// ������ɲ�������ֵ
void vesc_can_set_handbrake_rel(uint8_t controller_id, float current_rel, FDCAN_HandleTypeDef *hcan);

// ���ղ�����VESC CAN��Ϣ
void VESC_CAN_Receive(FDCAN_RxHeaderTypeDef RxHeader, uint8_t *CanReceiveData, VESC_STA *VESC);

#endif
