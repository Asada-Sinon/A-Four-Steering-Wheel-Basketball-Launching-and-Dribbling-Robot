#ifndef _VESC_H_
#define _VESC_H_
#include "stm32h7xx.h"
#include "main.h"
typedef enum {
CAN_PACKET_SET_DUTY=0 ,          //0	设置电机占空比	4字节	有符号整数	单位：Thousandths of percent (5000 0 –> 50%)
CAN_PACKET_SET_CURRENT,          //1	设置电机电流	4字节	有符号整数	mA
CAN_PACKET_SET_CURRENT_BRAKE,    //2	设置制动电流	4字节	有符号整数	mA
CAN_PACKET_SET_RPM,              //3	设置（电）转速	4字节	有符号整数	ERPM
CAN_PACKET_SET_POS,              //4	设置电机转角位置
CAN_PACKET_FILL_RX_BUFFER,       //5
CAN_PACKET_FILL_RX_BUFFER_LONG,  //6
CAN_PACKET_PROCESS_RX_BUFFER,    //7
CAN_PACKET_PROCESS_SHORT_BUFFER, //8
CAN_PACKET_STATUS_1,             //9
CAN_PACKET_SET_CURRENT_REL,      //10
CAN_PACKET_SET_CURRENT_BRAKE_REL,//11
CAN_PACKET_SET_CURRENT_HANDBRAKE,
CAN_PACKET_SET_CURRENT_HANDBRAKE_REL,
CAN_PACKET_STATUS_2,
CAN_PACKET_STATUS_3,
CAN_PACKET_STATUS_4,
CAN_PACKET_PING,
CAN_PACKET_PONG,
CAN_PACKET_DETECT_APPLY_ALL_FOC,
CAN_PACKET_DETECT_APPLY_ALL_FOC_RES,
CAN_PACKET_CONF_CURRENT_LIMITS,
CAN_PACKET_CONF_STORE_CURRENT_LIMITS,
CAN_PACKET_CONF_CURRENT_LIMITS_IN,
CAN_PACKET_CONF_STORE_CURRENT_LIMITS_IN,
CAN_PACKET_CONF_FOC_ERPMS,
CAN_PACKET_CONF_STORE_FOC_ERPMS,
CAN_PACKET_STATUS_5
} CAN_PACKET_ID;


//用于接受vesc发出的can数据的结构体：
typedef struct {
	uint32_t rx_time;
	int32_t rpm;
	float current;
	float duty;
} can_status_msg_1;

typedef struct {
	uint32_t rx_time;
	float amp_hours;
	float amp_hours_charged;
} can_status_msg_2;

typedef struct {	
	uint32_t rx_time;
	float watt_hours;
	float watt_hours_charged;
} can_status_msg_3;

typedef struct {
	uint32_t rx_time;
	float temp_fet;
	float temp_motor;
	float current_in;
	float pid_pos_now;
} can_status_msg_4;

typedef struct {
	uint32_t rx_time;
	int32_t tacho_value;
	float v_input;
	int16_t reserved;
} can_status_msg_5;

typedef struct {
	can_status_msg_1 vesc_msg_1;
	can_status_msg_2 vesc_msg_2;
	can_status_msg_3 vesc_msg_3;
	can_status_msg_4 vesc_msg_4;
	can_status_msg_5 vesc_msg_5;	

} VESC_STA;
 
void VESC_CAN_Send(uint32_t id, const uint8_t *data, uint8_t len, FDCAN_HandleTypeDef *hcan);
void VESC_Set_RPM(uint32_t id, int32_t RPM, FDCAN_HandleTypeDef *hcan);//设置电机转速（电转速）
void VESC_CAN_Receive(FDCAN_RxHeaderTypeDef RxHeader, uint8_t* CanReceiveData);
void VESC_Set_Break_Current(uint32_t id, float current, FDCAN_HandleTypeDef *hcan);
//VESC接收数据
#endif
