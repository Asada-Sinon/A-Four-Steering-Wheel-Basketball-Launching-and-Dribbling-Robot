#include "mycan.h"
#include "stm32h7xx_hal_fdcan.h"
#include "chassis.h"
#include "pid.h"
#include "return_data_process.h"
#include "string.h"
#include "route.h"
#include "cmsis_os2.h"
#include "FreeRTOS.h"
#include "task.h"
#include "shangceng.h"

Order_To_Subcontroller_Struct Order_To_Subcontroller;   //给分控发送的命令
VESC_Data_From_Subcontroller_Struct VESC_Data_From_Subcontroller[4]; // 从分控接收的VESC数据
extern Motor_Struct Chassis_Steering_Motor[4];
extern Coordinate_Speed_Struct Robot_Coordinate_System_V;
HAL_StatusTypeDef FDCAN_Config_Start(FDCAN_HandleTypeDef *hfdcan)
{
    FDCAN_FilterTypeDef FDCAN_FilterConfig;
    FDCAN_FilterConfig.IdType = FDCAN_STANDARD_ID;
    FDCAN_FilterConfig.FilterIndex = 0;
    FDCAN_FilterConfig.FilterType = FDCAN_FILTER_MASK;
    FDCAN_FilterConfig.FilterConfig = FDCAN_FILTER_TO_RXFIFO0;
    FDCAN_FilterConfig.FilterID1 = 0x00000000;
    FDCAN_FilterConfig.FilterID2 = 0x00000000;
    HAL_FDCAN_ConfigFilter(hfdcan, &FDCAN_FilterConfig);
    // 未来如果不进中断的话，可以尝试配置一下过滤器，现在过滤器还没有配置
    //HAL_FDCAN_ConfigGlobalFilter(&hfdcan1, FDCAN_REJECT, FDCAN_REJECT, FDCAN_FILTER_REMOTE, FDCAN_FILTER_REMOTE);
    //HAL_FDCAN_ConfigGlobalFilter(hfdcan, FDCAN_ACCEPT_IN_RX_FIFO0, FDCAN_ACCEPT_IN_RX_FIFO0, FDCAN_FILTER_REMOTE, FDCAN_FILTER_REMOTE);
    HAL_FDCAN_Start(hfdcan);
    HAL_FDCAN_ActivateNotification(hfdcan, FDCAN_IT_RX_FIFO0_NEW_MESSAGE, 0);
    return HAL_OK;
}

// FDCAN使用经典can模式发送数据
void CAN_Send_Data(FDCAN_HandleTypeDef *hfdcan, uint16_t ID, uint8_t *pData, Motor_Struct *Motor, uint16_t Len)
{
    int16_t current[4];
    for (int i = 0; i < 4; i++)
        current[i] = (int16_t)((Motor + i)->Current_Output);
    // Ensure pData is large enough and clear it
    memset(pData, 0, 8);
    pData[0] = (uint8_t)(current[0] >> 8) & 0xFF;
    pData[1] = (uint8_t)(current[0]) & 0xFF;
    pData[2] = (uint8_t)(current[1] >> 8) & 0xFF;
    pData[3] = (uint8_t)(current[1]) & 0xFF;
    pData[4] = (uint8_t)(current[2] >> 8) & 0xFF;
    pData[5] = (uint8_t)(current[2]) & 0xFF;
    pData[6] = (uint8_t)(current[3] >> 8) & 0xFF;
    pData[7] = (uint8_t)(current[3]) & 0xFF;

    FDCAN_TxHeaderTypeDef Tx_Header;
    Tx_Header.IdType = FDCAN_STANDARD_ID;
    Tx_Header.TxFrameType = FDCAN_DATA_FRAME;
    Tx_Header.Identifier = ID; // 标识符
    Tx_Header.DataLength = Len;
    Tx_Header.ErrorStateIndicator = FDCAN_ESI_ACTIVE;
    Tx_Header.BitRateSwitch = FDCAN_BRS_OFF;
    Tx_Header.FDFormat = FDCAN_CLASSIC_CAN;
    Tx_Header.TxEventFifoControl = FDCAN_NO_TX_EVENTS;
    Tx_Header.MessageMarker = 0;

    HAL_StatusTypeDef status = HAL_FDCAN_AddMessageToTxFifoQ(hfdcan, &Tx_Header, pData);
    if (status != HAL_OK)
    {
        HAL_Delay(1);
        // Handle error, e.g., log the error or retry
    }
}

// fdcan使用fdcan发送数据
void FDCAN_Send_Data(FDCAN_HandleTypeDef *hfdcan, uint16_t ID, uint8_t Data[16], Coordinate_Speed_Struct *Speed, Order_To_Subcontroller_Struct *Order)
{
    FDCAN_TxHeaderTypeDef Tx_Header = {0}; // 初始化结构体
    Tx_Header.IdType = FDCAN_STANDARD_ID;
    Tx_Header.TxFrameType = FDCAN_DATA_FRAME;
    Tx_Header.Identifier = ID;
    Tx_Header.DataLength = FDCAN_DLC_BYTES_16; // 16字节数据
    Tx_Header.ErrorStateIndicator = FDCAN_ESI_ACTIVE;
    Tx_Header.BitRateSwitch = FDCAN_BRS_ON; // 启用可变速率
    Tx_Header.FDFormat = FDCAN_FD_CAN;      // CAN FD帧格式
    Tx_Header.TxEventFifoControl = FDCAN_NO_TX_EVENTS;
    Tx_Header.MessageMarker = 0;

    // 假设 Speed->Vx/Vy/Vw 是浮点数，使用 memcpy 提取二进制字节流
    // 如果是整型，直接赋值即可（例如 uint32_t vx = Speed->Vx;）
    uint32_t vx_bits, vy_bits, vw_bits;
    memcpy(&vx_bits, &Speed->Vx, sizeof(float));
    memcpy(&vy_bits, &Speed->Vy, sizeof(float));
    memcpy(&vw_bits, &Speed->Vw, sizeof(float));
    memset(Data, 0, 16);

    // 小端序填充（低位在前，高位在后）
    Data[0] = (uint8_t)(vx_bits);       // 字节0: 最低字节
    Data[1] = (uint8_t)(vx_bits >> 8);  
    Data[2] = (uint8_t)(vx_bits >> 16);
    Data[3] = (uint8_t)(vx_bits >> 24); // 字节3: 最高字节

    Data[4] = (uint8_t)(vy_bits);
    Data[5] = (uint8_t)(vy_bits >> 8);
    Data[6] = (uint8_t)(vy_bits >> 16);
    Data[7] = (uint8_t)(vy_bits >> 24);

    Data[8] = (uint8_t)(vw_bits);
    Data[9] = (uint8_t)(vw_bits >> 8);
    Data[10] = (uint8_t)(vw_bits >> 16);
    Data[11] = (uint8_t)(vw_bits >> 24);

    Data[12] = (uint8_t)(Order->Wheel_Break);
    Data[13] = (uint8_t)(Order->Wheel_Lock);
    Data[14] = (uint8_t)(Order->Order_2);
    Data[15] = (uint8_t)(Order->Order_3);
    // 发送数据到FDCAN
    HAL_FDCAN_AddMessageToTxFifoQ(hfdcan, &Tx_Header, Data);
}

// Fifo0中断回调
void HAL_FDCAN_RxFifo0Callback(FDCAN_HandleTypeDef *hfdcan, uint32_t RxFifo0ITs)
{
    uint32_t status_value;
	status_value = taskENTER_CRITICAL_FROM_ISR(); // 临界代码保护
    FDCAN_RxHeaderTypeDef RxHeader;
    uint8_t Can_Rx_Data[16] = {0};
    HAL_FDCAN_GetRxMessage(hfdcan, FDCAN_RX_FIFO0, &RxHeader, Can_Rx_Data);
    if (hfdcan->Instance == FDCAN1)
    {
        //分控VESC速度回传
        if (Can_Rx_Data[0] == 0x0A && RxHeader.Identifier == 0x0BB &&RxHeader.DataLength == FDCAN_DLC_BYTES_6)
        {
            VESC_Data_Receive( Can_Rx_Data, VESC_Data_From_Subcontroller);
        }
    }
    if (hfdcan->Instance == FDCAN2)
    {
        //2006速度回传
        if ((RxHeader.Identifier & 0xF00) == 0x200 && RxHeader.IdType == FDCAN_STANDARD_ID) // 获得接收到的数据头和数据，判断是否为3508数据
        {
            Motor_DJI_Angel_Sum_Return_Process(&RxHeader, Can_Rx_Data, ShangCeng_motor);
        }
    }
    taskEXIT_CRITICAL_FROM_ISR(status_value);
}
