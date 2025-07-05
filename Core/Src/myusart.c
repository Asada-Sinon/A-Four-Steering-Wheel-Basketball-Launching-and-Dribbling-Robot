#include "myusart.h"
#include "usart.h"
#include "disk.h"
#include "Bluetooth.h"
#include "cmsis_os2.h"
#include "FreeRTOS.h"
#include "task.h"
#include "Teaching_Pendant.h"
#include "Computer_Vision.h"
#include "A1_Motor.h"
#include "ANO_TC.h"
#include "mycan.h"
#include "gyro.h"

extern float Vx, Vy, Vw;
float datasend[8];
void HAL_UARTEx_RxEventCallback(UART_HandleTypeDef *huart, uint16_t Size)
{
  uint32_t status_value;
  status_value = taskENTER_CRITICAL_FROM_ISR(); // 临界段代码保护
  // 串口1用来收码盘的数据
  if (huart->Instance == USART1)
  {
    if ((Disk_Encoder.Rec_Data[0] == 0X0D) && (Disk_Encoder.Rec_Data[1] == 0X0A) && (Disk_Encoder.Rec_Data[26] == 0X0A) && (Disk_Encoder.Rec_Data[27] == 0X0D))
    {
      Disk_Encoder_Data_Process(&Disk_Encoder);
    }
    Disk_Encoder_Restart();
  }
  // 串口2用来接收视觉相关的所有数据
  if (huart->Instance == USART2)
  {
    if ((Computer_Vision_Data.Computer_Vision_Rec_Data[0] == 0X0A) && (Computer_Vision_Data.Computer_Vision_Rec_Data[1] == 0X0D) && (Computer_Vision_Data.Computer_Vision_Rec_Data[30] == 0X0B) && (Computer_Vision_Data.Computer_Vision_Rec_Data[31] == 0X0C))
    {
      Computer_Vision_Data_Process(&Computer_Vision_Data);
    }
    Computer_Vision_Restart();
  }
  // 串口4用来接收手柄发送数据
  if (huart->Instance == UART4)
  {
    HT10A_process(Teaching_Pendant_buffer);
    Teaching_Pendant_Restart();
  }
  // 串口7用来接收陀螺仪的数据
  if (huart->Instance == UART7)
  {
    witdecoded(Gyro_data, &Wit_Gyro);
    Wit_Gyro_Restart();
  }
  // 串口10用来和A1电机通信
  if (huart->Instance == USART10)
  {
    g_a1motor.feedback = A1Motor_ProcessFeedback(huart, &hdma_usart10_rx);
  }
  taskEXIT_CRITICAL_FROM_ISR(status_value);
}

void HAL_UART_ErrorCallback(UART_HandleTypeDef *huart)
{
  uint32_t status_value;
  status_value = taskENTER_CRITICAL_FROM_ISR(); // 临界段代码保护
  // 串口1用来收码盘的数据
  if (huart->Instance == USART1)
  {
    if ((Disk_Encoder.Rec_Data[0] == 0X0D) && (Disk_Encoder.Rec_Data[1] == 0X0A) && (Disk_Encoder.Rec_Data[26] == 0X0A) && (Disk_Encoder.Rec_Data[27] == 0X0D))
    {
      Disk_Encoder_Data_Process(&Disk_Encoder);
    }
    Disk_Encoder_Restart();
  }
  // 串口2用来接收视觉相关的所有数据
  if (huart->Instance == USART2)
  {
    if ((Computer_Vision_Data.Computer_Vision_Rec_Data[0] == 0X0A) && (Computer_Vision_Data.Computer_Vision_Rec_Data[1] == 0X0D) && (Computer_Vision_Data.Computer_Vision_Rec_Data[30] == 0X0B) && (Computer_Vision_Data.Computer_Vision_Rec_Data[31] == 0X0C))
    {
      Computer_Vision_Data_Process(&Computer_Vision_Data);
    }
    Computer_Vision_Restart();
  }
  // 串口4用来接收手柄发送数据
  if (huart->Instance == UART4)
  {
    HT10A_process(Teaching_Pendant_buffer);
    Teaching_Pendant_Restart();
  }
  // 串口7用来接收陀螺仪的数据
  if (huart->Instance == UART7)
  {
    witdecoded(Gyro_data, &Wit_Gyro);
    Wit_Gyro_Restart();
  }
  // 串口10用来和A1电机通信
  if (huart->Instance == USART10)
  {
    g_a1motor.feedback = A1Motor_ProcessFeedback(huart, &hdma_usart10_rx);
  }
  taskEXIT_CRITICAL_FROM_ISR(status_value);
}
