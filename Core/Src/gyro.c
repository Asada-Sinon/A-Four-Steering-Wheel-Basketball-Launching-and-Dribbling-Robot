#include "gyro.h"
#include "usart.h"
#include "disk.h"

#define Sign(a) a > 0 ? (-1) : (1)
uint8_t Gyro_data[BUF_SIZE]; //__attribute__((section(".ARM.__at_0x24000044")));
GYROSCOPE Wit_Gyro;

#define Sign(a) a > 0 ? (-1) : (1)
float first_gyro;
uint8_t is_first_gyro = 20;

uint8_t z_v[2] = {0};
uint8_t r[2] = {0};
uint8_t p[2] = {0};
uint8_t y_gyro[2] = {0};


float limit180(float a)
{
  while (a < -180)
  {
    a += 360;
  }
  while (a > 180)
  {
    a -= 360;
  }
  return a;
}

void witdecoded(uint8_t data[BUF_SIZE], GYROSCOPE *gyro)
{
  if (data[0] == 0x55 && data[1] == 0x52)
  {
    for (int i = 0; i < 2; i++)
    {
      z_v[i] = data[i + 6];
    }
    gyro->GYRO_Z = ((Sign((z_v[1] & 0x80))) * (((z_v[1] & 0x7F) << 8) | z_v[0])) / 32768.0 * 2000.0;

    if ((Sign((z_v[1] & 0x80)) < 0))
      gyro->GYRO_Z = -gyro->GYRO_Z - 2000;
  }
  if (data[11] == 0x55 && data[12] == 0x53)
  {
    for (int i = 0; i < 2; i++)
    {
      r[i] = data[i + 13];
      p[i] = data[i + 15];
      y_gyro[i] = data[i + 17];
    }
    gyro->roll = ((Sign((r[1] & 0x80))) * (((r[1] & 0x7F) << 8) | r[0])) / 32768.0 * 180.0;
    gyro->pitch = ((Sign((p[1] & 0x80))) * (((p[1] & 0x7F) << 8) | p[0])) / 32768.0 * 180.0;
    gyro->yaw = ((Sign((y_gyro[1] & 0x80))) * (((y_gyro[1] & 0x7F) << 8) | y_gyro[0])) / 32768.0 * 180.0;
    if ((Sign((y_gyro[1] & 0x80)) < 0))
      gyro->yaw = -gyro->yaw - 180;
  }
  gyro->yaw = limit180(gyro->yaw);
  if (is_first_gyro)
  {
    first_gyro = gyro->yaw - Disk_Encoder.Yaw.Now_Yaw;
    is_first_gyro--;
  }
  gyro->yaw = limit180(gyro->yaw - first_gyro);
}


void Wit_Gyro_Restart(void)
{
  HAL_UARTEx_ReceiveToIdle_DMA(&huart7, &Gyro_data[0], 32);
  __HAL_DMA_DISABLE_IT(&hdma_uart7_rx, DMA_IT_HT);
}
