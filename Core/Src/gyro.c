#include "gyro.h"
#include "usart.h"

#define Sign(a) a > 0 ? (-1) : (1)
uint8_t Gyro_data[BUF_SIZE] __attribute__((section(".ARM.__at_0x24000044")));
GYROSCOPE Wit_Gyro;
uint8_t p[3], y[3], r[3], ax[3], ay[3], az[3], gx[3], gy[3], gz[3];

float parseByteArray(uint8_t byte[3])
{
  // 提取符号位（byte[0]的最低位）
  float sign = 0;
  if (((byte[0] >> 4) & 0x0F) == 0x00)
  {
    sign = 1.0;
  }
  if (((byte[0] >> 4) & 0x0F) == 0x01)
  {
    sign = -1.0;
  }
  float high8_intpart = byte[0] & 0x0f;

  // 提取整数部分和小数部分
  float high4_intPart = (byte[1] >> 4) & 0x0F; // 提取byte[1]的高4位
  float low4_intPart = byte[1] & 0x0F;         // 提取byte[1]的低4位

  float high4_fracPart = (byte[2] >> 4) & 0x0F; // 提取byte[2]的高4位
  float low4_fracPart = byte[2] & 0x0F;         // 提取byte[2]的低4位

  // 计算浮点数值
  float result;
  result = sign * (high8_intpart * 100 + high4_intPart * 10 + low4_intPart * 1 + high4_fracPart * 0.1f +
                   low4_fracPart * 0.01f);

  return result;
}
void ruifen_decoded(uint8_t data[BUF_SIZE], GYROSCOPE *gyro)
{
  for (int i = 0; i < 3; i++)
  {
    r[i] = data[i + 4];
    p[i] = data[i + 7];
    y[i] = data[i + 10];
    ax[i] = data[i + 13];
    ay[i] = data[i + 16];
    az[i] = data[i + 19];
    gx[i] = data[i + 22];
    gy[i] = data[i + 25];
    gz[i] = data[i + 28];
  }
  gyro->roll = parseByteArray(r);
  gyro->pitch = parseByteArray(p);
  gyro->yaw = parseByteArray(y);
  gyro->ACC_X = parseByteArray(ax);
  gyro->ACC_Y = parseByteArray(ay);
  gyro->ACC_Z = parseByteArray(az);
  gyro->GYRO_X = parseByteArray(gx);
  gyro->GYRO_Y = parseByteArray(gy);
  gyro->GYRO_Z = parseByteArray(gz);
}

void witdecoded(uint8_t data[BUF_SIZE], GYROSCOPE *gyro)
{
  for (int i = 0; i < 2; i++)
  {
    r[i] = data[i + 2];
    p[i] = data[i + 4];
    y[i] = data[i + 6];
  }
  gyro->roll = ((Sign((r[1] & 0x80))) * (((r[1] & 0x7F) << 8) | r[0])) / 32768.0 * 180.0;
  gyro->pitch = ((Sign((p[1] & 0x80))) * (((p[1] & 0x7F) << 8) | p[0])) / 32768.0 * 180.0;
  gyro->yaw = ((Sign((y[1] & 0x80))) * (((y[1] & 0x7F) << 8) | y[0])) / 32768.0 * 180.0;
  // if((Sign((y[1]&0x80)) < 0 )) gyro->yaw = -gyro->yaw-180;
}

void Wit_Gyro_Restart(void)
{
    HAL_UARTEx_ReceiveToIdle_DMA(&huart7, &Gyro_data[0], 32);
    __HAL_DMA_DISABLE_IT(&hdma_uart7_rx, DMA_IT_HT);
}

