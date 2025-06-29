#include "disk.h"
#include "usart.h"
#include "stm32h7xx_hal_dma.h"
#include "math.h"
#include "route.h"

Disk_Encoder_Struct Disk_Encoder;
uint8_t Disk_State = Disk_Error;
Coordinate_Position_Struct Eccentric={243,60,0};
extern Coordinate_Position_Struct Zero_Point;

void Disk_Encoder_Restart(void)
{
    HAL_UARTEx_ReceiveToIdle_DMA(&huart1, &Disk_Encoder.Rec_Data[0], 28);
    __HAL_DMA_DISABLE_IT(&hdma_usart1_rx, DMA_IT_HT);
}

/**
 * @brief 从4个无符号8位整数中获取浮点数，小端序
 * 该函数接收一个指向4个无符号8位整数的指针，并将这4个字节转换为一个浮点数返回。
 * @param p 指向4个无符号8位整数的指针
 * @return 转换后的浮点数
 */
float Get_Float_From_4u8(unsigned char *p)
{
    float result;
    unsigned char *float_bytes = (unsigned char *)&result;
    *float_bytes = p[0];
    *(float_bytes + 1) = p[1];
    *(float_bytes + 2) = p[2];
    *(float_bytes + 3) = p[3];
    return result;
}

/**
 * @brief 处理旋转编码器数据
 * 该函数处理旋转编码器的数据，计算当前角度、累计旋转角度以及在绝对坐标系下的偏航角。
 * @param Encoder 指向Disk_Encoder_Struct结构体的指针，用于存储编码器数据
 */
void Disk_Encoder_Data_Process(Disk_Encoder_Struct *Encoder)
{
    // 得到此时码盘角度值
    Encoder->Yaw.Now_Yaw = Get_Float_From_4u8(&Encoder->Rec_Data[2]);

    // 定义临时变量
    float Dleta_Angle1 = 0;
    float Dleta_Angle2 = 0;

    // 进行角度判断,得到真实变化的角度 Δψ,存储在Sum_Angle变量中
    if (Encoder->Yaw.Last_Yaw < Encoder->Yaw.Now_Yaw)
    {
        Dleta_Angle1 = Encoder->Yaw.Now_Yaw - Encoder->Yaw.Last_Yaw;
        Dleta_Angle2 = Encoder->Yaw.Now_Yaw - Encoder->Yaw.Last_Yaw - 360;
    }
    else
    {
        Dleta_Angle1 = Encoder->Yaw.Now_Yaw - Encoder->Yaw.Last_Yaw;
        Dleta_Angle2 = 360 + Encoder->Yaw.Now_Yaw - Encoder->Yaw.Last_Yaw;
    }

    // 无论顺时针转还是逆时针转，都是取小的那个角度
    Encoder->Yaw.Sum_Angle = (fabs(Dleta_Angle1)) > (fabs(Dleta_Angle2)) ? Dleta_Angle2 : Dleta_Angle1;

    // 累加算出的 Δψ，得到累计旋转角度，并且更新Last_Yaw的值
    Encoder->Yaw.Last_Yaw = Encoder->Yaw.Now_Yaw;
    Encoder->Yaw.Accumulated_Rotation_Angle += Encoder->Yaw.Sum_Angle;

    // Reset_Rotation_Angle 是初始状态或重定位时候的角度矫正，最后得到在绝对坐标系下的偏航角 World_Rotation_Angle
    Encoder->Yaw.World_Rotation_Angle = Encoder->Yaw.Accumulated_Rotation_Angle + Encoder->Yaw.Reset_Rotation_Angle;

    // 读取码盘回传的x,y坐标
    Encoder->Cod.RE_X = Get_Float_From_4u8(&Encoder->Rec_Data[14]);
    Encoder->Cod.RE_Y = Get_Float_From_4u8(&Encoder->Rec_Data[18]);
    //当码盘偏心时，需要进行数据处理
    //Disk_Encoder.Cod.Disk_Position = Position_Coordinate_Transformation(&Eccentric, &Zero_Point, -Disk_Encoder.Yaw.Accumulated_Rotation_Angle);
    //Disk_Encoder.Cod.Chassis_Position_From_Disk.X = -Disk_Encoder.Cod.Disk_Position.X + Disk_Encoder.Cod.RE_X + Eccentric.X;
    //Disk_Encoder.Cod.Chassis_Position_From_Disk.Y = -Disk_Encoder.Cod.Disk_Position.Y - Disk_Encoder.Cod.RE_Y + Eccentric.Y;
    Disk_Encoder.Cod.Chassis_Position_From_Disk.X = Encoder->Cod.RE_X;
    Disk_Encoder.Cod.Chassis_Position_From_Disk.Y = Encoder->Cod.RE_Y;
}
