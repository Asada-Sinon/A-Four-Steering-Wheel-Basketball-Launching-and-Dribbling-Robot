#include "Bluetooth.h"
#include "usart.h"
#include "dma.h"

remote_stu bluetooth_rem;

float Char_to_Float(unsigned char data[])
{
	float res;
	unsigned char *pp = (unsigned char *)&res;
	pp[0] = data[0];
	pp[1] = data[1];
	pp[2] = data[2];
	pp[3] = data[3];
	return res;
};

void remote_data_process()
{
    if (bluetooth_rem.rx_data[0] == 0xA5 && bluetooth_rem.rx_data[20]== 0x5A)
    {
        bluetooth_rem.control_mode = bluetooth_rem.rx_data[1];
        bluetooth_rem.route_select = bluetooth_rem.rx_data[2];
        bluetooth_rem.fire = bluetooth_rem.rx_data[3];
        bluetooth_rem.reposition = bluetooth_rem.rx_data[4];
        bluetooth_rem.pass = bluetooth_rem.rx_data[5];
        bluetooth_rem.dribble = bluetooth_rem.rx_data[6];
        bluetooth_rem.x = Char_to_Float(&bluetooth_rem.rx_data[7]);
        bluetooth_rem.y = Char_to_Float(&bluetooth_rem.rx_data[11]);
        bluetooth_rem.w = Char_to_Float(&bluetooth_rem.rx_data[15]);
    }
    else
    {
        HAL_UARTEx_ReceiveToIdle_DMA(&huart3, bluetooth_rem.rx_data, 42);
        __HAL_DMA_DISABLE_IT(&hdma_usart1_rx, DMA_IT_HT);
    }
}

