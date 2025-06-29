#ifndef __Bluetooth_H
#define __Bluetooth_H

#include "stdint.h"
#include "main.h"

#define BLUETOOTH_DATA_LENGTH 42

typedef struct
{
	  uint8_t rx_data[BLUETOOTH_DATA_LENGTH];
    uint8_t control_mode;
    uint8_t route_select;
    uint8_t fire;
    uint8_t reposition;
    uint8_t dribble;
    uint8_t pass;
    float x;
    float y;
    float w;
}remote_stu;

float Char_to_Float(unsigned char data[]);

extern remote_stu bluetooth_rem;

void remote_data_process(void);



#endif
