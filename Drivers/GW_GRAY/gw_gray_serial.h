#ifndef _GW_GRAY_SERIAL_H_
#define _GW_GRAY_SERIAL_H_

#include "clock.h"
#include "ti_msp_dl_config.h"
#include "uart.h"

extern uint8_t gw_gray_sensor[8];

uint8_t gw_gray_serial_read();
void gw_gray_serial_tick();

#endif /* #ifndef _GW_GRAY_SERIAL_H_ */