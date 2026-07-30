#ifndef _GRAY_TRACE_H_
#define _GRAY_TRACE_H_

#include "gw_gray_serial.h"
#include "mode.h"
#include "motor.h"
#include "pid.h"
#include "ti_msp_dl_config.h"
#include "uart.h"

// void gray_trace(uint8_t *sensorValues);
void gray_trace_tick();

#endif // _GRAY_TRACE_H_