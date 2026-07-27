#ifndef _INTERRUPT_H_
#define _INTERRUPT_H_

#include "clock.h"
#include "key.h"
#include "motor.h"
#include "stdio.h"
#include "ti_msp_dl_config.h"
#include "uart.h"
#include <string.h>

extern uint8_t enable_group1_irq;

void Interrupt_Init(void);

#endif /* #ifndef _INTERRUPT_H_ */