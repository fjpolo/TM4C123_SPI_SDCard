// standard C
#include <stdarg.h>
#include <stdbool.h>
#include <stdint.h>
// inc
#include "inc/hw_types.h"
#include "inc/hw_memmap.h"
#include "inc/hw_ints.h"
// driverlib
#include "driverlib/pin_map.h"
#include "driverlib/gpio.h"
#include "driverlib/sysctl.h"
#include "driverlib/pwm.h"
#include "driverlib/interrupt.h"
#include "driverlib/uart.h"
#include "driverlib/systick.h"
#include "driverlib/timer.h"
// datalogger
#include "datalogger/datalogger.h"
#include "datalogger/delay.h"

unsigned long int _MSDELAY_BASE;
unsigned long int _USDELAY_BASE;

void delayMS(unsigned long int ms){
	SysCtlDelay(ms * _MSDELAY_BASE);
}
/*
 * Tiene un error de 1.32 uS
 * Mínimo tiempo de delay 3 uS (ejecuta 3 + 1.32 = 4.36)
 *
 * */
void delayUS(unsigned long int us){
	SysCtlDelay(us * _USDELAY_BASE);
}
