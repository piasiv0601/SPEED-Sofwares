/*
 * main.c
 */

#include <stdbool.h>
#include <stdint.h>
#include "inc/hw_i2c.h"
#include "inc/hw_memmap.h"
#include "inc/hw_types.h"
#include "driverlib/rom.h"
#include "driverlib/gpio.h"
#include "driverlib/i2c.h"
#include "inc/tm4c123gh6pge.h"
#include "driverlib/ssi.h"
#include "driverlib/interrupt.h"
#include "driverlib/pin_map.h"
#include "driverlib/sysctl.h"
#include "driverlib/uart.h"
#include "utils/uartstdio.h"
#include "inc/hw_nvic.h"
#include "driverlib/rom.h"
#include "driverlib/timer.h"
#include "driverlib/udma.h"
#include "driverlib/flash.h"
#include "inc/hw_ssi.h"
#include "ISR.h"
#include "I2C.h"
#include "SoftLED.h"
#include "HV_Control.h"




void main(void)
{

    SysCtlClockSet(SYSCTL_SYSDIV_2_5 | SYSCTL_USE_PLL | SYSCTL_XTAL_16MHZ |
                       SYSCTL_OSC_MAIN);
    ConfigureWatchdog();
    FlashProtectSet(0x8000,FlashReadWrite);
    FlashProtectSave();
    pyld_status=0;
    mode_change=3;
    PYLD_SPEED_STANDBY_INIT();
    Flash_Parameter_Program();
    UARTprintf("Printing Data \n");
    Print_Buffer();

    int rest=0;
	while(1)
	{

		loop_counter++;
		if(mode_change==1)
		{
			mode_change=0;
			WatchdogIntClear(WATCHDOG0_BASE);
			SysCtlDelay(10);
			PYLD_SPEED_STANDBY_INIT();
		}
		if(mode_change==2)
		{
			mode_change=0;
			WatchdogIntClear(WATCHDOG0_BASE);
			SysCtlDelay(10);
			PYLD_SPEED_HIBERNATE_INIT();
		}
		if(mode_change==3)
		{
			mode_change=0;
			WatchdogIntClear(WATCHDOG0_BASE);
			SysCtlDelay(10);
			PYLD_SPEED_SCIENCE_INIT();
		}
		if(loop_counter==6000000)
		{
			rest++;
			UARTprintf("System Idle... %d \n \r",rest);
			//if(rest == 3)
			//{
				//mode_change = 2;
			//}
			WatchdogIntClear(WATCHDOG0_BASE);
			loop_counter=0;
			SysCtlDelay(SysCtlClockGet()/3);

		}
		SysCtlDelay(10);
	}

}
