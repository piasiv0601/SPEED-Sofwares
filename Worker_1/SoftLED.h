/*
 * SoftLED.h
 *
 *  Created on: Jul 24, 2016
 *      Author: USER
 */

#ifndef SOFTLED_H_
#define SOFTLED_H_

#include <stdbool.h>
#include <stdint.h>
#include "inc/hw_i2c.h"
#include "inc/hw_ints.h"
#include "inc/hw_memmap.h"
#include "inc/hw_types.h"
#include "driverlib/rom.h"
#include "driverlib/gpio.h"
#include "driverlib/i2c.h"
#include "driverlib/ssi.h"
#include "driverlib/interrupt.h"
#include "driverlib/pin_map.h"
#include "driverlib/sysctl.h"
#include "driverlib/uart.h"
#include "driverlib/adc.h"
#include "inc/hw_nvic.h"
#include "driverlib/rom.h"
#include "driverlib/adc.c"
#include "driverlib/timer.h"
#include <stdbool.h>
#include <stdint.h>
#include "inc/hw_i2c.h"
#include "inc/hw_ints.h"
#include "inc/hw_memmap.h"
#include "inc/hw_sysctl.h"
#include "inc/hw_types.h"
#include "driverlib/debug.h"
#include "driverlib/i2c.h"
#include "driverlib/udma.h"
#include "driverlib/interrupt.h"
#include "driverlib/flash.h"
#include "inc/hw_ssi.h"
#include "inc/tm4c123gh6pge.h"

uint16_t LED_data[4096];
void LEDAcquire(uint32_t pwidth, uint32_t period,uint32_t nos);
void MAV_Filter(void);
uint16_t Peak_Detect(void);
void LED_Buffer_Clear(void);
void SoftLEDConfig(void);
void PrtData(void);
int i,pia;

void prtData(void)
{
	for(i = 0 ; i < 4096 ; i++)
	{
		UARTprintf("%d \n",LED_data[i]);
	}
}



void LEDAcquire(uint32_t pwidth, uint32_t period,uint32_t nos)
{
	uint32_t  lt[1];
	LED_Buffer_Clear();
	SoftLEDConfig();
	Configure_ADC_SCI();
	UARTprintf("LED Calibration intializing ... \n");
	for (i = 0; i < nos; i++ )
	{
		//GPIOPinWrite(GPIO_PORTN_BASE, GPIO_PIN_2, GPIO_PIN_2);
		GPIO_PORTN_DATA_R = 0x04;
		SysCtlDelay(pwidth);
		GPIO_PORTN_DATA_R = 0x00;
		//GPIOPinWrite(GPIO_PORTN_BASE, GPIO_PIN_2, 0);
		pia = 0;
		while(!(GPIOPinRead(GPIO_PORTD_BASE,GPIO_PIN_5) == GPIO_PIN_5))
		{
			pia++;
			if(pia > 100)
			{
				break;
			}
		}
		SysCtlDelay(20);
	    ADCProcessorTrigger(ADC1_BASE, 3);   /// ADC 1 for bulk.... code changed to bulk tiggered dedx
	    while(!ADCIntStatus(ADC1_BASE, 3, false))
	    {
	    }
	    ADCIntClear(ADC1_BASE, 3);
	    ADCSequenceDataGet(ADC1_BASE, 3, lt);
	    LED_data[lt[0]]++;
		GPIOPinWrite(GPIO_PORTK_BASE, GPIO_PIN_5, 0);
		SysCtlDelay(100);
		GPIOPinWrite(GPIO_PORTK_BASE, GPIO_PIN_5, GPIO_PIN_5);
		GPIOPinWrite(GPIO_PORTD_BASE, GPIO_PIN_7, 0);
		SysCtlDelay(100);
		GPIOPinWrite(GPIO_PORTD_BASE, GPIO_PIN_7, GPIO_PIN_7);
		SysCtlDelay(period);
	}

	MAV_Filter();
	prtData();
	uint16_t jet = Peak_Detect();
	Data[0] = jet >> 8;
	Data[1] = jet;
	LED_Buffer_Clear();

	UARTprintf("\n-----------------------------\n");

	for (i = 0; i < nos; i++ )
	{
		//GPIOPinWrite(GPIO_PORTN_BASE, GPIO_PIN_2, GPIO_PIN_2);
		GPIO_PORTH_DATA_R = 0x80;
		SysCtlDelay(pwidth);
		GPIO_PORTH_DATA_R = 0x00;
		pia = 0;
		//GPIOPinWrite(GPIO_PORTN_BASE, GPIO_PIN_2, 0);
		while(!(GPIOPinRead(GPIO_PORTL_BASE,GPIO_PIN_3) == GPIO_PIN_3))
		{
			pia++;
			if(pia > 100)
			{
				break;
			}
		}
		SysCtlDelay(20);
	    ADCProcessorTrigger(ADC0_BASE, 3);   /// ADC 1 for bulk.... code changed to bulk tiggered dedx
	    while(!ADCIntStatus(ADC0_BASE, 3, false))
	    {
	    }
	    ADCIntClear(ADC0_BASE, 3);
	    ADCSequenceDataGet(ADC0_BASE, 3, lt);
	    LED_data[lt[0]]++;
		GPIOPinWrite(GPIO_PORTK_BASE, GPIO_PIN_5, 0);
		SysCtlDelay(100);
		GPIOPinWrite(GPIO_PORTK_BASE, GPIO_PIN_5, GPIO_PIN_5);
		GPIOPinWrite(GPIO_PORTD_BASE, GPIO_PIN_7, 0);
		SysCtlDelay(100);
		GPIOPinWrite(GPIO_PORTD_BASE, GPIO_PIN_7, GPIO_PIN_7);
		SysCtlDelay(period);
	}
	MAV_Filter();
	jet = Peak_Detect();
	Data[2] = jet >> 8;
	Data[3] = jet;
	LED_Buffer_Clear();

}

void LED_Buffer_Clear(void)
{
	for (i = 0; i < 4096; i++)
	{
		LED_data[i] = 0;
	}
}

void SoftLEDConfig(void)
{
	SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOH);
	SysCtlPeripheralEnable(SYSCTL_PERIPH_GPION);
	GPIOPinTypeGPIOOutput(GPIO_PORTH_BASE, GPIO_PIN_7);
	GPIOPinTypeGPIOOutput(GPIO_PORTN_BASE, GPIO_PIN_2);
	GPIOPadConfigSet(GPIO_PORTH_BASE, GPIO_PIN_7, GPIO_STRENGTH_4MA, GPIO_PIN_TYPE_STD);
	GPIOPadConfigSet(GPIO_PORTN_BASE, GPIO_PIN_2, GPIO_STRENGTH_4MA, GPIO_PIN_TYPE_STD);
}

void MAV_Filter(void)
{
	for(i = 0; i<4096; i++ )
	{
		LED_data[i] = (LED_data[i] + LED_data[i+1] + LED_data[i+2] + LED_data[i+3])/4;
	}
}

uint16_t Peak_Detect(void)
{
  uint16_t c, max, index;

  max = LED_data[0];
  index = 0;

  for (c = 1; c < 4096; c++)
  {
    if (LED_data[c] > max)
    {
       index = c;
       max = LED_data[c];
    }
  }
  return index;
}

#endif /* SOFTLED_H_ */
