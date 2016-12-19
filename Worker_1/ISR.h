/*
 * ISR.h
 *
 *  Created on: Dec 14, 2015
 *      Author: USER
 */



#ifndef ISR_H_
#define ISR_H_

#include <stdbool.h>
#include <stdint.h>
#include "inc/hw_i2c.h"
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
#include "utils/uartstdio.h"
#include "inc/hw_nvic.h"
#include "driverlib/rom.h"
#include "driverlib/timer.h"
#include "driverlib/udma.h"
#include "driverlib/flash.h"
#include "inc/hw_ssi.h"
#include "I2C.h"
#include "Functions.h"
#include "HV_Control.h"
#include "driverlib/systick.h"
#include "inc/hw_nvic.h"


uint8_t OVC_State;   //1 if normal, 0 if OVC

#define BULK_RAMP GPIO_PIN_7
#define BULK_PKDT GPIO_PIN_6
#define BULK_A121 GPIO_PIN_5
#define BULK_VIN  GPIO_PIN_3

#define DEDX_RAMP GPIO_PIN_5
#define DEDX_PKDT GPIO_PIN_7
#define DEDX_A121 GPIO_PIN_3
#define DEDX_VIN  GPIO_PIN_2

#define VETO_A121 GPIO_PIN_3


#define LOOKUP1(x,y) *((uint8_t *)(LOOKUP_BASE+x>>7+y))
#define LOOKUP2(x,y) *((uint8_t *)(LOOKUP2_BASE+x>>7+y))
uint32_t ui32Status;	//Variable for SPI Interrupt
uint8_t j;
uint8_t prev_coincidence,flag1,I2C_Err = 0,IFLAG = 0;


void HK_ISR(void)
{
	TimerIntClear(TIMER2_BASE,TIMER_TIMA_TIMEOUT);
	UARTprintf("Collecting HK data \n");
	HK_Data_Acquire();
	UARTprintf("Done Collecting HK data \n");
	WatchdogIntClear(WATCHDOG0_BASE);
}

void I2C_DATA_PRINT_R(void)
{
	for(pia=0;pia<135;pia++)

		{
			UARTprintf("%d \n",g_ui32DataRx[pia] );
		}
}

void HV_OVC_ISR (void)
{
	GPIOIntClear(GPIO_PORTM_BASE, GPIO_INT_PIN_1);
	UARTprintf("HV OVC Triggered\n");
	if (HV_OVC_ST == 0)
	{
		HV_OVC_ST = 1;
	}
	else if (HV_OVC_ST == 1)
	{
		HV_OVC_ST = 2;
	}
	SysCtlDelay(10);
	HV_Shutdown();
	HV_Switch(0);
}

void Watchdog0IntHandler(void)
{
	UARTprintf("Watchdog Interrupt received!!\n");
	WTCT++;
	while(1)
	{
		//Debugging action must be done here
		REDLED_Blink();
		SysCtlDelay(SysCtlClockGet()/6);
	}
}
void SSI0_IntHandler(void)
{
	  ui32Status = SSIIntStatus(SSI0_BASE, 1);
	  SSIIntClear(SSI0_BASE, ui32Status);
	  SysCtlDelay(10);
	  if(transfer==7)
	  {
		  transfer=0;
		  /*for(i=0;i<60;i++)		//Clearing the buffer after the data transfer
		  {
			  for(j=0;j<56;j++)
			  {
				  *(ptr2+i*56+j)=0;
			  }
		  }*/
		  UARTprintf("Transfer Done \n");
		  //Print_Buffer();
		  return;
	 }
	  DMA_SPI_Transfer_Science(SPI);
}
void dE_dX_INT(void)
{
	UARTprintf("de\n");
	//HWREG(NVIC_ST_CTRL) |= NVIC_ST_CTRL_CLK_SRC | NVIC_ST_CTRL_ENABLE;
	/*
	 * 				Since only one  chain, no need to wait for the other chain
	while((GPIO_PORTD_DATA_R & BULK_A121)!=BULK_A121)
	{
	    if (HWREG(NVIC_ST_CURRENT) < 100)
	    {
	    		GPIO_PORTL_ICR_R|=DEDX_A121;
	    		while(GPIO_PORTK_DATA_R & DEDX_PKDT);
	    		//GPIO_PORTB_DATA_R &= (~(DUMP1|DUMP2));//GPIOPinWrite(GPIO_PORTB_BASE,DUMP1|DUMP2,0);
				GPIO_PORTK_DATA_R &= (~(DEDX_RAMP));
				GPIO_PORTD_DATA_R &= (~(BULK_RAMP));
	    		while((GPIO_PORTK_DATA_R & DEDX_PKDT)!= DEDX_PKDT);
	    		while((GPIO_PORTD_DATA_R & BULK_PKDT)!= BULK_PKDT);
	    		GPIO_PORTK_DATA_R |= DEDX_RAMP;
	    		GPIO_PORTD_DATA_R |= BULK_RAMP;
	    		HWREG(NVIC_ST_CURRENT) = 0 ;
	    		HWREG(NVIC_ST_CTRL) &= ~(NVIC_ST_CTRL_INTEN);
	    		return ;
	       	}
	   }	*/
	   GPIO_PORTL_ICR_R|=DEDX_A121;
	   //flag1=prev_coincidence;
	   /*
	    * 								No coincidence and anticoincidence can be done-Single chain
	   if((GPIO_PORTB_RIS_R & VETO_A121)==VETO_A121)
	    {
	    	prev_coincidence=0;
	    	anticoincidence++;
	    }
	    else
	    {
	    	prev_coincidence=1;
	    }*/
	    /*			Again, no coincidence or anticoincidence
	   	 if(!(flag1))
	    {
	    	ulADC0_Value=ADC0_SSFIFO3_R;
			ulADC1_Value=ADC1_SSFIFO3_R;
	    	while((GPIO_PORTK_DATA_R & DEDX_PKDT) | (GPIO_PORTD_DATA_R & BULK_PKDT));

	    	ADC0_PSSI_R = 1<<3;  // Processor trigger for ADC
	    	ADC1_PSSI_R = 1<<3;
	    	if(science_data_mode==1)
	    	{
	    		x=ulADC0_Value>>9;
	    		y=ulADC1_Value>>10;
	    		temp=x<<2+y;
	    		trial2=ptr1+frame*56;
	    		*(trial2+temp)=*(trial2+temp)+1;
	    	}

	    	x= ulADC0_Value >> 5;	  //  Instead using Right shift by 5. This reduces the number of bins to 128
	    	y= ulADC1_Value >> 5;

	    	temp = LOOKUP2(x,y);		// Lookup table in the header.h file

	    	//trial=(uint8_t *)(ptr1+frame*56);
	    	//trial2=(uint16_t *)trial;
	    	trial2=ptr1+frame*56;
	    	*(trial2+temp)=*(trial2+temp)+1;
	    	//*(ptr1 +frame*35+ temp) = (*(ptr1 +frame*35+ temp))+1;
	    	// incrementing the corresponding bin

	    	while(!ADCIntStatus(ADC0_BASE, 3, false));
	    	while(!ADCIntStatus(ADC1_BASE, 3, false));
	    	GPIO_PORTD_ICR_R|=BULK_A121;
	    	//Dumping Sequence
			GPIO_PORTK_DATA_R &= (~(DEDX_RAMP));
			GPIO_PORTD_DATA_R &= (~(BULK_RAMP));
    		while((GPIO_PORTK_DATA_R & DEDX_PKDT)!= DEDX_PKDT);
    		while((GPIO_PORTD_DATA_R & BULK_PKDT)!= BULK_PKDT);
    		GPIO_PORTK_DATA_R |= DEDX_RAMP;
    		GPIO_PORTD_DATA_R |= BULK_RAMP;
	        GPIO_PORTB_ICR_R|=VETO_A121;
	    }
	    */
	    //else 	No coincidence or anticoincidence
	    {
	    	//Coincidence
	    	ulADC0_Value=ADC0_SSFIFO3_R;
			ulADC1_Value=ADC1_SSFIFO3_R;

			while(GPIO_PORTK_DATA_R & DEDX_PKDT);
			//while((GPIO_PORTK_DATA_R & DEDX_PKDT) | (GPIO_PORTD_DATA_R & BULK_PKDT));

			ADC0_PSSI_R = 1<<3;  // Processor trigger for ADC
			ADC1_PSSI_R = 1<<3;
			if(science_data_mode==1)
			{
				coincidence++;
				//Data acquistion sequence for the purpose of demonstration - Must be removed
				x=ulADC0_Value >> 9;
				y=ulADC1_Value >> 10;
				temp=x<<2+y;
				trial2=ptr1+frame*56;
				*(trial2+temp)=*(trial2+temp)+1;
				while(!ADCIntStatus(ADC0_BASE, 3, false));
				while(!ADCIntStatus(ADC1_BASE, 3, false));
				GPIO_PORTL_ICR_R|=DEDX_A121;
				//Dumping Sequence
				GPIO_PORTK_DATA_R &= (~(DEDX_RAMP));
				GPIO_PORTD_DATA_R &= (~(BULK_RAMP));
				while((GPIO_PORTK_DATA_R & DEDX_PKDT)!= DEDX_PKDT);
				//while((GPIO_PORTD_DATA_R & BULK_PKDT)!= BULK_PKDT);	Comments to be removed if double chain is used
				GPIO_PORTK_DATA_R |= DEDX_RAMP;
				GPIO_PORTD_DATA_R |= BULK_RAMP;
		        HWREG(NVIC_ST_CURRENT) = 0 ;
		        HWREG(NVIC_ST_CTRL) &= ~(NVIC_ST_CTRL_INTEN);
				return;

			 }
			x= ulADC0_Value >> 5;	  //  Instead using Right shift by 5. This reduces the number of bins to 128
			y= ulADC1_Value >> 5;

			temp = LOOKUP1(x,y);		// Lookup table in the header.h file

	    	//trial=(uint8_t *)ptr1+frame*112;
	    	trial2=ptr1+frame*56;
	    	*(trial2+temp)=*(trial2+temp)+1;
			//*(ptr1 +frame*35+ temp) = (*(ptr1 +frame*35+ temp))+1;	// incrementing the corresponding bin

			while(!ADCIntStatus(ADC0_BASE, 3, false));
			while(!ADCIntStatus(ADC1_BASE, 3, false));
	    	GPIO_PORTD_ICR_R|=BULK_A121;
			//Dumping Sequence
			GPIO_PORTK_DATA_R &= (~(DEDX_RAMP));
			GPIO_PORTD_DATA_R &= (~(BULK_RAMP));
    		while((GPIO_PORTK_DATA_R & DEDX_PKDT)!= DEDX_PKDT);
    		//while((GPIO_PORTD_DATA_R & BULK_PKDT)!= BULK_PKDT);  Comments to be removed if multiple chains are used
    		GPIO_PORTK_DATA_R |= DEDX_RAMP;
    		GPIO_PORTD_DATA_R |= BULK_RAMP;
	        HWREG(NVIC_ST_CURRENT) = 0 ;
	        HWREG(NVIC_ST_CTRL) &= ~(NVIC_ST_CTRL_INTEN);
	    }
}



void Bulk_INT(void)
{
	UARTprintf("bu\n");
	HWREG(NVIC_ST_CTRL) |= NVIC_ST_CTRL_CLK_SRC | NVIC_ST_CTRL_ENABLE;
	while((GPIO_PORTL_DATA_R & DEDX_A121)!=DEDX_A121)
	{
	    if (HWREG(NVIC_ST_CURRENT) < 100)
	    {
	    		GPIO_PORTD_ICR_R|=BULK_A121;
	    		while(GPIO_PORTD_DATA_R & BULK_PKDT);
	    		//GPIO_PORTB_DATA_R &= (~(DUMP1|DUMP2));//GPIOPinWrite(GPIO_PORTB_BASE,DUMP1|DUMP2,0);
				GPIO_PORTK_DATA_R &= (~(DEDX_RAMP));
				GPIO_PORTD_DATA_R &= (~(BULK_RAMP));
	    		while((GPIO_PORTK_DATA_R & DEDX_PKDT)!= DEDX_PKDT);
	    		while((GPIO_PORTD_DATA_R & BULK_PKDT)!= BULK_PKDT);
	    		GPIO_PORTK_DATA_R |= DEDX_RAMP;
	    		GPIO_PORTD_DATA_R |= BULK_RAMP;
	    		HWREG(NVIC_ST_CURRENT) = 0 ;
	    		HWREG(NVIC_ST_CTRL) &= ~(NVIC_ST_CTRL_INTEN);
	    		return ;
	       	}
	   }
	   GPIO_PORTD_ICR_R|=BULK_A121;
	   flag1=prev_coincidence;
	   if((GPIO_PORTB_RIS_R & VETO_A121)==VETO_A121)
	    {
	    	prev_coincidence=0;
	    	anticoincidence++;
	    }
	   else
		   prev_coincidence=1;
	   if(!flag1)
	   {
	    	ulADC0_Value=ADC0_SSFIFO3_R;
			ulADC1_Value=ADC1_SSFIFO3_R;
	    	while((GPIO_PORTK_DATA_R & DEDX_PKDT) | (GPIO_PORTD_DATA_R & BULK_PKDT));

	    	ADC0_PSSI_R = 1<<3;  // Processor trigger for ADC
	    	ADC1_PSSI_R = 1<<3;

	    	if(science_data_mode==1)
	    	{
	    		x=ulADC0_Value>>9;
	    		y=ulADC1_Value>>10;
	    		temp=x<<2+y;
	    		trial2=ptr1+frame*56;
	    		*(trial2+temp)=*(trial2+temp)+1;
		    	while(!ADCIntStatus(ADC0_BASE, 3, false));
		    	while(!ADCIntStatus(ADC1_BASE, 3, false));
		    	GPIO_PORTL_ICR_R|=DEDX_A121;
		    	//Dumping Sequence
				GPIO_PORTK_DATA_R &= (~(DEDX_RAMP));
				GPIO_PORTD_DATA_R &= (~(BULK_RAMP));
	    		while((GPIO_PORTK_DATA_R & DEDX_PKDT)!= DEDX_PKDT);
	    		while((GPIO_PORTD_DATA_R & BULK_PKDT)!= BULK_PKDT);
		        GPIO_PORTB_ICR_R|=VETO_A121;
	    		GPIO_PORTK_DATA_R |= DEDX_RAMP;
	    		GPIO_PORTD_DATA_R |= BULK_RAMP;
	    		return;
	    	}
	    	x= ulADC0_Value >> 5;	  //  Instead using Right shift by 5. This reduces the number of bins to 128
	    	y= ulADC1_Value >> 5;

	    	temp = LOOKUP2(x,y);		// Lookup table in the header.h file
	    	//trial=(uint8_t *)ptr1+frame*112;
	    	trial2=ptr1+frame*56;
	    	*(trial2+temp)=*(trial2+temp)+1;
	    	//*(ptr1 +trial+ temp) = (*(ptr1 +frame*35+ temp))+1;	// incrementing the corresponding bin

	    	while(!ADCIntStatus(ADC0_BASE, 3, false));
	    	while(!ADCIntStatus(ADC1_BASE, 3, false));
	    	GPIO_PORTL_ICR_R|=DEDX_A121;
	    	//Dumping Sequence
			GPIO_PORTK_DATA_R &= (~(DEDX_RAMP));
			GPIO_PORTD_DATA_R &= (~(BULK_RAMP));
    		while((GPIO_PORTK_DATA_R & DEDX_PKDT)!= DEDX_PKDT);
    		while((GPIO_PORTD_DATA_R & BULK_PKDT)!= BULK_PKDT);
	        GPIO_PORTB_ICR_R|=VETO_A121;
    		GPIO_PORTK_DATA_R |= DEDX_RAMP;
    		GPIO_PORTD_DATA_R |= BULK_RAMP;
	    }
	    else
	    {
	    	//Coincidence
	    	ulADC0_Value=ADC0_SSFIFO3_R;
			ulADC1_Value=ADC1_SSFIFO3_R;

			while((GPIO_PORTK_DATA_R & DEDX_PKDT) | (GPIO_PORTD_DATA_R & BULK_PKDT));

			ADC0_PSSI_R = 1<<3;  // Processor trigger for ADC
			ADC1_PSSI_R = 1<<3;

			x= ulADC0_Value >> 5;	  //  Instead using Right shift by 5. This reduces the number of bins to 128
			y= ulADC1_Value >> 5;
			if(science_data_mode==1)
			{
				coincidence++;
				while(!ADCIntStatus(ADC0_BASE, 3, false));
							while(!ADCIntStatus(ADC1_BASE, 3, false));
					    	GPIO_PORTL_ICR_R|=DEDX_A121;
							//Dumping Sequence
							GPIO_PORTK_DATA_R &= (~(DEDX_RAMP));
							GPIO_PORTD_DATA_R &= (~(BULK_RAMP));
				    		while((GPIO_PORTK_DATA_R & DEDX_PKDT)!= DEDX_PKDT);
				    		while((GPIO_PORTD_DATA_R & BULK_PKDT)!= BULK_PKDT);
				    		GPIO_PORTK_DATA_R |= DEDX_RAMP;
				    		GPIO_PORTD_DATA_R |= BULK_RAMP;
					        HWREG(NVIC_ST_CURRENT) = 0 ;
					        HWREG(NVIC_ST_CTRL) &= ~(NVIC_ST_CTRL_INTEN);
					        return;
			}
			temp = LOOKUP1(x,y);		// Lookup table in the header.h file

	    	//trial=(uint8_t *)ptr1+frame*112;
	    	trial2=ptr1+frame*56;
	    	*(trial2+temp)=*(trial2+temp)+1;
		//	*(ptr1 +buffer+ temp) = (*(ptr1 +frame*35+ temp))+1;	// incrementing the corresponding bin

			while(!ADCIntStatus(ADC0_BASE, 3, false));
			while(!ADCIntStatus(ADC1_BASE, 3, false));
	    	GPIO_PORTL_ICR_R|=DEDX_A121;
			//Dumping Sequence
			GPIO_PORTK_DATA_R &= (~(DEDX_RAMP));
			GPIO_PORTD_DATA_R &= (~(BULK_RAMP));
    		while((GPIO_PORTK_DATA_R & DEDX_PKDT)!= DEDX_PKDT);
    		while((GPIO_PORTD_DATA_R & BULK_PKDT)!= BULK_PKDT);
    		GPIO_PORTK_DATA_R |= DEDX_RAMP;
    		GPIO_PORTD_DATA_R |= BULK_RAMP;
	        HWREG(NVIC_ST_CURRENT) = 0 ;
	        HWREG(NVIC_ST_CTRL) &= ~(NVIC_ST_CTRL_INTEN);
	    }
}



void I2C0SlaveIntHandler (void)
{
	uint16_t Time = 0;
	TC_SIZE = 11; // Should Be 11
	I2CSlaveIntClear(I2C1_BASE); // Clearing the interrupt

	for(pia = 0 ; pia < TC_SIZE ; pia++)  // getting data
	{
		while(!(I2CSlaveStatus(I2C1_BASE) & I2C_SLAVE_ACT_RREQ))
					    {
							Time++;
							if(Time > 100000)
							{
								I2C_Err ++;
								I2C_Decode_TC();
								I2C_Encode_Telemetry(0);
								Buffer_Clear(); // to ensure nothing is executed
								IFLAG=1;
								break;
							}
					    }
		Time = 0;
		g_ui32DataRx[pia] = I2CSlaveDataGet(I2C1_BASE);
		if(pia == 1)
			{
				if((g_ui32DataRx[1] & 0x10))
					{
						TC_SIZE = 135 ; // long or short TC
					}
				else
				{
					TC_SIZE = 11;
				}
			}

	}


	UARTprintf(" I2C Command Recieved .. \n");
	CRC_Value = 0;

	CRC_Value = CRC_Value |  g_ui32DataRx[9] ;
	CRC_Value = CRC_Value << 8;
	CRC_Value = CRC_Value |  g_ui32DataRx[10] ;
	CRC_Value1 = CRC_16_Calc(g_ui32DataRx,9);
	UARTprintf("%d --- %d \n",CRC_Value1,CRC_Value);
	if(!(CRC_Value == CRC_Value1))
	{
		if (IFLAG == 0)
		{
			I2C_Decode_TC();
			I2C_Encode_Telemetry(0); // NACK alone
			UARTprintf("CRC FAIL \n");
			Buffer_Clear(); // to ensure nothing is executed
		}
		else
		{
			IFLAG = 0;
		}
	}
	UARTprintf("CRC Passed \n");
	I2C_DATA_PRINT_R();
	I2C_Decode_TC();
    I2CSlaveIntClear(I2C1_BASE);
    I2CSlaveIntClearEx(I2C1_BASE,I2C_SLAVE_INT_DATA);
    IntPendClear(INT_I2C1);
    I2C_FLAG = 1;
    UARTprintf("I2C Transaction Complete \n");
    Buffer_Clear();  // should be uncommented finally
}


void Fail_Safe(void)
{
	TimerIntClear(TIMER1_BASE,TIMER_TIMB_TIMEOUT);
	temp32       = HWREG(Fail_Safe_TM);
	FlashErase(Fail_Safe_TM);
	if (temp32 < 100000000)
	{
		FL_TEMP[0] = temp32 + 1;
		FlashProgram(FL_TEMP, Fail_Safe_TM, 4);
	}
	else
	{
		FL_TEMP[0] = 0;
		FlashProgram(FL_TEMP, Fail_Safe_TM, 4);
		mode_change = 3;
	}

}

void
SCI_Timer_INT_Handler(void)
{
	uint8_t txt = 0;
	TimerIntClear(TIMER0_BASE,TIMER_TIMA_TIMEOUT);

	/*
	if(science_data_mode==0)
	{
		trial=(uint8_t *)(ptr1+frame*56);
		trial=trial+96;
		fast_counts=(uint32_t *)trial;
		*fast_counts=anticoincidence;
		anticoincidence=0;
		fast_counts++;
		*fast_counts=TimerValueGet(WTIMER1_BASE,TIMER_A);	//dEdX Fast Chain counts
		fast_counts++;
		*fast_counts=TimerValueGet(WTIMER4_BASE,TIMER_A);	//Bulk Fast Chain counts
		fast_counts++;
		*fast_counts=TimerValueGet(WTIMER3_BASE,TIMER_A);	//Veto Fast Chain Counts
	}

	else if(science_data_mode==1)
	{
		trial=(uint8_t *)(ptr1+frame*56);
		trial=trial+64;
		fast_counts=(uint32_t *)trial;
		*fast_counts=coincidence;
		coincidence=0;
		fast_counts++;
		*fast_counts=TimerValueGet(WTIMER1_BASE,TIMER_A);	//dEdX Fast Chain counts
		fast_counts++;
		*fast_counts=TimerValueGet(WTIMER4_BASE,TIMER_A);	//Bulk Fast Chain counts
		fast_counts++;
		*fast_counts=TimerValueGet(WTIMER3_BASE,TIMER_A);	//Veto Fast Chain Counts
	}
	*/

	WTIMER1_TAV_R=0;	//Resetting the timers
	WTIMER1_TAPS_R=0;

	WTIMER4_TAV_R=0;
	WTIMER4_TAPS_R=0;

	WTIMER3_TAV_R=0;
	WTIMER3_TAPS_R=0;

	if(frame==60)
	{
		UARTprintf("Sending Science Raw data - Normal mode\n");
		//GPIOPinWrite(GPIO_PORTC_BASE,GPIO_PIN_7,GPIO_PIN_7);
		//SysCtlDelay(1000);
		//GPIOPinWrite(GPIO_PORTC_BASE,GPIO_PIN_7,0);
		frame=0;
		//trial2=ptr1;	//Exchanging the ping-pong buffers
		//ptr1=ptr2;
		//ptr2=trial2;
		BLUELED_Blink();
		UARTprintf("Packet Sequence Count %d \n",packet_sequence_count);
		SSIDataPut(SSI0_BASE,txt);
		while(SSIBusy(SSI0_BASE));
		SSIDataPut(SSI0_BASE,packet_sequence_count);	//Sending the Packet sequence count and science data mode
		//SSIDataPut(SSI0_BASE,1000);	//Sending the Packet sequence count and science data mode
		while(SSIBusy(SSI0_BASE));
		//SSIDataPut(SSI0_BASE,0);
		//while(SSIBusy(SSI0_BASE));
		//SSIDataPut(SSI0_BASE,science_data_mode);
		SSIDataPut(SSI0_BASE,science_data_mode);

		while(SSIBusy(SSI0_BASE));
		UARTprintf("Started Transfer \n");
		if(packet_sequence_count == 10)
		{
			packet_sequence_count = 0;
		}

		else
		{
			packet_sequence_count++;
		}
		DMA_SPI_Transfer_Science(SPI);
		return;
	}
	frame++;
	SysCtlDelay(10);

}

void Test (void)
{
	UARTprintf("uDMA error \n ");
	while(1);
}

void HV_Timer_INT_Handler(void)
{
	TimerIntClear(TIMER1_BASE,TIMER_TIMA_TIMEOUT);
	if(HV_OVC_ST == 1)
	{
		HV_Switch(1);
		HV_Value_Set(high_voltage);
		HV_OVC_ST = 3;
		IntDisable(INT_TIMER1A);
		TimerDisable(TIMER1_BASE,TIMER_A);
		TimerIntDisable(TIMER1_BASE,TIMER_TIMA_TIMEOUT);
	}
	else
	{
		IntDisable(INT_TIMER1A);
		TimerDisable(TIMER1_BASE,TIMER_A);
		TimerIntDisable(TIMER1_BASE,TIMER_TIMA_TIMEOUT);
	}

}


#endif /* ISR_H_ */
