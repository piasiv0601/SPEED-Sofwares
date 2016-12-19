/*
 * HV_Control.h
 *
 *  Created on: Dec 14, 2015
 *      Author: USER
 */

#ifndef HV_CONTROL_H_
#define HV_CONTROL_H_

uint16_t Config_ST = 28672 ,Config_ST_1 =24576,RAMP_UP = 100,Current_ADC_Value = 0,DAC_VALUE;
uint32_t ADC_VALUE[1],Set_HV_Value = 1000;
uint16_t  Error_HV = 0 ,Current_DAC_Value = 0;
uint8_t Config_End = 0;

void HV_CONTROL(uint32_t HV);
extern void BLUELED_Blink(void);
extern void Configure_ADC_HV(void);
extern void Configure_ADC_SCI(void);
uint16_t GET_HV_VOLTAGE(void);

uint16_t GET_HV_VOLTAGE(void)
{

	uint16_t HV_Value;
	Configure_ADC_HV();
	SysCtlDelay(100);
	ADCProcessorTrigger(ADC0_BASE, 3);
	while(!ADCIntStatus(ADC0_BASE, 3, false))
	{
	}
	ADCIntClear(ADC0_BASE, 3);
	ADCSequenceDataGet(ADC0_BASE, 3, ADC_VALUE);
	HV_Value = (uint16_t)((ADC_VALUE[0]*1000*3.3)/4096) ;
	SysCtlDelay(100);
	Configure_ADC_SCI();
	//HV_Value = (((106*ADC_VALUE[0]) - 487 )/100) - 170;
	return HV_Value ;

}

void DAC_Config(void)
{
	//Configure_ADC_HV();
	if ( GET_HV_VOLTAGE() > 100)
	{
		return;
	}
	else
	{
		GPIOPinWrite(GPIO_PORTN_BASE, GPIO_PIN_7, 0);
		SysCtlDelay(1000);
		SSIDataPut(SSI1_BASE, Config_ST);
		SSIDataPut(SSI1_BASE, Config_End);
		SysCtlDelay(1000);
		GPIOPinWrite(GPIO_PORTN_BASE, GPIO_PIN_7, GPIO_PIN_7);
	}
}

void DAC_Value_SET(uint16_t DAC_Value)
{
	uint8_t test,test1;
	BLUELED_Blink();
	GPIOPinWrite(GPIO_PORTN_BASE, GPIO_PIN_7, 0);
	SysCtlDelay(1000);
	SSIDataPut(SSI1_BASE, 0x3F);  // Select Internal ref and power on all DACs
	DAC_Value = DAC_Value << 4 ;
    test = DAC_Value >> 8   ;
    SSIDataPut(SSI1_BASE, test);
    test1 = DAC_Value - ( test << 8 ) ;
    SSIDataPut(SSI1_BASE, test1);
	SysCtlDelay(1000);
	GPIOPinWrite(GPIO_PORTN_BASE, GPIO_PIN_7, GPIO_PIN_7);

}

void HV_Value_Set (uint32_t HV_VOLT)
{

	uint32_t pia = 0,i,temp = 0;
	temp = GET_HV_VOLTAGE();

		if (HV_VOLT > temp)
		{
			if(temp < 50)
			{
				pia = (HV_VOLT)/RAMP_UP ;
				for(i = 0; i < pia; i++)
				{
					Current_DAC_Value = Current_DAC_Value + ((200*RAMP_UP)/100) ;  // 2 for sar and 2.67 for thv
					DAC_Value_SET(Current_DAC_Value);
					SysCtlDelay(SysCtlClockGet()/3);
				}
			}

			else
			{
				Current_DAC_Value = 200*temp/100;
				pia = (HV_VOLT - temp)/RAMP_UP ;
				for(i = 0; i < pia; i++)
				{
					Current_DAC_Value = Current_DAC_Value + ((200*RAMP_UP)/100) ;  // 2 for sar and 2.67 for thv
					DAC_Value_SET(Current_DAC_Value);
					SysCtlDelay(SysCtlClockGet()/3);
				}
			}

		}
		else if (HV_VOLT < temp)
		{
			//pia = (GET_HV_VOLTAGE())/RAMP_UP ;
			Current_DAC_Value = (temp*200)/100;
			pia = (temp - HV_VOLT)/RAMP_UP ;
			for(i = 0; i < pia; i++)
			{
				if (Current_DAC_Value < (2*RAMP_UP))
				{
					DAC_Value_SET(Current_DAC_Value);
				}
				else
				{
					Current_DAC_Value = Current_DAC_Value - ((200*RAMP_UP)/100) ;  // 2 for sar and 2.67 for thv
					DAC_Value_SET(Current_DAC_Value);
					SysCtlDelay(SysCtlClockGet()/3);
				}
			}
		}
		HV_CONTROL(HV_VOLT);
}

void HV_Shutdown(void)
{
	DAC_Value_SET(0);
}

void HV_Switch (uint8_t state)
{
	if(state == 0) // off
	{
		GPIOPinWrite(GPIO_PORTN_BASE, GPIO_PIN_1, GPIO_PIN_1); // Switching off HV side
	}
	else if (state == 1)//on
	{
		GPIOPinWrite(GPIO_PORTN_BASE, GPIO_PIN_1, 0); // Switching on HV side
	}
}

uint16_t Print_ADC_Value(void)
{

	uint32_t ADC_Value[1];
	ADCProcessorTrigger(ADC1_BASE, 3);
	while(!ADCIntStatus(ADC1_BASE, 3, false))
	{
	}
	ADCIntClear(ADC1_BASE, 3);
	ADCSequenceDataGet(ADC1_BASE, 3, ADC_Value);
	return ADC_Value[0] ;

}

void HV_CONTROL(uint32_t HV)   // may go into infinite loop will fix this
{
	uint32_t DIFF = 0 ;
	UARTprintf("HV_CTL\n");
		if(HV > GET_HV_VOLTAGE())
		{
			DIFF = HV - GET_HV_VOLTAGE();
		}
		else if(HV < GET_HV_VOLTAGE())
		{
			DIFF = GET_HV_VOLTAGE() - HV;
		}
		while(!(DIFF < 1 ))

		{
			if(HV > GET_HV_VOLTAGE())
				{
					Error_HV = (HV - GET_HV_VOLTAGE());
					Current_DAC_Value = Current_DAC_Value + (Error_HV);
					DAC_Value_SET(Current_DAC_Value);
				}

			else if (HV < GET_HV_VOLTAGE())
				{
					Error_HV = (GET_HV_VOLTAGE() - HV);
					Current_DAC_Value = Current_DAC_Value - (Error_HV);
					DAC_Value_SET(Current_DAC_Value);
				}

			DIFF = 0;

			if(HV > GET_HV_VOLTAGE())
			{
				DIFF = HV - GET_HV_VOLTAGE();
			}
			else if(HV < GET_HV_VOLTAGE())
			{
				DIFF = GET_HV_VOLTAGE() - HV;
			}
		}
}



#endif /* HV_CONTROL_H_ */
