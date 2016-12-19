/*
 * I2C.h
 *
 *  Created on: Dec 14, 2015
 *      Author: USER
 */




#ifndef I2C_H_
#define I2C_H_


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
#include "ISR.h"
#include "Functions.h"



#define TOPBIT16                (1 << 15)
#define POLYNOMIAL16            0x1021
#define POLYNOMIAL              0xD8
#define CRC16                   0x11021   // 0x8010
#define SLAVE_ADDRESS           0x20
#define NACK_TC                 0x02
#define NACK_CRC                0x01
#define ACK                     0xA0

uint16_t CRC_16_Calc(const unsigned char message[], unsigned int nBytes);
uint16_t CRC_16_Calc1(const uint8_t *data, uint16_t size);
void I2C_Write (int size);
void I2C_Encode_Telemetry(int cmd);
void Buffer_Clear(void);
void I2C_Decode_TC(void);

uint8_t   g_ui32DataRx[140], g_ui32DataTx[140],  dma=0 , dma1=0;
uint8_t   Command, TCPSC, TMPSC, Service, ST_ADDR = 0, ST_ADDR1 = 0, ED_ADDR = 0, ED_ADDR1 = 0, TM_SIZE = 0 , TM_SIZE1 = 0, TC_SIZE = 11;
uint16_t  CRC_Value = 0, CRC_Value1 = 0;
uint32_t  ST_ADDR_32 = 0, ED_ADDR_32 = 0, FLSH_RD[256], FLSH_WR[256] , Buffer = 0,FL_TEMP[3];;
uint32_t  pui32DataTx[3];
uint32_t  pui32DataRx[3];
int pia,siv,I2C_FLAG;
int Flag=0,i=0,Flag1=1;
int result;

uint8_t BF[60];
uint32_t TP[1],TP1[1];
uint32_t temp32 = 0;
uint16_t temp16 = 0, HV_MAX = 0, HV_MIN = 0;
uint8_t  temp8  = 0;
uint8_t Data[140];
uint8_t WTCT=0;
uint8_t HV_OVC_ST = 0;

void LEDAcquire(uint32_t pwidth, uint32_t period,uint32_t nos);
void FLASH_BUFFER_WRITE(void);

int uc_core_temp(void);

void dummy (void)
{
			Buffer_Clear();
			ST_ADDR_32 = 49152 ;
			FLASH_BUFFER_WRITE();
			FlashProgram(FLSH_WR , ST_ADDR_32 , 128);
}

void HK_Data_Acquire(void)
{
    SysCtlPeripheralEnable(SYSCTL_PERIPH_ADC0);
    SysCtlPeripheralEnable(SYSCTL_PERIPH_ADC1);
    SysCtlDelay(100);

    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOB);
    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOE);
	GPIOPinTypeADC(GPIO_PORTE_BASE,GPIO_PIN_0|GPIO_PIN_1|GPIO_PIN_4|GPIO_PIN_5|GPIO_PIN_6|GPIO_PIN_7);
	GPIOPinTypeADC(GPIO_PORTB_BASE,GPIO_PIN_4|GPIO_PIN_5);
	SysCtlDelay(100);
    ADCSequenceConfigure(ADC0_BASE, 3, ADC_TRIGGER_PROCESSOR, 0);
    ADCSequenceConfigure(ADC1_BASE, 3, ADC_TRIGGER_PROCESSOR, 0);

	ADCSequenceStepConfigure(ADC0_BASE, 3, 0, ADC_CTL_CH2 | ADC_CTL_IE |
	                                 ADC_CTL_END);    //Temp2
	ADCSequenceStepConfigure(ADC1_BASE, 3, 0, ADC_CTL_CH3 | ADC_CTL_IE |
		                                 ADC_CTL_END);  // Temp3
	ADCHardwareOversampleConfigure(ADC0_BASE,8);
	ADCHardwareOversampleConfigure(ADC1_BASE,8);
	ADCSequenceEnable(ADC0_BASE, 3);
	ADCSequenceEnable(ADC1_BASE, 3);
	ADCProcessorTrigger(ADC0_BASE, 3);
	ADCProcessorTrigger(ADC1_BASE, 3);

    while(!ADCIntStatus(ADC0_BASE, 3, false))
    {
    }
    ADCIntClear(ADC0_BASE, 3);
    while(!ADCIntStatus(ADC1_BASE, 3, false))
    {
    }
    ADCIntClear(ADC1_BASE, 3);
    ADCSequenceDataGet(ADC0_BASE, 3, TP);
    ADCSequenceDataGet(ADC1_BASE, 3, TP1);

    TP [0]  = (TP[0]*3300)/4096;
    TP [0]  = 2414 - ((TP[0]*909)/1000);
    TP [0]  = (TP[0]/10);
    TP1 [0]  = (TP[0]*3300)/4096;
    TP1 [0]  = 2414 - ((TP1[0]*909)/1000);
    TP1 [0]  = (TP1[0]/10);
    UARTprintf("Bulk and Dedx temperature : %d --- %d \n",(TP[0]-50),(TP[1]-50));
    if(TP[0] > BF[0])
    	{
    		if(TP[0] > BF[1])
    			{
    				BF[1] = TP[0] ;
    			}
    		else
    			{
    				BF[2] = TP[0] ;
    			}
    	}

    else
    	{
    		BF[0] = TP[0];
    	}

    if(TP1[0] > BF[3])
        	{
        		if(TP1[0] > BF[4])
        			{
        				BF[4] = TP1[0] ;
        			}
        		else
        			{
        				BF[5] = TP1[0] ;
        			}
        	}

     else
       	{
       		BF[3] = TP1[0];
       	}

///////////////////////////////////////////////////////////////////////////////////
	ADCSequenceStepConfigure(ADC0_BASE, 3, 0, ADC_CTL_CH14 | ADC_CTL_IE |
	                                 ADC_CTL_END);
	ADCSequenceStepConfigure(ADC1_BASE, 3, 0, ADC_CTL_CH12 | ADC_CTL_IE |
		                                 ADC_CTL_END);
	ADCSequenceEnable(ADC0_BASE, 3);
	ADCSequenceEnable(ADC1_BASE, 3);
	ADCProcessorTrigger(ADC0_BASE, 3);
	ADCProcessorTrigger(ADC1_BASE, 3);
    while(!ADCIntStatus(ADC0_BASE, 3, false))
    {
    }
    ADCIntClear(ADC0_BASE, 3);
    while(!ADCIntStatus(ADC1_BASE, 3, false))
    {
    }
    ADCIntClear(ADC1_BASE, 3);
    ADCSequenceDataGet(ADC0_BASE, 3, TP);
    ADCSequenceDataGet(ADC1_BASE, 3, TP1);
    TP [0] = (TP[0] * 3300)/4096;
    TP [0] = (TP[0]/100);
    TP1 [0] = (TP1[0] * 3300)/4096;
    TP1 [0] = (TP1[0]/100);
    UARTprintf("PMT Thermistor Volatge value = %d \n ",TP[0]);
    if(TP[0] > BF[6])
    	{
    		if(TP[0] > BF[7])
    			{
    				BF[7] = TP[0] ;
    			}
    		else
    			{
    				BF[8] = TP[0] ;
    			}
    	}

    else
    	{
    		BF[6] = TP[0];
    	}

    if(TP1[0] > BF[9])
        	{
        		if(TP1[0] > BF[10])
        			{
        				BF[10] = TP1[0] ;
        			}
        		else
        			{
        				BF[11] = TP1[0] ;
        			}
        	}

     else
       	{
       		BF[9] = TP1[0];
       	}

//////////////////////////////////////////////////////////////////////////////

	ADCSequenceStepConfigure(ADC0_BASE, 3, 0, ADC_CTL_CH15 | ADC_CTL_IE |
	                                 ADC_CTL_END);
	ADCSequenceStepConfigure(ADC1_BASE, 3, 0, ADC_CTL_CH8 | ADC_CTL_IE |
		                                 ADC_CTL_END);   //DAC
	ADCSequenceEnable(ADC0_BASE, 3);
	ADCSequenceEnable(ADC1_BASE, 3);
	ADCProcessorTrigger(ADC0_BASE, 3);
	ADCProcessorTrigger(ADC1_BASE, 3);
    while(!ADCIntStatus(ADC0_BASE, 3, false))
    {
    }
    ADCIntClear(ADC0_BASE, 3);
    while(!ADCIntStatus(ADC1_BASE, 3, false))
    {
    }
    ADCIntClear(ADC1_BASE, 3);
    ADCSequenceDataGet(ADC0_BASE, 3, TP);
    ADCSequenceDataGet(ADC1_BASE, 3, TP1);
    UARTprintf("TEMP %d--------DAC %d\n",TP[0],TP1[0]);
    TP [0]  = (TP[0]*3300)/4096;
    TP [0]  = 2414 - ((TP[0]*909)/1000);
    TP [0]  = (TP[0]/10);
    TP1 [0] = (TP1[0] * 3300)/4096;
    TP1 [0] = (TP1[0]/100);
    UARTprintf("TEMP %d--------DAC %d\n",TP[0],TP1[0]);
    if(TP[0] > BF[12])
    	{
    		if(TP[0] > BF[13])
    			{
    				BF[13] = TP[0] ;
    			}
    		else
    			{
    				BF[14] = TP[0] ;
    			}
    	}

    else
    	{
    		BF[12] = TP[0];
    	}

    if(TP1[0] > BF[15])
        	{
        		if(TP1[0] > BF[16])
        			{
        				BF[16] = TP1[0] ;
        			}
        		else
        			{
        				BF[17] = TP1[0] ;
        			}
        	}

     else
       	{
       		BF[15] = TP1[0];
       	}
///////////////////////////////////////////////////////////////////////

	ADCSequenceStepConfigure(ADC0_BASE, 3, 0, ADC_CTL_CH10 | ADC_CTL_IE |
	                                 ADC_CTL_END);// 5V A121
	ADCSequenceStepConfigure(ADC1_BASE, 3, 0, ADC_CTL_CH11 | ADC_CTL_IE |
	                                 ADC_CTL_END);// -5 PH300
	ADCSequenceEnable(ADC0_BASE, 3);
	ADCSequenceEnable(ADC1_BASE, 3);

	ADCProcessorTrigger(ADC0_BASE, 3);
	ADCProcessorTrigger(ADC1_BASE, 3);
    while(!ADCIntStatus(ADC0_BASE, 3, false))
    {
    }
    ADCIntClear(ADC0_BASE, 3);
    while(!ADCIntStatus(ADC1_BASE, 3, false))
    {
    }
    ADCIntClear(ADC1_BASE, 3);
    ADCSequenceDataGet(ADC0_BASE, 3, TP);
    ADCSequenceDataGet(ADC1_BASE, 3, TP1);
    UARTprintf("5V and -5V readings are %d----%d\n",TP[0],TP1[0]);
    TP [0] =  (TP[0]*3300)/4095;
    TP [0] =  (TP[0]*7300*125)/22 ;
    TP [0] =  TP[0]/1000000;
    TP [0] =  TP[0];
    TP1 [0] =  (TP1[0]*3300)/4095;
    TP1 [0] =  (TP1[0]*5100)/22 ;
    TP1 [0] =  TP1[0]/10000;
    UARTprintf("5V and -5V readings are %d----%d\n",TP[0],TP1[0]);

    if(TP[0] > BF[18])
    	{
    		if(TP[0] > BF[19])
    			{
    				BF[19] = TP[0] ;
    			}
    		else
    			{
    				BF[20] = TP[0] ;
    			}
    	}

    else
    	{
    		BF[18] = TP[0];
    	}

    if(TP1[0] > BF[21])
        	{
        		if(TP1[0] > BF[22])
        			{
        				BF[22] = TP1[0] ;
        			}
        		else
        			{
        				BF[23] = TP1[0] ;
        			}
        	}

     else
       	{
       		BF[21] = TP1[0];
       	}
///////////////////////////////////////////////////////////////////////////////


	ADCSequenceStepConfigure(ADC0_BASE, 3, 0, ADC_CTL_CH21 | ADC_CTL_IE |
	                                 ADC_CTL_END); //12V
	ADCSequenceStepConfigure(ADC1_BASE, 3, 0, ADC_CTL_CH20 | ADC_CTL_IE |
		                                 ADC_CTL_END);//  HV 5V
	ADCSequenceEnable(ADC0_BASE, 3);
	ADCSequenceEnable(ADC1_BASE, 3);
	ADCProcessorTrigger(ADC0_BASE, 3);
	ADCProcessorTrigger(ADC1_BASE, 3);
    while(!ADCIntStatus(ADC0_BASE, 3, false))
    {
    }
    ADCIntClear(ADC0_BASE, 3);
    while(!ADCIntStatus(ADC1_BASE, 3, false))
    {
    }
    ADCIntClear(ADC1_BASE, 3);
    ADCSequenceDataGet(ADC0_BASE, 3, TP);
    ADCSequenceDataGet(ADC1_BASE, 3, TP1);
    UARTprintf("12V and 5V HV readings are %d----%d\n",TP[0],TP1[0]);
    TP [0] =  (TP[0]*3300)/4095;
    TP [0] =  (TP[0]*5636*120) ;
    TP [0] =  TP[0]/10000000;
    TP [0] =   TP[0];
    TP1 [0] =  (TP1[0]*3300)/4095;
    TP1 [0] =  (TP1[0]*7300)/22 ;
    TP1 [0] =  TP1[0]/10000;
    UARTprintf("12V and 5V HV readings are %d----%d\n",TP[0],TP1[0]);
    if(TP[0] > BF[24])
    	{
    		if(TP[0] > BF[25])
    			{
    				BF[25] = TP[0] ;
    			}
    		else
    			{
    				BF[26] = TP[0] ;
    			}
    	}

    else
    	{
    		BF[24] = TP[0];
    	}

    if(TP1[0] > BF[27])
        	{
        		if(TP1[0] > BF[28])
        			{
        				BF[28] = TP1[0] ;
        			}
        		else
        			{
        				BF[29] = TP1[0] ;
        			}
        	}

     else
       	{
       		BF[27] = TP1[0];
       	}
//////////////////////////////////////////////////////////////////////////



    TP[0] = (uint8_t)uc_core_temp();

    if(TP[0] > BF[30])
    	{
    		if(TP[0] > BF[31])
    			{
    				BF[31] = TP[0] ;
    			}
    		else
    			{
    				BF[32] = TP[0] ;
    			}
    	}

    else
    	{
    		BF[30] = TP[0];
    	}

    temp16 =(uint16_t) GET_HV_VOLTAGE();

    if(temp16 > HV_MIN)
    	{
    		if(temp16 > HV_MAX)
    			{
    				BF[35] = (uint8_t)temp16 >> 8;
    				BF[36] = (uint8_t)temp16 ;
    			}
    		else
    			{
					BF[37] = (uint8_t)temp16 >> 8;
					BF[38] = (uint8_t)temp16 ;
    			}
    	}

    else
    	{
			BF[33] = (uint8_t)temp16 >> 8;
			BF[34] = (uint8_t)temp16 ;
    	}

    BF[39] =  WTCT;
    BF[40] =  pyld_status;
    Configure_ADC_SCI();
    temp16   =  high_voltage;
    BF[41] = (uint8_t)temp16 >> 8;
    BF[42] = (uint8_t)temp16 ;
    temp16   =  low_voltage;
    BF[43] = (uint8_t)temp16 >> 8;
    BF[44] = (uint8_t)temp16 ;
    BF[45] = samples;
    temp16   = CTime;
    BF[46] = (uint8_t)temp16 >> 8;
    BF[47] = (uint8_t)temp16 ;
    BF[48] = HV_OVC_ST;
    temp16   = TSet;
    BF[49] = (uint8_t)temp16 >> 8;
    BF[50] = (uint8_t)temp16 ;
    BF[51] = science_data_mode;
}

void HK_Begin(void)
{
    SysCtlPeripheralEnable(SYSCTL_PERIPH_ADC0);
    SysCtlPeripheralEnable(SYSCTL_PERIPH_ADC1);
    SysCtlDelay(100);

    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOB);
    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOE);
	GPIOPinTypeADC(GPIO_PORTE_BASE,GPIO_PIN_0|GPIO_PIN_1|GPIO_PIN_4|GPIO_PIN_5|GPIO_PIN_6|GPIO_PIN_7);
	GPIOPinTypeADC(GPIO_PORTB_BASE,GPIO_PIN_4|GPIO_PIN_5);
	SysCtlDelay(100);
    ADCSequenceConfigure(ADC0_BASE, 3, ADC_TRIGGER_PROCESSOR, 0);
    ADCSequenceConfigure(ADC1_BASE, 3, ADC_TRIGGER_PROCESSOR, 0);

	ADCSequenceStepConfigure(ADC0_BASE, 3, 0, ADC_CTL_CH2 | ADC_CTL_IE |
	                                 ADC_CTL_END);
	ADCSequenceStepConfigure(ADC1_BASE, 3, 0, ADC_CTL_CH3 | ADC_CTL_IE |
		                                 ADC_CTL_END);
	ADCHardwareOversampleConfigure(ADC0_BASE,8);
	ADCHardwareOversampleConfigure(ADC1_BASE,8);
	ADCSequenceEnable(ADC0_BASE, 3);
	ADCSequenceEnable(ADC1_BASE, 3);
	ADCProcessorTrigger(ADC0_BASE, 3);
	ADCProcessorTrigger(ADC1_BASE, 3);

    while(!ADCIntStatus(ADC0_BASE, 3, false))
    {
    }
    ADCIntClear(ADC0_BASE, 3);
    while(!ADCIntStatus(ADC1_BASE, 3, false))
    {
    }
    ADCIntClear(ADC1_BASE, 3);
    ADCSequenceDataGet(ADC0_BASE, 3, TP);
    ADCSequenceDataGet(ADC1_BASE, 3, TP1);

    TP [0] = (uint8_t) TP[0];
    TP1 [0] = (uint8_t) TP1[0];
   	BF[1] = TP[0] ;
   	BF[2] = TP[0] ;
  	BF[0] = TP[0];
   	BF[4] = TP1[0] ;
   	BF[5] = TP1[0] ;
	BF[3] = TP1[0];
///////////////////////////////////////////////////////////////////////////////////
	ADCSequenceStepConfigure(ADC0_BASE, 3, 0, ADC_CTL_CH14 | ADC_CTL_IE |
	                                 ADC_CTL_END);
	ADCSequenceStepConfigure(ADC1_BASE, 3, 0, ADC_CTL_CH12 | ADC_CTL_IE |
		                                 ADC_CTL_END);
	ADCSequenceEnable(ADC0_BASE, 3);
	ADCSequenceEnable(ADC1_BASE, 3);
	ADCProcessorTrigger(ADC0_BASE, 3);
	ADCProcessorTrigger(ADC1_BASE, 3);
    while(!ADCIntStatus(ADC0_BASE, 3, false))
    {
    }
    ADCIntClear(ADC0_BASE, 3);
    while(!ADCIntStatus(ADC1_BASE, 3, false))
    {
    }
    ADCIntClear(ADC1_BASE, 3);
    ADCSequenceDataGet(ADC0_BASE, 3, TP);
    ADCSequenceDataGet(ADC1_BASE, 3, TP1);
    TP [0] = (uint8_t) TP[0];
    TP1 [0] = (uint8_t) TP1[0];
   	BF[7] = TP[0] ;
   	BF[8] = TP[0] ;
   	BF[6] = TP[0];
    BF[10] = TP1[0] ;
   	BF[11] = TP1[0] ;
  	BF[9] = TP1[0];

//////////////////////////////////////////////////////////////////////////////

	ADCSequenceStepConfigure(ADC0_BASE, 3, 0, ADC_CTL_CH15 | ADC_CTL_IE |
	                                 ADC_CTL_END);
	ADCSequenceStepConfigure(ADC1_BASE, 3, 0, ADC_CTL_CH8 | ADC_CTL_IE |
		                                 ADC_CTL_END);   //DAC
	ADCSequenceEnable(ADC0_BASE, 3);
	ADCSequenceEnable(ADC1_BASE, 3);
	ADCProcessorTrigger(ADC0_BASE, 3);
	ADCProcessorTrigger(ADC1_BASE, 3);
    while(!ADCIntStatus(ADC0_BASE, 3, false))
    {
    }
    ADCIntClear(ADC0_BASE, 3);
    while(!ADCIntStatus(ADC1_BASE, 3, false))
    {
    }
    ADCIntClear(ADC1_BASE, 3);
    ADCSequenceDataGet(ADC0_BASE, 3, TP);
    ADCSequenceDataGet(ADC1_BASE, 3, TP1);
    UARTprintf("TEMP %d--------DAC %d\n",TP[0],TP1[0]);
    TP [0]  = (TP[0]*3300)/4096;
    TP [0]  = 2414 - ((TP[0]*909)/1000);
    TP [0]  = (TP[0]/10);
    TP1 [0] = (TP1[0] * 3300)/4096;
    TP1 [0] = (TP1[0]/100);
    UARTprintf("TEMP %d--------DAC %d\n",TP[0],TP1[0]);
  	BF[13] = TP[0] ;
   	BF[14] = TP[0] ;
   	BF[12] = TP[0];
   	BF[16] = TP1[0] ;
	BF[17] = TP1[0] ;
    BF[15] = TP1[0];
	ADCSequenceStepConfigure(ADC0_BASE, 3, 0, ADC_CTL_CH10 | ADC_CTL_IE |
	                                 ADC_CTL_END);// 5V A121
	ADCSequenceStepConfigure(ADC1_BASE, 3, 0, ADC_CTL_CH11 | ADC_CTL_IE |
	                                 ADC_CTL_END);// -5 PH300
	ADCSequenceEnable(ADC0_BASE, 3);
	ADCSequenceEnable(ADC1_BASE, 3);

	ADCProcessorTrigger(ADC0_BASE, 3);
	ADCProcessorTrigger(ADC1_BASE, 3);
    while(!ADCIntStatus(ADC0_BASE, 3, false))
    {
    }
    ADCIntClear(ADC0_BASE, 3);
    while(!ADCIntStatus(ADC1_BASE, 3, false))
    {
    }
    ADCIntClear(ADC1_BASE, 3);
    ADCSequenceDataGet(ADC0_BASE, 3, TP);
    ADCSequenceDataGet(ADC1_BASE, 3, TP1);
    UARTprintf("5V and -5V readings are %d----%d\n",TP[0],TP1[0]);
    TP [0] =  (TP[0]*3300)/4095;
    TP [0] =  (TP[0]*7300*125)/22 ;
    TP [0] =  TP[0]/1000000;
    TP [0] =  TP[0];
    TP1 [0] =  (TP1[0]*3300)/4095;
    TP1 [0] =  (TP1[0]*5100)/22 ;
    TP1 [0] =  TP1[0]/10000;
    UARTprintf("5V and -5V readings are %d----%d\n",TP[0],TP1[0]);
   	BF[19] = TP[0] ;
   	BF[20] = TP[0] ;
   	BF[18] = TP[0];
   	BF[22] = TP1[0] ;
    BF[23] = TP1[0] ;
    BF[21] = TP1[0];
	ADCSequenceStepConfigure(ADC0_BASE, 3, 0, ADC_CTL_CH21 | ADC_CTL_IE |
	                                 ADC_CTL_END); //12V
	ADCSequenceStepConfigure(ADC1_BASE, 3, 0, ADC_CTL_CH20 | ADC_CTL_IE |
		                                 ADC_CTL_END);//  HV 5V
	ADCSequenceEnable(ADC0_BASE, 3);
	ADCSequenceEnable(ADC1_BASE, 3);
	ADCProcessorTrigger(ADC0_BASE, 3);
	ADCProcessorTrigger(ADC1_BASE, 3);
    while(!ADCIntStatus(ADC0_BASE, 3, false))
    {
    }
    ADCIntClear(ADC0_BASE, 3);
    while(!ADCIntStatus(ADC1_BASE, 3, false))
    {
    }
    ADCIntClear(ADC1_BASE, 3);
    ADCSequenceDataGet(ADC0_BASE, 3, TP);
    ADCSequenceDataGet(ADC1_BASE, 3, TP1);
    UARTprintf("12V and 5V HV readings are %d----%d\n",TP[0],TP1[0]);
    TP [0] =  (TP[0]*3300)/4095;
    TP [0] =  (TP[0]*5636*120) ;
    TP [0] =  TP[0]/10000000;
    TP [0] =   TP[0];
    TP1 [0] =  (TP1[0]*3300)/4095;
    TP1 [0] =  (TP1[0]*7300)/22 ;
    TP1 [0] =  TP1[0]/10000;
    UARTprintf("12V and 5V HV readings are %d----%d\n",TP[0],TP1[0]);
    BF[25] = TP[0] ;
    BF[26] = TP[0] ;
    BF[24] = TP[0];
    BF[28] = TP1[0] ;
    BF[29] = TP1[0] ;
    BF[27] = TP1[0];
    TP[0] = (uint8_t)uc_core_temp();
    BF[31] = TP[0] ;
    BF[32] = TP[0] ;
    BF[30] = TP[0];
    temp16 =(uint16_t) GET_HV_VOLTAGE();
   	BF[35] = (uint8_t)temp16 >> 8;
   	BF[36] = (uint8_t)temp16 ;
   	BF[37] = (uint8_t)temp16 >> 8;
	BF[38] = (uint8_t)temp16 ;
	BF[33] = (uint8_t)temp16 >> 8;
	BF[34] = (uint8_t)temp16 ;
    BF[39] =  WTCT;
    BF[40] =  mode_change;
    Configure_ADC_SCI();
    temp16   =  high_voltage;
    BF[41] = (uint8_t)temp16 >> 8;
    BF[42] = (uint8_t)temp16 ;
    temp16   =  low_voltage;
    BF[43] = (uint8_t)temp16 >> 8;
    BF[44] = (uint8_t)temp16 ;
    BF[45] = samples;
    temp16   = CTime;
    BF[46] = (uint8_t)temp16 >> 8;
    BF[47] = (uint8_t)temp16 ;
    BF[48] = HV_OVC_ST;
    temp16   = TSet;
    BF[49] = (uint8_t)temp16 >> 8;
    BF[50] = (uint8_t)temp16 ;
    BF[51] = science_data_mode;
}


void ADCConfigure_TEMP_uC(void)
{


    SysCtlPeripheralEnable(SYSCTL_PERIPH_ADC0);
    SysCtlDelay(10);
    ADCSequenceConfigure(ADC0_BASE, 3, ADC_TRIGGER_PROCESSOR, 0);

    ADCSequenceStepConfigure(ADC0_BASE, 3, 0, ADC_CTL_TS | ADC_CTL_IE |
                                 ADC_CTL_END);
    ADCSequenceEnable(ADC0_BASE, 3);
    ADCIntClear(ADC0_BASE, 3);

}



void Reset(void)
{

	HWREG(NVIC_APINT) = NVIC_APINT_VECTKEY | NVIC_APINT_SYSRESETREQ;
}

int uc_core_temp(void)
{

	uint32_t pui32ADC0Value[1];
    uint32_t ui32TempValueC;

    		ADCConfigure_TEMP_uC();
	        ADCProcessorTrigger(ADC0_BASE, 3);
	        while(!ADCIntStatus(ADC0_BASE, 3, false))
	        {
	        }
	        ADCIntClear(ADC0_BASE, 3);
	        ADCSequenceDataGet(ADC0_BASE, 3, pui32ADC0Value);
	        ui32TempValueC = (1475  - ((2250 * pui32ADC0Value[0])/4096)) / 10;
	        UARTprintf("Temperature of uC is %d \n",ui32TempValueC);
            return ui32TempValueC;
}

void I2C_DATA_PRINT(void)
{
	for(pia=0;pia<134;pia++)

		{
			UARTprintf("%d \n",g_ui32DataTx[pia] );
		}
}

void FLASH_BUFFER_WRITE(void)
{
	int siv = 0;
	for( pia = 6 ; pia < 133 ; pia = pia + 4 )
	{
		FLSH_WR[siv] = ( ((uint32_t)g_ui32DataRx[pia] << 24) | ((uint32_t)g_ui32DataRx[pia + 1] << 16) | ((uint32_t)g_ui32DataRx[pia + 2] << 8) | ((uint32_t)g_ui32DataRx[pia + 3]) ) ;
		siv++;
	}
}

void I2C_Write (int size)
{
	int Time_out = 0;
    GPIOPinWrite(GPIO_PORTA_BASE, GPIO_PIN_0, GPIO_PIN_0);
    SysCtlDelay(100);
    while ( !(GPIOPinRead(GPIO_PORTF_BASE,GPIO_PIN_5) == GPIO_PIN_5) )
    {
    	Time_out ++ ;
    	if (Time_out > 10000000)
    	{
    		UARTprintf("I2C Write Timeout \n");
    		break;
    	}
    }
    SysCtlDelay(10000);
    for(pia=0;pia<size;pia++)
    {
    	I2CSlaveDataPut(I2C1_BASE,g_ui32DataTx[pia]);
    	SysCtlDelay(1500);
    }
    GPIOPinWrite(GPIO_PORTA_BASE, GPIO_PIN_0, 0);
    UARTprintf("Telemetry Sent \n");
}


void I2C_Encode_Telemetry(int cmd)
{


	if(cmd == 0) // ACL L234 ACK
	{
		g_ui32DataTx[0]  = 0xB ;
		g_ui32DataTx[1]  = TCPSC;
		g_ui32DataTx[2]  = ACK;
		g_ui32DataTx[3]  = 0;
		g_ui32DataTx[4]  = 0;
		g_ui32DataTx[5]  = 0;
		g_ui32DataTx[6]  = 0;
		g_ui32DataTx[7]  = 0;
		g_ui32DataTx[8]  = 0;
		g_ui32DataTx[9]  = 0;
		g_ui32DataTx[10] = 0;
		CRC_Value        = CRC_16_Calc(g_ui32DataTx,11);
		g_ui32DataTx[11] = CRC_Value >> 8;
		g_ui32DataTx[12] = CRC_Value ;
		I2C_Write(134);
		Flag = 1;
	}

	else if(cmd == 1) // ACK L234 NACK_TC
	{
		g_ui32DataTx[0]  = 0x0B ;
		g_ui32DataTx[1]  = TCPSC;
		g_ui32DataTx[2]  = NACK_TC;
		g_ui32DataTx[3]  = 0;
		g_ui32DataTx[4]  = 0;
		g_ui32DataTx[5]  = 0;
		g_ui32DataTx[6]  = 0;
		g_ui32DataTx[7]  = 0;
		g_ui32DataTx[8]  = 0;
		g_ui32DataTx[9]  = 0;
		g_ui32DataTx[10] = 0;
		CRC_Value        = CRC_16_Calc(g_ui32DataTx,11);
		g_ui32DataTx[11] = CRC_Value >> 8;
		g_ui32DataTx[12] = CRC_Value ;
		I2C_Write(134);
		Flag = 1;
	}

	else if(cmd == 2) // ACK L234 NACK_CRC
	{
		g_ui32DataTx[0]  = 0xB0 ;
		g_ui32DataTx[1]  = 0;
		g_ui32DataTx[2]  = NACK_CRC;
		g_ui32DataTx[3]  = 0;
		g_ui32DataTx[4]  = 0;
		g_ui32DataTx[5]  = 0;
		g_ui32DataTx[6]  = 0;
		g_ui32DataTx[7]  = 0;
		g_ui32DataTx[8]  = 0;
		g_ui32DataTx[9]  = 0;
		g_ui32DataTx[10] = 0;
		CRC_Value        = CRC_16_Calc(g_ui32DataTx,11);
		g_ui32DataTx[11] = CRC_Value >> 8;
		g_ui32DataTx[12] = CRC_Value ;
		I2C_Write(134);
		Flag = 1;
	}

	else if(cmd == 3) // FMS ACK + Data
	{
		g_ui32DataTx[0]  = 0x78 ;
		g_ui32DataTx[1]  = TCPSC;
		g_ui32DataTx[2]  = ACK;
		g_ui32DataTx[3]  = 0;
		for (pia = 0 ; pia < 128 ; pia++ )
		{
			g_ui32DataTx[pia + 4] = Data [pia];
		}
		CRC_Value        = CRC_16_Calc(g_ui32DataTx,132);
		UARTprintf("CRC_Value %d \n ", CRC_Value );
		g_ui32DataTx[132] = (CRC_Value >> 8) ; //132
		g_ui32DataTx[133] = (CRC_Value) ;        //133
		I2C_DATA_PRINT();
		I2C_Write(134);

		Flag = 1;
	}

	else if(cmd == 4) // FMS NACK_TC
	{
		g_ui32DataTx[0]  = 0x78 ;
		g_ui32DataTx[1]  = TCPSC;
		g_ui32DataTx[2]  = NACK_TC;
		g_ui32DataTx[3]  = 0;
		for (pia = 0 ; pia < 128 ; pia++ )
		{
			g_ui32DataTx[pia + 4] = 0;
		}
		CRC_Value        = CRC_16_Calc(g_ui32DataTx,132);
		g_ui32DataTx[132] = (CRC_Value >> 8) ;
		g_ui32DataTx[133] = (CRC_Value) ;
		I2C_Write(134);

		Flag = 1;
	}


	else if(cmd == 5) // LMB ACK + DATA
	{
		g_ui32DataTx[0]  = 0x30;
		g_ui32DataTx[1]  = TCPSC;
		g_ui32DataTx[2]  = ACK;
		g_ui32DataTx[3]  = 0;
		for(pia = 0 ; pia < 128 ; pia++ )
		{
			g_ui32DataTx[4 + pia] = Data[pia];
		}
		CRC_Value        = CRC_16_Calc(g_ui32DataTx,132);
		UARTprintf("CRC_Value %d \n ", CRC_Value );
		g_ui32DataTx[132] = CRC_Value >> 8;
		g_ui32DataTx[133] = CRC_Value ;
		I2C_DATA_PRINT();
		I2C_Write(134);
	}

	else if(cmd == 6) // LMB ACK + NACK_TC
	{
		g_ui32DataTx[0]  = 0x30;
		g_ui32DataTx[1]  = TCPSC;
		g_ui32DataTx[2]  = NACK_TC;
		g_ui32DataTx[3]  = 0;
		for(pia = 0 ; pia < 128 ; pia++ )
		{
			g_ui32DataTx[4 + pia] = 0;
		}
		CRC_Value        = CRC_16_Calc(g_ui32DataTx,132);
		g_ui32DataTx[132] = CRC_Value >> 8;
		g_ui32DataTx[133] = CRC_Value ;
		I2C_Write(134);
	}

	else if(cmd == 7) // LMB ACK + NACK_CRC
	{
		g_ui32DataTx[0]  = 0x30;
		g_ui32DataTx[1]  = TCPSC;
		g_ui32DataTx[2]  = NACK_CRC;
		g_ui32DataTx[3]  = 0;
		for(pia = 0 ; pia < 128 ; pia++ )
		{
			g_ui32DataTx[4 + pia] = 0;
		}
		CRC_Value        = CRC_16_Calc(g_ui32DataTx,132);
		g_ui32DataTx[132] = CRC_Value >> 8;
		g_ui32DataTx[133] = CRC_Value ;
		I2C_Write(134);
	}


}

void I2C_Decode_TC(void) // done like this coz of the change
{

	UARTprintf("Decoding Telecommand \n");
	if(TC_SIZE == 135)
	{
		TCPSC     = g_ui32DataRx[0];
		Service   = g_ui32DataRx[2];
		ST_ADDR_32 = ST_ADDR_32 | g_ui32DataRx[3] ;
		ST_ADDR_32 = ST_ADDR_32 << 8;
		ST_ADDR_32 = ST_ADDR_32 | g_ui32DataRx[4] ;
		ST_ADDR_32 = ST_ADDR_32 * 4 ;
		for(pia = 0 ; pia < 128 ; pia++ )
		{
			g_ui32DataTx[4 + pia] = 0;
		}
		if ( Service == 0x66 )
		{
			FLASH_BUFFER_WRITE();
			FlashProgram(FLSH_WR , ST_ADDR_32 , 128);
			I2C_Encode_Telemetry(0);
		}

		else
		{
			I2C_Encode_Telemetry(1);
		}

	}

	else if(TC_SIZE == 11)
	{
		TCPSC     = g_ui32DataRx[0];
		Service   = g_ui32DataRx[2];
		if (Service == 0x81) // FMS
		{
			UARTprintf("FMS Telecommand Recieved \n");
			Command   = g_ui32DataRx[3];
			if (Command == 0x41)
			{
				UARTprintf("HK Min Max reset \n");
				for(pia = 0; pia < 40; pia ++ )
				{
					BF[pia] = 0;
				}
				I2C_Encode_Telemetry(0);
			}

			else if (Command == 0xD0)
			{
				UARTprintf("Reading uC temperature \n ");
				Data[0] = uc_core_temp();
				I2C_Encode_Telemetry(3);
			}

			else if (Command == 0xD1)
			{
				UARTprintf("Sending SRP through I2C \n");
				I2C_Encode_Telemetry(0);
			}

			else if (Command == 0xD2)
			{
				for (pia = 0; pia < 128 ; pia++)
				{
					Data[pia] = 0;
				}
				LEDAcquire(1,1000,1);
				I2C_Encode_Telemetry(3);
			}
			else if (Command == 0x31)
			{
				for (pia = 0; pia < 128 ; pia++)
				{
					Data[pia] = 0;
				}
				I2C_Encode_Telemetry(3);
				UARTprintf("Reset TC recieved \n");
				Reset();
			}

			else if (Command == 0x01)
			{
				UARTprintf("SPEED Intializing \n");
				PYLD_SPEED_INIT();
				I2C_Encode_Telemetry(0);
			}

			else if (Command == 0x02)
			{
				PL_PREV_STATE_CHECK(1);
				mode_change=1;
				//PYLD_SPEED_STANDBY_INIT();
				I2C_Encode_Telemetry(0);
			}

			else if (Command == 0x03)
			{
				PL_PREV_STATE_CHECK(2);
				mode_change=2;
				//PYLD_SPEED_HIBERNATE_INIT();
				I2C_Encode_Telemetry(0);
			}

			else if (Command == 0x04)
			{
				mode_change=3;
				UARTprintf("Science Mode on \n");
				//PYLD_SPEED_SCIENCE_INIT();
				I2C_Encode_Telemetry(0);
			}

			else if (Command == 0x12) // LED Switch on
			{
				LED_SW = 1;
				GPIOPinWrite(GPIO_PORTH_BASE, GPIO_PIN_5, 0);
				UARTprintf("LED Switch is now On \n ");
				I2C_Encode_Telemetry(0);
			}

			else if (Command == 0X22) // LED Switch off
			{
				LED_SW = 0;
				GPIOPinWrite(GPIO_PORTH_BASE, GPIO_PIN_5, GPIO_PIN_5);
				UARTprintf("LED Switch is now OFF \n ");
				I2C_Encode_Telemetry(0);
			}

			else if (Command == 0x13) // HV Switch on
			{
				HV_SW = 1;
				GPIOPinWrite(GPIO_PORTN_BASE, GPIO_PIN_1, 0);
				UARTprintf("HV Switch is now On \n ");
				I2C_Encode_Telemetry(0);
			}
			else if (Command == 0x23) // LED Switch on
			{
				HV_SW = 0;
				GPIOPinWrite(GPIO_PORTN_BASE, GPIO_PIN_1, GPIO_PIN_1);
				UARTprintf("HV Switch is now OFF \n ");
				I2C_Encode_Telemetry(0);
			}

			else
			{
				I2C_Encode_Telemetry(1); /// NACK Bad command
				UARTprintf("Bad Short TC \n");
			}

		}

		else if (Service == 0x61 ) // MMS read FLASH
		{
			UARTprintf("FLASH Read Command Recieved \n");
			ST_ADDR_32 = ST_ADDR_32 | g_ui32DataRx[3] ;
			ST_ADDR_32 = ST_ADDR_32 << 8;
			ST_ADDR_32 = ST_ADDR_32 | g_ui32DataRx[4] ;
			ST_ADDR_32 = ST_ADDR_32 * 4 ;
			UARTprintf("Start Address %d \n",ST_ADDR_32);
			for ( pia = 0 ; pia < 32 ; pia++ )
			{
				siv = pia*4;
				Buffer       = HWREG(ST_ADDR_32 + (4*pia));
				Data[siv]    = Buffer >> 24;
				Data[siv+1]  = Buffer >> 16;
				Data[siv+2]  = Buffer >> 8;
				Data[siv+3]  = Buffer ;

			}
			I2C_Encode_Telemetry(5);
		}
		else if (Service == 0x62)// MMS read RAM HK
		{
			if (g_ui32DataRx[4] == 0x00)
			{
				UARTprintf("Sending HK Min Max \n");
			    for (pia = 0; pia < 52 ; pia++)
			    {
			    	Data[pia] = BF[pia];
			    }
				I2C_Encode_Telemetry(5);
			}
		}

		else if (Service == 0x65)
		{
			Command = g_ui32DataRx[4];
			if(Command == 0x01)
			{
				temp32 = 0;
				temp16 = 0;
				temp16 = temp16 | (uint16_t) g_ui32DataRx[5];
				temp16 = temp16 << 8 ;
				temp16 = temp16 | (uint16_t) g_ui32DataRx[6];
				temp32 = temp16;
				high_voltage = temp16;
				UARTprintf("High Voltage Will be Set to %d V \n",temp16);
				FL_TEMP[0]   = temp32;
				FlashErase(HV_NOR_AD);
				UARTprintf("Flash write %d \n",FlashProgram(FL_TEMP, HV_NOR_AD, 4));
				temp32 = 0;
				temp16 = 0;
				temp16 = temp16 | g_ui32DataRx[7];
				temp16 = temp16 << 8 ;
				temp16 = temp16 | g_ui32DataRx[8];
				temp32 = temp16;
				low_voltage = temp16;
				UARTprintf("Hibernate Volatage Updated to %d \n",temp16);
				FL_TEMP[0]   = temp32;
				FlashErase(HV_HIB_AD);
				FlashProgram(FL_TEMP, HV_HIB_AD, 4);
				temp32 = 0;
				temp16 = 0;
				I2C_Encode_Telemetry(0);
				//HV_Value_Set(high_voltage);
			}

			else if (Command == 0x02)
			{
				temp32 = 0;
				temp16 = 0;
				temp32 = g_ui32DataRx[5];
				samples = g_ui32DataRx[5];
				FL_TEMP[0]   = temp32;
				FlashErase(Samples_AD);
				FlashProgram(FL_TEMP, Samples_AD, 4);
				temp32 = 0;
				temp16 = 0;
				temp16 = temp16 | (uint16_t) g_ui32DataRx[6];
				temp16 = temp16 << 8 ;
				temp16 = temp16 | (uint16_t) g_ui32DataRx[7];
				CTime  = temp16;
				temp32 = temp16;
				FL_TEMP[0]   = temp32;
				FlashErase(CT_Time_AD);
				FlashProgram(FL_TEMP, CT_Time_AD, 4);
				I2C_Encode_Telemetry(0);
			}

			else if (Command == 0x03)
			{
				temp32 = 0;
				temp32 = g_ui32DataRx[8];
				FL_TEMP[0]   = temp32;
				FlashErase(SCI_MD_AD);
				FlashProgram(FL_TEMP, SCI_MD_AD , 4);
				science_data_mode = g_ui32DataRx[8];
				I2C_Encode_Telemetry(0);
			}

			else if (Command == 0x04)
			{
				temp32 = 0;
				temp16 = 0;
				temp16 = temp16 | (uint16_t) g_ui32DataRx[5];
				temp16 = temp16 << 8 ;
				temp16 = temp16 | (uint16_t) g_ui32DataRx[6];
				temp32 = temp16;
				TSet = temp16;
				FL_TEMP[0]   = temp32;
				FlashErase(TSet_AD);
				FlashProgram(FL_TEMP, TSet_AD, 4);
				I2C_Encode_Telemetry(0);
				Configure_DPOT();
			}

			else if (Command == 0x05)
			{
				temp32 = 0;
				temp16 = 0;
				temp16 = temp16 | (uint16_t) g_ui32DataRx[5];
				temp16 = temp16 << 8 ;
				temp16 = temp16 | (uint16_t) g_ui32DataRx[6];
				temp32 = temp16;
				LED_ISET = temp16;
				FL_TEMP[0]   = temp32;
				FlashErase(LED_ISet_AD);
				FlashProgram(FL_TEMP, LED_ISet_AD, 4);
				I2C_Encode_Telemetry(0);
				Configure_DPOT_LED();
			}

			else if (Command == 0x06)
			{
				temp16 = 0;
				temp32 = 0;
				temp16 = temp16 | (uint16_t) g_ui32DataRx[5];
				temp16 = temp16 << 8 ;
				temp16 = temp16 | (uint16_t) g_ui32DataRx[6];
				temp32 = temp16;
				SP_MD_ST = temp16;
				FlashErase(SP_MD_ST_AD);
				FlashProgram(FL_TEMP, SP_MD_ST_AD, 4);
				temp16 = 0;
				temp32 = 0;
				temp16 = temp16 | (uint16_t) g_ui32DataRx[7];
				temp16 = temp16 << 8 ;
				temp16 = temp16 | (uint16_t) g_ui32DataRx[8];
				temp32 = temp16;
				SP_MD_ED = temp16;
				FlashErase(SP_MD_ED_AD);
				FlashProgram(FL_TEMP, SP_MD_ED_AD, 4);
				I2C_Encode_Telemetry(0);
			}

			else if (Command == 0x07)
			{
				temp32 = ( ((uint32_t)g_ui32DataRx[5] << 24) | ((uint32_t)g_ui32DataRx[6] << 16) | ((uint32_t)g_ui32DataRx[7] << 8) | ((uint32_t)g_ui32DataRx[8]) ) ;
				UARTprintf("Erasing block %05x \n",temp32);
				if(FlashErase(temp32) == 0 )
				{
					I2C_Encode_Telemetry(0);
				}

				else
				{
					I2C_Encode_Telemetry(1);
				}

			}

		}

		else if (Service == 0X69)// MMS CRC flash
		{

		}
	}
}




void Buffer_Clear(void)
{
	for(pia=0 ; pia<140; pia++)
	{

		g_ui32DataRx[pia] = 0;
		g_ui32DataTx[pia] = 0;
		Data [pia]   = 0;

	}
}

uint16_t CRC_16_Calc1(const uint8_t *data, uint16_t size)
{
    uint16_t CRC_OUT = 0;
    int bits_read = 0, bit_flag;



    while(size > 0)
    {
        bit_flag = CRC_OUT >> 15;

        /* Get next bit: */
        CRC_OUT <<= 1;
        CRC_OUT |= (*data >> (7 - bits_read)) & 1;

        /* Increment bit counter: */
        bits_read++;
        if(bits_read > 7)
        {
            bits_read = 0;
            data++;
            size--;
        }

        /* Cycle check: */
        if(bit_flag)
        	CRC_OUT ^= CRC16;

    }
    return CRC_OUT;
}

     uint16_t   CRC_16_Calc(const unsigned char message[], unsigned int nBytes){
    	 uint16_t remainder = 0xffff;
        int byte;
        char bit;

        for( byte = 0 ; byte < nBytes ; byte++ ){
            /*
            Bring the data byte by byte
            each time only one byte is brought
            0 xor x = x
            */
            remainder = remainder ^ ( message[byte] << 8 );

            for( bit = 8 ; bit > 0 ; bit--){
                /*
                for each bit, xor the remainder with polynomial
                if the MSB is 1
                */
                if(remainder & TOPBIT16){
                    remainder = (remainder << 1) ^ POLYNOMIAL16;
                    /*
                    each time the remainder is xor-ed with polynomial, the MSB is made zero
                    hence the first digit of the remainder is ignored in the loop
                    */
                }
                else{
                    remainder = (remainder << 1);
                }
            }
        }

        return remainder;
    }


#endif /* I2C_H_ */
