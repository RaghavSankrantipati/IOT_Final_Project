//SAI RAGHAVENDRA SANKRANTIPATI
//ASSIGNMENT IV, October 6


/* @section License
 * <b>(C) Copyright 2014 Silicon Labs, http://www.silabs.com</b>
 *******************************************************************************
 *
 * This file is licensed under the Silicon Labs Software License Agreement. See
 * "http://developer.silabs.com/legal/version/v11/Silicon_Labs_Software_License_Agreement.txt"
 * for details. Before using this software for any purpose, you must agree to the
 * terms of that agreement.
 *
 ***************************************************************************** */



/*  This code is originally Silicon Labs and copy righted by Silicon Labs' in 2015 and Silicon Labs'
 * grants permission to anyone to use the software for any purpose, including commercial
 * applications, and to alter it, and redistribution it freely subject that the origins is not miss represented,
 * altered or removed from any source distribution.
 * Developer changed the Names of the routines to the following Routines include:
*  void blockLeepMOde(uint32_t em);
*  void Sleep(void);
*  converToCelsius();
*  adcbasic.c;
*  i2c_master_slave.c
*  i2c_tempsens_main.c
*  and also Prof. Graham lectures were referred while making this code.
*  Documents like Application notes of EFM32 were also referred.
*
*  */

#include "em_chip.h"
#include "em_int.h"
#include "em_cmu.h"
#include "em_emu.h"
#include "em_letimer.h"
#include "em_timer.h"
#include "em_acmp.h"
#include "em_gpio.h"
#include "em_adc.h"
#include "em_leuart.h"
#include "dmactrl.c"
#include "dmactrl.h"
#include "i2cspm.h"
#include "em_dma.h"
#include "em_i2c.h"
#include "i2c.h"
#include "circbuff.h"
#include "leuart.h"
#include "segmentlcd.h"
#include "caplesense.h"
#include "mma8452q.h"
#include "lesense_letouch.h"
#include <stdint.h>
#include <stdbool.h>
#include <string.h>
#include <stddef.h>
#include <stdio.h>
#include <em_lesense.h>



#define EM0 0                                                                                                                /* Energy modes in EFM32*/
#define EM1 1
#define EM2 2
#define EM3 3
#define EM4 4

unsigned int sleep_block_counter[EM4+1];                               /*This is an Array that stores the number of peripherals in each energy state */


#define LE0_period              3                                                                                       /*time period in seconds*/
#define LE0_LEDon               0.004                                                                               /*LED should blink for every 50ms*/
#define LFXOcount               32768                                                                  /*Number of cycles LFXO can make in one second*/
#define ULFRCOcount             1000                                                              /*Number of cycles ULFRCO can make in one second*/
#define EMmode                  EM2                                                                       /*Energy mode in which LETIMER runs*/
float OSCratio =1;
int a, b;                                                                                       //variables declared for reading ACMP status

#define TSL2561                 0
uint8_t period = 1;

#define DMAstatus               1                                                               //if DMA is not used initialise 0 and 1 when DMA is used
#define templow                 15
#define temphigh                35
#define TempSenseRate           100000
#define ADC0resolution          adcRes12Bit
#define ADC0Prescaler           9
#define DMAchannel              0
#define ADCReadings             500

#define acmphigh                3
#define acmplow                 1

#define LIGHTSENSE_EXCITE_PORT   gpioPortD
#define LIGHTSENSE_EXCITE_PIN    6U
#define LIGHTSENSE_SENSOR_PORT   gpioPortC
#define LIGHTSENSE_SENSOR_PIN    6U
/*define peripherals */

#define BME 			1
#define MMA				1
#define TOUCH			1
#define circular_buffer	1
#define ULFRCO_Config	0
#define ADC 			1
#define Light_Sense		1

void CMUConfiguration (void);
void blockSleepMode(unsigned int minimumMode);
void Sleep(void);
void LETIMER0Setup(void);
void LED0on(void);
void LED0off(void);
void LED1on(void);
void LED1off(void);
void ULFRCOSetup(void);
void unblockSleepMode(int minimumMode);
void DMASetup(void);
void ADCSetup(void);
void CALLBackfn(unsigned int channel, bool primary, void *user);
void WhenDMAoff(void);
float converToCelsius(int32_t adcSample);
void CircBuff(void);
volatile uint16_t ADCSamples[ADCReadings];
void LS_on(void);
void LS_off(void);
void LCD_off(void);
void print_temp(void);
float temp;
uint16_t channels_touched = 0;
bool pattern=false;
DMA_CB_TypeDef CALLBack;
uint8_t disp = 0;



void CMUConfiguration(void)
{
	CMU_OscillatorEnable(cmuOsc_HFXO,false, false);
	CMU_ClockEnable(cmuClock_CORELE, true);
	CMU_ClockEnable(cmuClock_HFPER, true);                                                                                      /* Enable High Frequency Low energy Clock*/

	if (EMmode == EM3)
	{
		CMU_OscillatorEnable(cmuOsc_LFXO, false, false);		//Make sure Low Frequency Crystal Oscillator is turned OFF
		CMU_OscillatorEnable(cmuOsc_LFRCO, false, false);		//Make sure Low Frequency RC Oscillator is turned OFF
		CMU_OscillatorEnable(cmuOsc_ULFRCO, true, true);		//Turn on Ultra Low Frequency RC Oscillator
		CMU_ClockSelectSet(cmuClock_LFA, cmuSelect_ULFRCO);		//Linking Ultra Low Frequency RC Oscillator with Low Frequency A Clock if Energy Mode is EM3
	}

	else
	{

		CMU_OscillatorEnable(cmuOsc_LFRCO, false, false);		//Make sure Low Frequency RC Oscillator is turned OFF
		CMU_OscillatorEnable(cmuOsc_ULFRCO, false, false);		//Make sure Ultra Low Frequency RC Oscillator is turned OFF
		CMU_OscillatorEnable(cmuOsc_LFXO, true, true);			//Turn on Low Frequency Crystal Oscillator

		CMU_ClockSelectSet(cmuClock_LFA, cmuSelect_LFXO);		//Linking Low Frequency Crystal Oscillator with Low Frequency A Clock if Energy Mode is not EM3
	}
}


void blockSleepMode(unsigned int minimumMode)
{
	INT_Disable();
	sleep_block_counter[minimumMode]++;                                   /* the value of an element in the array increases by 1 when a peripheral calls this function*/
	INT_Enable();
}

void unblockSleepMode(int minimumMode)
{
    INT_Disable();
    sleep_block_counter[minimumMode]--;
    INT_Enable();
}


void Sleep(void)
{
	if (sleep_block_counter[EM0] > 0)
	{
	}

	else if (sleep_block_counter[EM1] > 0)
	{
		EMU_EnterEM1();                                                                                                                   /* Enter in Sleep mode 1*/
	}

	else if (sleep_block_counter[EM2] > 0)
	{
		EMU_EnterEM2(true);                                                                                                                    /* Enter in sleep mode 2*/
	}

	else if (sleep_block_counter[EM3] > 0)
	{
		EMU_EnterEM3(true);                                                                                                                     /*Enter in sleep mode 3*/
	}
}

void ULFRCOSetup(void)
{

	int a,b,c,p,q,r;
	blockSleepMode(EM0);

	CMU_ClockEnable(cmuClock_LETIMER0, true);                         //EM0 mode is set and LFXO HFLE were enabled
	LETIMER0->CTRL=0x281;
	LETIMER0->CNT=LFXOcount;

	CMU_ClockEnable(cmuClock_TIMER0, true);
	CMU_ClockEnable(cmuClock_TIMER1, true);                           //clocks are enabled to LETIMER0, TIMER0, TIMER1

	TIMER_Init_TypeDef Timer0_init = TIMER_INIT_DEFAULT;
	TIMER_Init(TIMER0, &Timer0_init);
	TIMER_Init_TypeDef Timer1_init = TIMER_INIT_DEFAULT;              //TIMER1 is cascaded to TIMER0 and sync is enabled
	Timer1_init.clkSel = timerClkSelCascade;
	Timer1_init.sync =true;
	TIMER_Init(TIMER1, &Timer1_init);

	TIMER_Enable(TIMER0, true);
	TIMER_Enable(TIMER1, true);
	LETIMER_Enable(LETIMER0, true);
	LETIMER0->CMD=LETIMER_CMD_START;
	TIMER0->CMD=TIMER_CMD_START;
	TIMER1->CMD=TIMER_CMD_START;


	while(LETIMER0->CNT!=0);                                          // LETIMER0 is counted down from LFXO and after completion TIMERs and LETIMER was disabled

	TIMER_Enable(TIMER0, false);
	TIMER_Enable(TIMER1, false);
	LETIMER_Enable(LETIMER0, false);

	a=TIMER0->CNT;
    b=TIMER1->CNT;
    c=b*65536 +a;                                                      // total count of High frequency clock 14MHz is calculated
    TIMER0->CNT=0;
    TIMER1->CNT=0;

    CMU_ClockEnable(cmuClock_LETIMER0, false);
    CMU_OscillatorEnable(cmuOsc_LFXO, false, false);
    CMU_OscillatorEnable(cmuOsc_ULFRCO, true, true);
    CMU_ClockSelectSet(cmuClock_LFA, cmuSelect_ULFRCO);
    CMU_ClockEnable(cmuClock_LETIMER0, true);

    LETIMER0->CTRL=0x81;
    LETIMER0->CNT=ULFRCOcount;                                         // LETIMER0 is again setup and ULFRCO is given as clock and 1000 is given to counter
    TIMER_Enable(TIMER0, true);
    TIMER_Enable(TIMER1, true);
    LETIMER_Enable(LETIMER0, true);

    while(LETIMER0->CNT!=0);

    TIMER0->CMD=0x00000002;
    TIMER1->CMD=0x00000002;
    LETIMER0->CMD=0X00000002;
    p=TIMER0->CNT;
    q=TIMER1->CNT;
    r=q*65536 +p;                                                 //total count of 14MHz clock is calculated in 1sec

    OSCratio = ((float)c/(float)r);                               // ratio of these two counts are calculated
    CMU_ClockEnable(cmuClock_TIMER0, false);
    CMU_ClockEnable(cmuClock_TIMER1, false);
    CMU_ClockEnable(cmuClock_LETIMER0,false);
    CMU_OscillatorEnable(cmuOsc_LFXO, false, false);
    unblockSleepMode(EM0);
}




void LETIMER0Setup(void)
{
	CMU_ClockEnable(cmuClock_LETIMER0, true);
	int interruptflag;
	int Comp0value;
	int Comp1value;

	LETIMER_Init_TypeDef  LETIMER0_InitStructure = LETIMER_INIT_DEFAULT;				//LETIMER0 structure for setting up LETIMER

	LETIMER0_InitStructure.enable			=	false;				//False sets the LETIMER not to turn on as soon as it sets up
	LETIMER0_InitStructure.comp0Top 		=	true;				//Load COMP0 register into CNT when counter underflows to yes
	LETIMER0_InitStructure.repMode			=	letimerRepeatFree;	//Select LETIMER mode as Free Repeat Mode

	LETIMER_Init(LETIMER0,&LETIMER0_InitStructure);				//Initiate LETIMER setup

	if (EMmode == EM3)
	{
		Comp0value = LE0_period *OSCratio* ULFRCOcount;                                                              /* Comp0value= 2 secs * 1000 = 2000. 0x7D is the Comp0 value.*/
	}

	else if(EMmode ==EM2)
	{
		int i=0;
		int DesirePeriod=LE0_period*LFXOcount;
		while ((DesirePeriod/ (1<<i))>65535)
            {
                i++;
            }
		CMU->LFAPRESC0 |= (i<<8);
		int prescaler=1<<i;
		Comp0value = LE0_period * (LFXOcount-1)/prescaler;                                                   /* Comp0value= 2 secs * (32768-1)= 65534. 0xFFFE is the Comp0 value.*/
	}

	else if(EMmode ==EM1)
	{
		int i=0;
		int DesirePeriod=LE0_period*LFXOcount;
		while ((DesirePeriod/ (1<<i))>65535)
            {
                i++;
            }
		CMU->LFAPRESC0 |= (i<<8);
		int prescaler=1<<i;
		Comp0value = LE0_period * (LFXOcount)/prescaler;                                                    /* Comp0value= 2 secs * (32768-1)= 65534. 0xFFFE is the Comp0 value.*/
	}
else if(EMmode ==EM0)
{
    int i=0;
    int DesirePeriod=LE0_period*LFXOcount;
    while ((DesirePeriod/ (1<<i))>65535)
            {
                i++;
            }
    CMU->LFAPRESC0 |= (i<<8);
    int prescaler=1<<i;
    Comp0value = LE0_period * (LFXOcount)/prescaler;                                                    /* Comp0value= 2 secs * (32768-1)= 65534. 0xFFFE is the Comp0 value.*/
}
LETIMER0->CNT = Comp0value;
LETIMER_CompareSet(LETIMER0,0,Comp0value);


	if(EMmode == EM3)
	{
		Comp1value = LE0_LEDon * OSCratio *ULFRCOcount;                                                             /* Comp0value= 0.05 secs * 1000 = 50. 0x32 is the Comp1 value.*/
	}
	else if(EMmode == EM2)
	{
		int i=0;
		int DesirePeriod=LE0_period*LFXOcount;
		while ((DesirePeriod/ (1<<i))>65535)
				{
					i++;
				}
		CMU->LFAPRESC0 |= (i<<8);
		int prescaler=1<<i;
		Comp1value = LE0_LEDon * LFXOcount/prescaler ;                                                      /* Comp0value= 0.05 secs * (32768)= 65534. 0x666 is the Comp1 value.*/
	}
	else if(EMmode == EM1)
	{
		int i=0;
		int DesirePeriod=LE0_period*LFXOcount;
		while ((DesirePeriod/ (1<<i))>65535)
				{
					i++;
				}
		CMU->LFAPRESC0 |= (i<<8);
		int prescaler=1<<i;
		Comp1value = LE0_LEDon * LFXOcount/prescaler ;                                                       /* Comp0value= 0.05 secs * (32768)= 65534. 0x666 is the Comp1 value.*/
	}

	else if(EMmode == EM0)
	{
		int i=0;
		int DesirePeriod=LE0_period*LFXOcount;
		while ((DesirePeriod/ (1<<i))>65535)
				{
					i++;
				}
		CMU->LFAPRESC0 |= (i<<8);
		int prescaler=1<<i;
		Comp1value = LE0_LEDon * LFXOcount/prescaler ;                                                       /* Comp0value= 0.05 secs * (32768)= 65534. 0x666 is the Comp1 value.*/
	}


	LETIMER_CompareSet(LETIMER0,1,Comp1value);

	interruptflag = LETIMER0->IF;                                                           /* value at interrupt flag register was stored in interrupt flag variable*/
	LETIMER0->IFC = interruptflag;                                                                                          /*All the interrupts were cleared*/
	LETIMER0->IEN = LETIMER_IEN_UF  | LETIMER_IEN_COMP1;                                     /* Interrupt enable register is loaded if underflow or comp1 flag is loaded*/

	blockSleepMode(EMmode);

	NVIC_EnableIRQ(LETIMER0_IRQn);                                                                                               /*nested  vector interrupt controller*/
	LETIMER_Enable(LETIMER0, true);
	}


void LED0on(void)
{
	CMU_ClockEnable(cmuClock_GPIO, true);
    GPIO_PinOutSet(led0port,led0pin);                                                       /* output at portE 2U pin is turned on when LED0on Function is called*/
    CMU_ClockEnable(cmuClock_GPIO, false);
}

void LED0off(void)
{
	CMU_ClockEnable(cmuClock_GPIO, true);
    GPIO_PinOutClear(led0port,led0pin);                                                      /* output at portE 2U pin is turned low when LED0off Function is called*/
    CMU_ClockEnable(cmuClock_GPIO, false);
}
void LED1on(void)
{
	CMU_ClockEnable(cmuClock_GPIO, true);
    GPIO_PinOutSet(led1port,led1pin);                                                       /* output at portE 3U pin is turned on when LED1on Function is called*/
    CMU_ClockEnable(cmuClock_GPIO, false);
}

void LED1off(void)
{
	CMU_ClockEnable(cmuClock_GPIO, true);
    GPIO_PinOutClear(led1port,led1pin);                                                      /* output at portE 3U pin is turned low when LED1off Function is called*/
    CMU_ClockEnable(cmuClock_GPIO, false);
}


void DMASetup(void)
{
	CMU_ClockEnable(cmuClock_DMA, true);
    blockSleepMode(EM3);
    DMA_Init_TypeDef DMAInit;
    DMAInit.hprot        = 0;
    DMAInit.controlBlock = dmaControlBlock;
    DMA_Init(&DMAInit);                                                     //DMA is initialised

    CALLBack.cbFunc  = CALLBackfn;                                          //call back function name is specified
    CALLBack.userPtr = NULL;

    DMA_CfgChannel_TypeDef DMAChannelCfg;
    DMAChannelCfg.highPri   = true;
    DMAChannelCfg.enableInt = true;
    DMAChannelCfg.select    = DMAREQ_ADC0_SINGLE;                           //ADC is given to channel 0
    DMAChannelCfg.cb        = &CALLBack;
    DMA_CfgChannel(0, &DMAChannelCfg);                                      //DMA channel is setup

    DMA_CfgDescr_TypeDef   DMADscrpfg;
    DMADscrpfg.dstInc  = dmaDataInc2;
    DMADscrpfg.srcInc  = dmaDataIncNone;
    DMADscrpfg.size    = dmaDataSize2;
    DMADscrpfg.arbRate = dmaArbitrate1;
    DMADscrpfg.hprot   = 0;
    DMA_CfgDescr(0, true, &DMADscrpfg);                                     //DMA descriptors are setup

    DMA_ActivateBasic(DMAchannel,
                      true,
                      false,
                      (void *)ADCSamples,
                      (void *)&(ADC0->SINGLEDATA),
                      ADCReadings - 1);
    unblockSleepMode(EM3);
}

void ADCSetup(void)
{
	CMU_ClockEnable(cmuClock_ADC0, true);

      blockSleepMode(EM1);
      ADC_Init_TypeDef ADCInit = ADC_INIT_DEFAULT;                              //ADC is initialised in defaultmode and prescaler and time base is changed to 9 and 13
      ADCInit.prescale = 12;
      ADCInit.timebase=13;
      ADC_Init(ADC0, &ADCInit);

      ADC_InitSingle_TypeDef  ADCSingleCfg = ADC_INITSINGLE_DEFAULT;            //ADC Single Mode is initilased with aquisition time 128
      ADCSingleCfg.input = adcSingleInputTemp;
      ADCSingleCfg.acqTime=adcAcqTime2;
      ADCSingleCfg.rep=true;
      ADC_InitSingle(ADC0, &ADCSingleCfg);

      ADC_Start(ADC0, adcStartSingle);                                          //start ADC
}


void CALLBackfn(unsigned int channel, bool primary, void *user)
{
  (void) channel;
  (void) primary;
  (void) user;

  uint32_t sum=0;                                                               // Total of 1000 is recorded and average is taken
  for (uint32_t a=0; a<ADCReadings; a++)
  {
      sum+=ADCSamples[a];
  }

  sum=sum/ADCReadings;
  temp=converToCelsius(sum);                                              // Average of this value is send to convert into Celcius
  uart_temperature = (uint8_t *)&temp;
  int i = 0;
  while(i<4)
  {
	  temp_pre[i] = *uart_temperature;
	  uart_temperature++;
	  i++;
  }
  uart_temperature = uart_temperature - 4;

  CircBuff();

	ADC0->CMD |= ADC_CMD_SINGLESTOP;
	ADC_Reset(ADC0);
	CMU_ClockEnable(cmuClock_ADC0, false);
	DMA_Reset();
 	CMU_ClockEnable(cmuClock_DMA, false);
 	unblockSleepMode(EM1);
 	CMU_ClockEnable(cmuClock_GPIO, true);
}


void print_temp(void)
{
		 SegmentLCD_Init(false);
		 SegmentLCD_AllOff();
		 SegmentLCD_Number(temp);
		 SegmentLCD_Write("TEMPERATURE");
}

float converToCelsius(int32_t adcSample)
{
    float temp;
    float cal_temp_0=(float)((DEVINFO->CAL & _DEVINFO_CAL_TEMP_MASK) >> _DEVINFO_CAL_TEMP_SHIFT);
    float cal_value_0=(float)((DEVINFO->ADC0CAL2 & _DEVINFO_ADC0CAL2_TEMP1V25_MASK )>>_DEVINFO_ADC0CAL2_TEMP1V25_SHIFT);
    float t_grad=-6.27;
    temp = (cal_temp_0 - ((cal_value_0 -adcSample) / t_grad));
    return temp;
}

void  WhenDMAoff(void)
{
      uint32_t sum=0;
      ADCSetup();
      ADC0->IEN|=1;
      for (uint32_t a=0; a<ADCReadings; a++)                                    //total of 1000 readings were noted whithout using DMA but using CPU
      {
          while(((ADC0->IF)&&(1)==0));

          ADCSamples[a]=ADC0->SINGLEDATA;
          sum+=ADCSamples[a];
          ADC0->IFC=1;
      }

      sum=sum/ADCReadings;
      float temp=converToCelsius(sum);                                      // average of this temperature is calculated in celcius

      if((temp>templow)&&(temp<temphigh))                                               //if temperature is between 15 and 35 cesisus LED is off else LED is on
      {
          LED1off();
      }
      else
      {
          LED1on();
      }
      unblockSleepMode(EM1);
}

void ACMPlowSetup()
{
    blockSleepMode(EM3);

    ACMP_Init_TypeDef acmp0init= ACMP_INIT_DEFAULT;                                                                                     //ACMP INITILISATION
    acmp0init.fullBias = false;
    acmp0init.biasProg = 0x00000000 ;
    acmp0init.vddLevel = 2;                                                                                                             //vdd level is set to 2/63 *vdd
    acmp0init.enable = true;

    ACMP_Init(ACMP0,&acmp0init);
    ACMP_ChannelSet(ACMP0,acmpChannelVDD,acmpChannel6);                                                                                 //Wait till warmup time
    while((ACMP0->STATUS & ACMP_STATUS_ACMPACT) != 1);                                                  //Channel 6 is initialised which is Ambient light sensor
    ACMP0->INPUTSEL |= 0x00010000;
}

void ACMPhighSetup()
{
    blockSleepMode(EM3);

    ACMP_Init_TypeDef acmp1init= ACMP_INIT_DEFAULT;                                                                                     //ACMP INITIALISATION
    acmp1init.fullBias = false;
    acmp1init.biasProg = 0x00000000 ;
    acmp1init.vddLevel = 60;                                                                                                            //Vdd level is sed to 61/63 * vdd
    acmp1init.enable = true;

    ACMP_Init(ACMP0,&acmp1init);

    ACMP_ChannelSet(ACMP0,acmpChannelVDD,acmpChannel6);                                                 //Channel 6 is initialised which id ambient light sensor
    while((ACMP0->STATUS & ACMP_STATUS_ACMPACT) != 1);                                                                                  //wait till warmup
    ACMP0->INPUTSEL |= 0x00010000;
}

 void LETIMER0_IRQHandler(void)
 {

	 int  interruptflag;
	 interruptflag = LETIMER0->IF;
	 LETIMER0->IFC= interruptflag;

	 if ((interruptflag & LETIMER_IF_COMP1) !=0)
  	 	 {

		 LS_on();

  	 	 }
	 else if ((interruptflag & LETIMER_IF_UF) !=0)
	 	 {

		 if(Light_Sense)
		 {


			 ACMPlowSetup();                                                                                                 //ACMP with 2/63 vdd level is configured
			 a=ACMP0->STATUS;                                                                                                    // its status bits are noted
		     ACMP_Disable(ACMP0);
		     ACMPhighSetup();                                                                                                //ACMP with 61/63 vdd level configured
		     b=ACMP0->STATUS;                                                                                                    //its status bits are noted
		     ACMP_Disable(ACMP0);
		     LS_off();

		     if((a==acmphigh)&&(b==acmphigh))                                                                                                //if both ACMP outputs are 1 then led0 is off
		      {
		          LED1off();
		          unblockSleepMode(EM3);
		      }
		      else if((a==acmplow)&&(b==acmplow))                                                                                        //if both ACMP outputs are 0 then led0 is on
		      {
		          LED1on();
		          unblockSleepMode(EM3);
		      }

		 }

     	 if(ADC)
     	 {
     		 ADCSetup();
     		 DMASetup();
     		 DMA_ActivateBasic(DMAchannel,
     				 true,
     				 false,
     				 (void *)ADCSamples,
     				 (void *)&(ADC0->SINGLEDATA),
     				 ADCReadings - 1);
     		 ADC_Start(ADC0, adcStartSingle);

     	 }
        if(BME)
        {
           	BMEI2CSetup();
        	BMEEnable();
        	BMESetup();
        	RawData();
        	DataCalibration();
        	BMEDisable();
        }


 	 }
	 LCD_off();

 }

 void CircBuff(void)
 {

	 if(circular_buffer)
	 {
	     uint8_t i=0;
	     while(i<4)
	     {
	     add_item(tx_buffer, *uart_temperature);

	     uart_temperature++;
	     i++;
	     }
	     i = 0;

	     uart_pressure();

	     uart_letter();


	 }
       LEUART0Setup();
       LEUART_FreezeEnable(LEUART0, true);
       LEUART0->CMD |= LEUART_CMD_TXEN;
       while(!(LEUART0->SYNCBUSY&LEUART_SYNCBUSY_CMD));
       LEUART0->ROUTE |= LEUART_ROUTE_TXPEN;
       LEUART0->IEN = LEUART_IEN_TXBL;
       LEUART_FreezeEnable(LEUART0, false);
 }

 void LCD_off(void)
 {
		 SegmentLCD_Disable();
		 //CMU_OscillatorEnable(cmuOsc_LFRCO, false, false);
 }

 void LS_on(void)
 {
	 CMU_ClockEnable(cmuClock_ACMP0, true);
	 CMU_ClockEnable(cmuClock_GPIO, true);
	 	  /* Initialize the 2 GPIO pins of the light sensor setup. */
 	  GPIO_PinModeSet(LIGHTSENSE_EXCITE_PORT, LIGHTSENSE_EXCITE_PIN, gpioModePushPull, 0);
	  GPIO_PinModeSet(LIGHTSENSE_SENSOR_PORT, LIGHTSENSE_SENSOR_PIN, gpioModeDisabled, 0);
	  GPIO_PinOutSet(LIGHTSENSE_EXCITE_PORT,LIGHTSENSE_EXCITE_PIN);


 }

 void LS_off(void)
 {

	  GPIO_PinOutClear(LIGHTSENSE_EXCITE_PORT,LIGHTSENSE_EXCITE_PIN);

	  ACMP_Disable(ACMP0);
	  CMU_ClockEnable(cmuClock_ACMP0, false);
 }

 void pattern_check(uint16_t channels_touched)
 {

	 if((channels_touched&256))
	 {
		 if(disp == 0)
		 {
			 print_letters();
			 disp++;
		 }
		 else if(disp == 1)
		 {
			 print_pressure();
			 disp++;
		 }
		 else if(disp == 2)
		 {
			 print_temp();
			 disp = 0;
		 }
	 }
	    if((channels_touched&2048))
	    {
	    	while(1)
	    	{
	    	channels_touched = LETOUCH_GetChannelsTouched();
	    	 if((channels_touched&1024))
	    	 {
	    	    	while(1)
	    	    	{
	    	    	channels_touched = LETOUCH_GetChannelsTouched();
	    	    	 if((channels_touched&256))
	    	    	 {
	    	    	    	while(1)
	    	    	    	{
	    	    	    	channels_touched = LETOUCH_GetChannelsTouched();
	    	    	    	 if((channels_touched&512))
	    	    	    	 {

	    	    	    		 SegmentLCD_Init(false);
	    	    	    		 SegmentLCD_AllOff();
	    	    	    		 SegmentLCD_Write("GRANT");
	    	    	    		 pattern = true;
	    	    	    		 break;
	    	    	    	 }
	    	    	    	 else if(channels_touched == 0)
	    	    	    	 {

	    	    	    	 }
	    	    	    	 else if(channels_touched&256)
	    	    	    	 {

	    	    	    	 }
	    	    	    	 else
	    	    	    		 break;
	    	    	    	 if(pattern)
	    	    	    		 break;
	    	    	    	 EMU_EnterEM2(true);

	    	    	    	}
	    	    	 }
	    	    	 else if(channels_touched == 0)
	    	    	 {

	    	    	 }
	    	    	 else if(channels_touched&1024)
	    	    	 {

	    	    	 }
	    	    	 else
	    	    		 break;
	    	    	 if(pattern)
	    	    	     break;
	    	    	 EMU_EnterEM2(true);
	    	    	}
	    	 }
	    	 else if(channels_touched == 0)
	    	 {

	    	 }
	    	 else if(channels_touched&2048)
	    	 {

	    	 }
	    	 else
	    		 break;
	    	 if(pattern)
	    	 {
	    		 pattern = false;
	    	     break;
	    	 }
	    	 EMU_EnterEM2(true);
	    	}
	    }
	    else if((channels_touched&0x10))
	    {

	    }
	    else if(channels_touched == 0)
	    {

	    }
 }

int main(void)
{
	tx_buffer = &buffer1;
	circ_buff_initialize(tx_buffer, 20);
	add_item(tx_buffer, 1);
	remove_item(tx_buffer);
	CHIP_Init();                                                                                                                /* Align different chip revisions */
	blockSleepMode(EM3);                                                                                                             /*Initialize block sleep mode*///
	if(ULFRCO_Config)
	{
		ULFRCOSetup();
	}
	CMUConfiguration();                                                                                                           /*initialize Clock management Unit*/
	LETIMER0Setup();				/* LETIMER is configured*/
	if(MMA)
	{
		accelerometer();
	}

	if(TOUCH)
	{
		float sensitivity[] = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,1.0, 1.0, 1.0, 1.0, 0.0, 0.0, 0.0, 0.0};
		LETOUCH_Init(sensitivity);
	}

	while (1)
	{
		channels_touched = LETOUCH_GetChannelsTouched();
	    pattern_check(channels_touched);
		Sleep();
	}

	}

