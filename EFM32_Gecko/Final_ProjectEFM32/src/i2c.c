/*
 * I2C.c
 *
 *  Created on: 25-Nov-2016
 *      Author: RAGHAVENDRA
 */

#include "i2c.h"
#include "em_i2c.h"
#include "em_cmu.h"
#include "em_gpio.h"
#include "mma8452q.h"
#include "em_chip.h"
#include "em_int.h"
#include "em_emu.h"
#include "em_letimer.h"
#include "em_timer.h"
#include "em_acmp.h"
#include "em_adc.h"
#include "em_leuart.h"
#include "dmactrl.c"
#include "dmactrl.h"
#include "i2cspm.h"
#include "em_dma.h"
#include "em_i2c.h"
#include "circbuff.h"
#include "leuart.h"
#include "segmentlcd.h"
#include <stdint.h>
#include <stdbool.h>


 float Final_Temperature = 0.0;
 uint16_t Final_Pressure = 0.0;
 uint8_t I2C_tx_buffer[2] = {0x00, 0x00};
 uint8_t I2C_rx_buffer= {0x00};
 uint8_t letters = 0;


void BMEI2CSetup(void)
{

	I2C_Enable(I2C1,false);
    CMU_ClockEnable(cmuClock_I2C1, true);
    I2C_Init_TypeDef I2CInit = I2C_INIT_DEFAULT;

    CMU_ClockEnable(cmuClock_GPIO, true);

    GPIO_PinModeSet(BMESCLport, BMESCLpin, gpioModeWiredAndDrivePullUpFilter, 1);
    GPIO_PinModeSet(BMESDAport, BMESDApin, gpioModeWiredAndDrivePullUpFilter, 1);


     for (int i = 0; i < 9; i++)
     {
       GPIO_PinOutClear(BMESCLport, BMESCLpin);
       GPIO_PinOutSet(BMESCLport, BMESCLpin);
     }
    // GPIO_PinModeSet(BMESCLport, BMESCLpin, gpioModeWiredAndDrivePullUpFilter, 1);


      I2C1->ROUTE = I2C_ROUTE_SDAPEN |
                      I2C_ROUTE_SCLPEN| (2 << _I2C_ROUTE_LOCATION_SHIFT);

      I2C_Init(I2C1, &I2CInit);

      if(I2C1->STATE&I2C_STATE_BUSY)
      {
          I2C1->CMD=I2C_CMD_ABORT;
      }

}

void MMAI2CSetup(void)
{

	//I2C_Enable(I2C1,false);
    CMU_ClockEnable(cmuClock_I2C1, true);
    I2C_Reset(I2C1);
    I2C_Enable(I2C1,false);
    I2C_Init_TypeDef I2CInit = I2C_INIT_DEFAULT;
    CMU_ClockEnable(cmuClock_GPIO, true);
    GPIO_PinModeSet(MMA_SCLPort, MMA_SCLPin, gpioModeWiredAndDrivePullUpFilter, 1);
    GPIO_PinModeSet(MMA_SDAPort, MMA_SDAPin, gpioModeWiredAndDrivePullUpFilter, 1);

     for (int i = 0; i < 9; i++)
     {
       GPIO_PinOutSet(MMA_SCLPort, MMA_SCLPin);
       GPIO_PinOutClear(MMA_SCLPort,MMA_SCLPin);
     }
     GPIO_PinModeSet(MMA_SCLPort, MMA_SCLPin, gpioModeWiredAndDrivePullUpFilter, 1);



      I2C1->ROUTE = I2C_ROUTE_SDAPEN |
                      I2C_ROUTE_SCLPEN| (0 << _I2C_ROUTE_LOCATION_SHIFT);

      I2C_Init(I2C1, &I2CInit);

      if(I2C1->STATE&I2C_STATE_BUSY)
      {
          I2C1->CMD=I2C_CMD_ABORT;
      }

}

void I2C0Write()
{

	I2C1->TXDATA=slave_address0<<1;
	I2C1->CMD = I2C_CMD_START;                   //starts I2C1
	I2C1->IFC=I2C_IFC_START;
	while ((I2C1->IF & I2C_IF_ACK) ==  0)
	{
	}
	I2C1->IFC = I2C_IFC_ACK;
	I2C1->TXDATA=I2C_tx_buffer[0];                // transmits buffer[0]
	while ((I2C1->IF & I2C_IF_ACK) ==  0)
	{

	}
	I2C1->IFC = I2C_IFC_ACK;
	I2C1->TXDATA=I2C_tx_buffer[1];                // transmits buffer[1]
	while ((I2C1->IF & I2C_IF_ACK) ==  0)
	{

	}
    I2C1->IFC = I2C_IFC_ACK;
	I2C1->CMD = I2C_CMD_STOP;                     // stops I2C1
    while ((I2C1->IF & I2C_IF_MSTOP) ==  0)
    {

    }
    I2C1->IFC=I2C_IFC_MSTOP;
}

int I2C0Read()
{
	I2C1->TXDATA=(slave_address0)<<1|0x00;                      // write mode
	I2C1->CMD = I2C_CMD_START;
	I2C1->IFC=I2C_IFC_START;
	while ((I2C1->IF & I2C_IF_ACK) ==  0)
	{

	}
	I2C1->IFC = I2C_IFC_ACK;
	I2C1->TXDATA= reg_address;                                // command to get ADC0 value
	while ((I2C1->IF & I2C_IF_ACK) ==  0)
	{

	}
	I2C1->IFC = I2C_IFC_ACK;
	I2C1->CMD = I2C_CMD_START;
	I2C1->TXDATA=(slave_address0)<<1|0x01;                      // read mode
	I2C1->IFC=I2C_IFC_START;
	while ((I2C1->IF & I2C_IF_ACK) ==  0)
	{

	}
	I2C1->IFC = I2C_IFC_ACK;
	while ((I2C1->IF & I2C_IF_RXDATAV) ==  0)
	{

	}
	data0 = I2C1->RXDATA;                  //  ADC0 LSB value

	I2C1->CMD =I2C_CMD_NACK;
	I2C1->CMD = I2C_CMD_STOP;
	while ((I2C1->IF & I2C_IF_MSTOP) ==  0)
	{

	}
	I2C1->IFC=I2C_IFC_MSTOP;
	return(data0);
}

void I2C1Write(uint8_t reg_address, uint8_t reg_value)
{

	I2C1->TXDATA=MMASlaveAddress<<1;
	I2C1->CMD = I2C_CMD_START;                   //starts I2C1
	I2C1->IFC=I2C_IFC_START;
	while ((I2C1->IF & I2C_IF_ACK) ==  0)
	{
	}
	I2C1->IFC = I2C_IFC_ACK;
	I2C1->TXDATA=reg_address;                // transmits buffer[0]
	while ((I2C1->IF & I2C_IF_ACK) ==  0)
	{

	}
	I2C1->IFC = I2C_IFC_ACK;
	I2C1->TXDATA=reg_value;                // transmits buffer[1]
	while ((I2C1->IF & I2C_IF_ACK) ==  0)
	{

	}
    I2C1->IFC = I2C_IFC_ACK;
	I2C1->CMD = I2C_CMD_STOP;                     // stops I2C1
    while ((I2C1->IF & I2C_IF_MSTOP) ==  0)
    {

    }
    I2C1->IFC=I2C_IFC_MSTOP;
}

int I2C1Read(uint16_t reg_address1)
{
	I2C1->TXDATA=(MMASlaveAddress)<<1|0x00;                      // write mode
	I2C1->CMD = I2C_CMD_START;
	I2C1->IFC=I2C_IFC_START;
	while ((I2C1->IF & I2C_IF_ACK) ==  0)
	{

	}
	I2C1->IFC = I2C_IFC_ACK;
	I2C1->TXDATA= reg_address1;                                // command to get ADC0 value
	while ((I2C1->IF & I2C_IF_ACK) ==  0)
	{

	}
	I2C1->IFC = I2C_IFC_ACK;
	I2C1->CMD = I2C_CMD_START;
	I2C1->TXDATA=(MMASlaveAddress)<<1|0x01;                      // read mode
	I2C1->IFC=I2C_IFC_START;
	while ((I2C1->IF & I2C_IF_ACK) ==  0)
	{

	}
	I2C1->IFC = I2C_IFC_ACK;
	while ((I2C1->IF & I2C_IF_RXDATAV) ==  0)
	{

	}
	data1 = I2C1->RXDATA;                  //  ADC0 LSB value

	I2C1->CMD =I2C_CMD_NACK;
	I2C1->CMD = I2C_CMD_STOP;
	while ((I2C1->IF & I2C_IF_MSTOP) ==  0)
	{

	}
	I2C1->IFC=I2C_IFC_MSTOP;
	return(data1);
}

void BMEEnable()
 {
	 int i=0;
	 GPIO_PinModeSet(BME3Vport,BME3Vpin,gpioModePushPull,0);
	 GPIO_PinOutSet(BME3Vport, BME3Vpin);
	 while(i<1000)
	 	{
	 	   i++;
	 	}
 }

 void BMEDisable()
 {
	 GPIO_PinOutClear(BME3Vport, BME3Vport);
	// GPIO_PinModeSet(I2C1_intport, I2C1_intpin, gpioModeDisabled, 0);
	 GPIO_PinModeSet(BMESCLport, BMESCLpin, gpioModeDisabled , 0);
	 GPIO_PinModeSet(BMESDAport, BMESDApin, gpioModeDisabled , 0);
	 CMU_ClockEnable(cmuClock_I2C1,false);
	 //CMU_ClockEnable(cmuClock_GPIO, true);

 }



 void BMESetup()
 {
	 uint8_t T_ovs = 1;             //Temperature oversampling x 1
	 uint8_t P_ovs = 1;             //Pressure oversampling x 1
	 uint8_t Mode = 3;               //Normal mode
	 uint8_t Standbytime = 5;               //Tstandby 1000ms
	 uint8_t IIR_Filter = 0;             //Filter off

     uint8_t control_reg_data;
     uint8_t config_reg_data;
	 control_reg_data = (T_ovs << 5) | (P_ovs << 2) | Mode;
	 config_reg_data = (Standbytime << 5) | (IIR_Filter << 2);

	 I2C_tx_buffer[0] = control_reg;
	 I2C_tx_buffer[1] = control_reg_data;
	 I2C0Write();
	 I2C_tx_buffer[0] = config_reg;
	 I2C_tx_buffer[1] = config_reg_data;
	 I2C0Write();


	 read_trimmed_data();

 }

 void read_trimmed_data()
 {
	  uint8_t data[32];
	  uint8_t i=0;
	  int a=calib00;
      while(i<24)
	 {
	 reg_address = a;
	 data[i]= I2C0Read(1);
	 i++;
	 a++;
	 }

	dig_T1 = (data[1] << 8) | data[0];
    dig_T2 = (data[3] << 8) | data[2];
    dig_T3 = (data[5] << 8) | data[4];
    dig_P1 = (data[7] << 8) | data[6];
    dig_P2 = (data[9] << 8) | data[8];
    dig_P3 = (data[11]<< 8) | data[10];
    dig_P4 = (data[13]<< 8) | data[12];
    dig_P5 = (data[15]<< 8) | data[14];
    dig_P6 = (data[17]<< 8) | data[16];
    dig_P7 = (data[19]<< 8) | data[18];
    dig_P8 = (data[21]<< 8) | data[20];
    dig_P9 = (data[23]<< 8) | data[22];
 }

 void RawData()
 {
	 int i = 0;
	 uint32_t data[8];
	 int d=Pressure_MSB;

	 while(i<6)
	 {
		 reg_address = d;
		 data[i]=I2C0Read(1);
		 i++;
		 d++;
	 }
	 Raw_pressure = (data[0] << 12) | (data[1] << 4) | (data[2] >> 4);
	 Raw_temperature = (data[3] << 12) | (data[4] << 4) | (data[5] >> 4);
 }

 void DataCalibration()
 {
	 signed long int Calibrated_Temperature;
	 unsigned long int Calibrated_Pressure;

	 Calibrated_Temperature = TempCalib(Raw_temperature);
	 Calibrated_Pressure = PressureCalib(Raw_pressure);
	 Final_Temperature = (float)Calibrated_Temperature / 100.0;
	 pressure_final = (float)Calibrated_Pressure / 1000.0;

	 uart_pressure1 = pressure_final;

 }
 void uart_pressure(void)
 {

	 add_item(tx_buffer, uart_pressure1);
 }

 void print_pressure(void)
 {
		 SegmentLCD_Init(false);
		 SegmentLCD_AllOff();
		 SegmentLCD_Number(pressure_final);
		 SegmentLCD_Write("PRESSURE");
 }


 signed long int TempCalib(signed long int adc_T)
 {
	 signed long int var1, var2, T;
	 var1 = ((((adc_T >> 3) - ((signed long int)dig_T1<<1))) * ((signed long int)dig_T2)) >> 11;
	 var2 = (((((adc_T >> 4) - ((signed long int)dig_T1)) * ((adc_T>>4) - ((signed long int)dig_T1))) >> 12) * ((signed long int)dig_T3)) >> 14;
	 t_fine = var1 + var2;
	 T = (t_fine * 5 + 128) >> 8;
	 return T;
 }

 unsigned long int PressureCalib(signed long int adc_P)
 {
     signed long int var1, var2;
     unsigned long int P;
     var1 = (((signed long int)t_fine)>>1) - (signed long int)64000;
     var2 = (((var1>>2) * (var1>>2)) >> 11) * ((signed long int)dig_P6);
     var2 = var2 + ((var1*((signed long int)dig_P5))<<1);
     var2 = (var2>>2)+(((signed long int)dig_P4)<<16);
     var1 = (((dig_P3 * (((var1>>2)*(var1>>2)) >> 13)) >>3) + ((((signed long int)dig_P2) * var1)>>1))>>18;
     var1 = ((((32768+var1))*((signed long int)dig_P1))>>15);
     if (var1 == 0)
     {
         return 0;
     }
     P = (((unsigned long int)(((signed long int)1048576)-adc_P)-(var2>>12)))*3125;
     if(P<0x80000000)
     {
        P = (P << 1) / ((unsigned long int) var1);
     }
     else
     {
         P = (P / (unsigned long int)var1) * 2;
     }
     var1 = (((signed long int)dig_P9) * ((signed long int)(((P>>3) * (P>>3))>>13)))>>12;
     var2 = (((signed long int)(P>>2)) * ((signed long int)dig_P8))>>13;
     P = (unsigned long int)((signed long int)P + ((var1 + var2 + dig_P7) >> 4));
     return P;
 }

 void MMAEnable(void)
 {

	 	GPIO_PinModeSet(MMA_3vPort, MMA_3vPin, gpioModePushPull, 1);

 }

 void MMASetup(void)
 {
	 INT_Disable();
	 int i=0;
	 	while(i<1000)
	 		i++;
		I2C1Write(MMA_CTRL_REG1_Address,MMA_CTRL_REG1_Value);
		I2C1Write(MMA_CTRL_REG2_Address,MMA_CTRL_REG2_Value);
		I2C1Write(MMA_CTRL_REG3_Address,MMA_CTRL_REG3_Value);
		I2C1Write(MMA_CTRL_REG4_Address,MMA_CTRL_REG4_Value);
		I2C1Write(MMA_CTRL_REG5_Address,MMA_CTRL_REG5_Value);
		I2C1Write(MMA_XYZ_DATA_CFG_Address,MMA_XYS_DATA_CFG_Value);
		I2C1Write(MMA_MotionConfig_Address,MMA_MotionConfig_Value);
		I2C1Write(MMA_MotionThreshold_Address,MMA_MotionThreshold_Value);
		I2C1Write(MMA_MotionCount_Address,MMA_MotionCount_Value);
		I2C1Write(MMA_CTRL_REG1_Address,MMA_CTRL_REG1_Value | MMA_MASK_ACTIVE);
		INT_Enable();
 }

 void MMADisable(void)
 {
	GPIO_PinModeSet(MMA_SCLPort, MMA_SCLPin, gpioModeDisabled, 0);	//Configuring PC4 to be used as SCL Pin for Active Luminosity Sensor
 	GPIO_PinModeSet(MMA_SDAPort, MMA_SDAPin, gpioModeDisabled, 0);	//Configuring PC5 to be used as SDA Pin for Active Luminosity Sensor
 	GPIO_PinModeSet(MMA_Int1Port, MMA_Int1Pin, gpioModeDisabled, 0);//Disabling interrupt pin
 	GPIO_PinOutClear(MMA_3vPort, MMA_3vPin);						//Disable power pin to sensor
    CMU_ClockEnable(cmuClock_GPIO, false);
 }

 void GPIO_ODD_IRQHandler(void)
 {
		//GPIO_PinOutSet(led1port, led1pin);
		 MMADisable();
		 accelerometer();
		 letters++;
		 print_letters();

 }

 void print_letters(void)
 {
		 SegmentLCD_Init(false);
		 SegmentLCD_AllOff();
		 SegmentLCD_Number(letters);
		 SegmentLCD_Write("LETTER");
 }

 void uart_letter(void)
 {

	 add_item(tx_buffer, letters);
 }

 void MMAIntEnable()
 {
     GPIO_IntDisable(GPIO_ODD_IRQn);
     GPIO_PinModeSet(MMA_Int1Port, MMA_Int1Pin, gpioModeInput, 0);
	 GPIO_ExtIntConfig(MMA_Int1Port, MMA_Int1Pin, 1, false, true, false); //Configuring the GPIO Interrupts
     NVIC_ClearPendingIRQ(GPIO_ODD_IRQn);       //Clearing pending IRQ
     NVIC_EnableIRQ(GPIO_ODD_IRQn);

 }

 void MMAIntDisable()
 {
     NVIC_DisableIRQ(GPIO_ODD_IRQn);
     NVIC_ClearPendingIRQ(GPIO_ODD_IRQn);  //Clearing pending IRQn
     GPIO_IntDisable(GPIO_ODD_IRQn);       //Disabling GPIO_ODD_IRQn
 }

void accelerometer(void)
{
	MMAI2CSetup();
	MMAEnable();
	MMASetup();
	MMAIntEnable();
	GPIO_IntEnable(GPIO_ODD_IRQn);
    GPIO_PinModeSet(MMA_SCLPort, MMA_SDAPort, gpioModeDisabled, 0);
    GPIO_PinModeSet(MMA_SDAPort, MMA_SDAPin, gpioModeDisabled, 0);
	  GPIO_PinOutClear(MMA_SCLPort,MMA_SDAPort);
	  GPIO_PinOutClear(MMA_SDAPort,MMA_SDAPin);
   // CMU_ClockEnable(cmuClock_GPIO, false);
	//MMAIntDisable();
	//MMADisable();
}

