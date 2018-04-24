# IOT_Final_Project
##Functionality of Project:
1.	LFXO oscillator is enabled and sleep counter is set
2.	ULFRCO is calibrated using TIMER 1 and TIMER 2 by using HFRCO and LFRCO.
3.	LETIMER0 is enabled and underflow flag is set for every 10 seconds. An interrupt occurs when COMP1 flag and UF flag set.
4.	LESENSE is initialized with channel 8, 9, 10 and 11 for scanning. ACMP1 is used by LESENSE and interrupt occurs when any of the channel is touched.
5.	Accelerometer Is turned on and all the control registers were configured for motion detection using I2C1 in Route 0 position. SDA and SCL ports are disabled.
6.	A function for pattern is called in while loop. It checks for channel 11 interrupt, then for channel 10 interrupt, then for channel 8 interrupt and finally for channel 9 interrupt. If any of the case fails EFM32 goes into EM2 mode.
7.	When the COMP1 interrupt occurs, Light Sense is turned on which stands for 0.04 seconds.
8.	When Underflow interrupt occurs ACMP0 status is recorded twice with both high and low ambient light values. LED0 is turned on and off per the status of ACMP0. ACMP0 and light sense is turned off.
9.	Pressure sensor is configured and uncalibrated readings were recorded and appropriate calculations were made to generalize pressure. BME 280 is configured using I2C1 on route 2 position. After pressure being calculated SDA, SCL and Vdd input for BME is disabled.
10.	 ADC is initialized with temperature sensor as input. 1000 readings were recorded and transferred using DMA. Average of these values were calculated.
11.	Finally, all these values are sent to a circular buffer. After all the values being added LEUART is enabled and transfer begins.
12.	These values were read in SAMB11 and first 4 bytes were for temperature service and another four for pressure service.
13.	After connecting to SAMB11 using ATMEL SMART Temperature Service and Battery Service would be visible which displays temperature and pressure respectively.
14.	 When channel 8 of LESENSE is touched once, number of interrupts were displayed on LCD. If its pressed again LCD should display temperature and on pressing channel 8 again LCD should display pressure.
15.	 If the channels of LESENSE were pressed in the following order 11, 10, 8 and 9 “GRANTED” would be displayed on LCD.
16.	LCD is turned off in LETIMER0 IRQ handler.

