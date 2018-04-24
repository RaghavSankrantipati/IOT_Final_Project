/*
 * I2C.h
 *
 *  Created on: 25-Nov-2016
 *      Author: RAGHAVENDRA
 */

#ifndef I2C_H_
#define I2C_H_

#include <stdint.h>
#include <stdbool.h>

#define BME3Vport gpioPortD                  // i2c port
#define BME3Vpin  3                          // i2c pin

#define BMESDAport gpioPortE                   // serial data line port
#define BMESDApin 0                            // serial data line pin
#define BMESCLport gpioPortE                   // serial clock line port
#define BMESCLpin 1     // serial clock line pin

#define led0port                gpioPortE
#define led0pin                 2U
#define led1port                gpioPortE
#define led1pin                 3U

//#define BME280_ADDRESS 0x76
#define slave_address0 0x76
#define control_reg 0xF4
#define config_reg  0xF5
#define status_reg  0xF3
#define temp_xlsb   0xFC
#define temp_lsb    0xFB
#define temp_msb	0xFA
#define press_xlsb	0xF9
#define press_lsb   0xF8
#define Pressure_MSB	0xF7
#define calib26		0xE1
#define calib41		0xF0
#define calib25		0xA1
#define calib00		0x88
#define reset		0xE0
#define id			0xD0



unsigned long int Raw_temperature,Raw_pressure;
signed long int t_fine;
uint16_t reg_address;

uint8_t data0;
uint8_t data1;

uint16_t dig_T1;
 int16_t dig_T2;
 int16_t dig_T3;
uint16_t dig_P1;
 int16_t dig_P2;
 int16_t dig_P3;
 int16_t dig_P4;
 int16_t dig_P5;
 int16_t dig_P6;
 int16_t dig_P7;
 int16_t dig_P8;
 int16_t dig_P9;

 uint8_t *UART_temp_ptr;
 uint8_t *UART_press_ptr;
 uint8_t byte_num;

 uint16_t pressure;
 uint8_t *uart_temperature;
 uint8_t uart_pressure1;
// uint8_t *uart_letters;

 uint8_t temp_pre[8];
 float pressure_final;



 void BMEI2CSetup(void);
 void MMAI2CSetup(void);
 void I2C0Write();
 int I2C0Read();
 void I2C1Write(uint8_t reg_address, uint8_t reg_value);
 int I2C1Read(uint16_t reg_address1);
 void BMEEnable();
 void BMEDisable();
 void BMESetup();
 void read_trimmed_data();
 void RawData();
 void DataCalibration();
 signed long int TempCalib(signed long int adc_T);
 unsigned long int PressureCalib(signed long int adc_P);
 void MMAEnable(void);
 void MMASetup(void);
 void MMADisable(void);
 void MMAIntEnable();
 void MMAIntDisable();
 void accelerometer(void);
 void print_letters(void);
 void print_pressure(void);
 void uart_letter(void);
 void uart_pressure(void);


#endif /* I2C_H_ */
