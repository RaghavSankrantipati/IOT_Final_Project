/*
 * mma8452q.h
 *
 *  Created on: 25-Nov-2016
 *      Author: RAGHAVENDRA
 */

#ifndef MMA8452Q_H_
#define MMA8452Q_H_

#define MMA_3vPort					gpioPortD
#define MMA_3vPin					0
#define MMA_Int1Port				gpioPortD
#define MMA_Int1Pin					1
#define MMA_Int2Port				gpioPortD
#define MMA_Int2Pin					2
#define MMA_SDAPort					gpioPortC
#define MMA_SDAPin					4
#define MMA_SCLPort					gpioPortC
#define MMA_SCLPin					5

#define MMASlaveAddress						0x1D

#define MMA_CTRL_REG1_Address				0x2A
#define MMA_CTRL_REG1_Value					0xFA		//Setting F_Read to 1 (8-bit mode)
#define MMA_MASK_ACTIVE						0x01

#define MMA_CTRL_REG2_Address				0x2B
#define MMA_CTRL_REG2_Value					0x03

#define MMA_CTRL_REG3_Address				0x2C
#define MMA_CTRL_REG3_Value					0x00

#define MMA_CTRL_REG4_Address				0x2D	//Interrupt Enable Register
#define MMA_CTRL_REG4_Value					0x04	//Motion Detection Interrupt Enabled

#define MMA_CTRL_REG5_Address				0x2E	//Interrupt Configuration Register
#define MMA_CTRL_REG5_Value					0x04	//Setting Motion Detection Interrupt to INT1 Pin

#define MMA_XYZ_DATA_CFG_Address			0x0E
#define MMA_XYS_DATA_CFG_Value				0x00	//Setting the dynamic range of 8-bit data to 2g; 0x01 for 4g; 0x02 for 8g

#define MMA_MotionConfig_Address			0x15
#define MMA_MotionConfig_Value				0x78	//ELE = 0

#define MMA_MotionThreshold_Address			0x17
#define MMA_MotionThreshold_Value			0xA0

#define MMA_MotionCount_Address				0x18
#define MMA_MotionCount_Value				0x01	//Sets 641ms time to check if the motion is still being observed (debounce time)

#define MMA_SourceDetectionReg_Address		0x16 //Read Only
#define MMA_INT_SOURCE_Address				0x0C

#endif /* MMA8452Q_H_ */
