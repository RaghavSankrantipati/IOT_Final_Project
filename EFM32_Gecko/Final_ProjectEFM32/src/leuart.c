/*
 * leuart.c
 *
 *  Created on: 26-Nov-2016
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
#include "circbuff.h"
#include <stdint.h>
#include <stdbool.h>

uint8_t txbuff[8] = {0};

void LEUART0Setup(void)
{
		CMU_ClockSelectSet(cmuClock_LFB, cmuSelect_LFXO);
		CMU_ClockEnable(cmuClock_LEUART0, true);

		LEUART_Init_TypeDef LEUART0_InitStructure;

	    LEUART0_InitStructure.baudrate = BaudRate;
	    LEUART0_InitStructure.databits = leuartDatabits8;
	    LEUART0_InitStructure.enable = leuartDisable;
	    LEUART0_InitStructure.parity = leuartNoParity;
	    LEUART0_InitStructure.refFreq = 0;
	    LEUART0_InitStructure.stopbits = leuartStopbits1;

		LEUART_Init(LEUART0, &LEUART0_InitStructure);

		CMU_ClockEnable(cmuClock_GPIO, true);
		GPIO_PinModeSet(leuartport, leuartpinTx, gpioModePushPull, 1);

		NVIC_EnableIRQ(LEUART0_IRQn);

		LEUART_FreezeEnable(LEUART0, true);
		LEUART0->CMD = LEUART_CMD_TXEN;
		LEUART0->ROUTE = LEUART_ROUTE_TXPEN | LEUART_ROUTE_LOCATION_LOC0;
		LEUART0->IEN = LEUART_IEN_TXBL;
		LEUART_FreezeEnable(LEUART0, false);

}

void LEUART0_IRQHandler(void)
{

	if(uart_bytes < 6)
	{
		txbuff[uart_bytes] = remove_item(tx_buffer);

		//LEUART0->TXDATA = txbuff[uart_bytes];

		LEUART0->TXDATA = txbuff[uart_bytes];
		while(!(LEUART0->STATUS & LEUART_STATUS_TXC));
		LEUART0->IFC = LEUART_IEN_TXC;
		uart_bytes++;
	}
	else
	{
		uart_bytes = 0;
		LEUART_FreezeEnable(LEUART0, true);
	    LEUART0->CMD &= ~LEUART_CMD_TXEN;
	    LEUART0->ROUTE &= ~LEUART_ROUTE_TXPEN;
	    LEUART0->IEN &= ~LEUART_IEN_TXBL;
	    LEUART_FreezeEnable(LEUART0, false);

	    GPIO_PinModeSet(leuartport, leuartpinTx, gpioModeDisabled, 0);
	    //CMU_ClockEnable(cmuClock_GPIO, false);
	    CMU_ClockEnable(cmuClock_LEUART0, false);
	}
}
