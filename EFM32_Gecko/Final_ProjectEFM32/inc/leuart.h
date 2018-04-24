/*
 * leuart.h
 *
 *  Created on: 26-Nov-2016
 *      Author: RAGHAVENDRA
 */

#ifndef LEUART_H_
#define LEUART_H_

void LEUART0Setup(void);

#define leuartport              gpioPortD
#define leuartpinTx             4
#define atmelledOn              1
#define atmelledOff             2
#define BaudRate                9600
#define pd4andpd5               LEUART_ROUTE_LOCATION_LOC0

uint8_t uart_bytes;


uint8_t tx_data[9];

#endif /* LEUART_H_ */
