/*
 * Copyright (c) 2015, Freescale Semiconductor, Inc
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted provided that the following conditions are met:
 *
 * o Redistributions of source code must retain the above copyright notice, this list
 *   of conditions and the following disclaimer.
 *
 * o Redistributions in binary form must reproduce the above copyright notice, this
 *   list of conditions and the following disclaimer in the documentation and/or
 *   other materials provided with the distribution.
 *
 * o Neither the name of Freescale Semiconductor, Inc. nor the names of its
 *   contributors may be used to endorse or promote products derived from this
 *   software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
 * ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
 * WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR
 * ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
 * (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 * LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON
 * ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 * (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
 * SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */
#ifndef CIRCBUFF_H
#define CIRCBUFF_H
#include<stdint.h>
#include<stdbool.h>

struct circular_buffer *tx_buffer;
struct circular_buffer buffer1;

struct node
{
	uint8_t data;
	struct node *next_data;
};

struct circular_buffer
{
	struct node *buffer_start;
	struct node *buffer_end;
	struct node *head;
	struct node *tail;
	uint32_t current_length;
	uint32_t max_length;
};


void circ_buff_initialize(struct circular_buffer *buffer1, uint32_t max_length);
void circ_buff_destroy(struct circular_buffer *buffer);
int buffer_full(struct circular_buffer *buffer1);
int buffer_empty(struct circular_buffer *buffer1);
void add_item(struct circular_buffer *buffer1, uint8_t item);
uint8_t remove_item(struct circular_buffer *buffer1);



typedef enum
{
bufferempty=0,
buffernotempty=1
}buffer_state;
#endif
