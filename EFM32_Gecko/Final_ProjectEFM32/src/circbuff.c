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


#include<stdio.h>
#include<stdint.h>
#include<stdlib.h>
#include "circbuff.h"

struct circular_buffer buffer;
struct node declare_node;
struct node *circbuffer;

void circ_buff_initialize(struct circular_buffer *buffer, uint32_t max_length)
{
	 circbuffer=malloc(sizeof(declare_node)*max_length);
	 buffer->max_length = max_length;
	 buffer->buffer_start = circbuffer;
	 buffer->buffer_end = buffer->buffer_start + sizeof(declare_node)*(buffer->max_length - 1);
	 (buffer->buffer_end)->next_data = buffer->buffer_start;
	 buffer->current_length=0;

	 buffer->head = buffer->buffer_start;
	 buffer->tail = buffer->buffer_start - sizeof(declare_node);
	 (buffer->tail)-> next_data = buffer->head;
	 (buffer->head)->next_data = buffer->head + sizeof(declare_node);


}

void circ_buff_destroy(struct circular_buffer *buffer)
{
	free(circbuffer);
	free(buffer);

}

int buffer_full(struct circular_buffer *buffer)
{

	if(buffer->current_length==buffer->max_length)
	{
		return 1;
	}
	else
		return 0;

}

int buffer_empty(struct circular_buffer *buffer)
{

	if(buffer->current_length==bufferempty)
	{
		return 1;
	}
	else
		return 0;
}

void add_item(struct circular_buffer *buffer, uint8_t item)
{
	struct node *temp;

	if(buffer->current_length!=buffer->max_length)
	{
		temp=(buffer->tail)->next_data;
		temp->data=item;
		if(temp!=buffer->buffer_end)
		{
		temp->next_data = temp + sizeof(declare_node);
		}
		buffer->tail=temp;
		buffer->current_length++;
	}

	else
	{
		temp=buffer->head;
		temp->data=item;
		buffer->tail = temp;
		buffer->head=temp->next_data;
	}
}


uint8_t remove_item(struct circular_buffer *buffer)
{
	struct node *temp;


	if(buffer->current_length!=bufferempty)
	{
		temp=buffer->head;
		buffer->head=temp->next_data;
		buffer->current_length--;
	}


 
	else
	{
	    exit(1);
	}
	return temp->data;
}
