/*
  HardwareSerial.cpp - Hardware serial library for Wiring
  Copyright (c) 2006 Nicholas Zambetti.  All right reserved.

  This library is free software; you can redistribute it and/or
  modify it under the terms of the GNU Lesser General Public
  License as published by the Free Software Foundation; either
  version 2.1 of the License, or (at your option) any later version.

  This library is distributed in the hope that it will be useful,
  but WITHOUT ANY WARRANTY; without even the implied warranty of
  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
  Lesser General Public License for more details.

  You should have received a copy of the GNU Lesser General Public
  License along with this library; if not, write to the Free Software
  Foundation, Inc., 51 Franklin St, Fifth Floor, Boston, MA  02110-1301  USA
  
  Modified 23 November 2006 by David A. Mellis
  Modified 28 September 2010 by Mark Sproul
  Modified 14 August 2012 by Alarus
  Modified 3 December 2013 by Matthijs Kooijman
*/

#include <stdlib.h>
#include <stdio.h>
#include <string.h>
#include <inttypes.h>


#include "HardwareSerial.h"


extern uint8_t g_rxBuffer[128];
extern uint8_t g_rxBuffer8[128];
// Constructors ////////////////////////////////////////////////////////////////

HardwareSerial::HardwareSerial(M4_USART_TypeDef *base) :
    _rx_buffer_head(0), _rx_buffer_tail(0)
{
	uart_base = base;
}

//int HardwareSerial::read(void)
//{
//	uint8_t c = 0;
//
//	LPUART_ReadBlocking(uart_base,&c,1);
//
//	return c;
//}

int HardwareSerial::read(void)
{
  // if the head isn't ahead of the tail, we don't have any characters
  if (_rx_buffer_head == _rx_buffer_tail) {
    return -1;
  } else {
    unsigned char c = HardwareSerial::_rx_buffer[_rx_buffer_tail];
    _rx_buffer_tail = (rx_buffer_index_t)(_rx_buffer_tail + 1) % SERIAL_RX_BUFFER_SIZE;
    return c;
  }
}

int HardwareSerial::available(void)
{
  return ((unsigned int)(SERIAL_RX_BUFFER_SIZE + _rx_buffer_head - _rx_buffer_tail)) % SERIAL_RX_BUFFER_SIZE;
}

void HardwareSerial::flush()
{

}

size_t HardwareSerial::write(uint8_t c)
{
    USART_SendData(uart_base, c);
    return 1;
}

// Actual interrupt handlers //////////////////////////////////////////////////////////////

void HardwareSerial::_rx_complete_callback(unsigned char c)
{
    rx_buffer_index_t i = (unsigned int)(_rx_buffer_head + 1) % SERIAL_RX_BUFFER_SIZE;

    // if we should be storing the received character into the location
    // just before the tail (meaning that the head would advance to the
    // current location of the tail), we're about to overflow the buffer
    // and so we don't write the character or advance the head.
    if (i != _rx_buffer_tail) {
      _rx_buffer[_rx_buffer_head] = c;
      _rx_buffer_head = i;

    }
}
void HardwareSerial::set_buffer_head(rx_buffer_index_t index)
{
	if (index != _rx_buffer_tail) {
	_rx_buffer_head = index;
	}
}

int HardwareSerial::peek(void)
{
//	uint32_t head, tail;
//
//	head = rx_buffer_head_;
//	tail = rx_buffer_tail_;
//	if (head == tail) {
//		__disable_irq();
//		head = rx_buffer_head_;  // reread head to make sure no ISR happened
//		if (head == tail) {
//			// Still empty Now check for stuff in FIFO Queue.
//			int c = -1;	// assume nothing to return
//			if (port->WATER & 0x7000000) {
//				c = port->DATA & 0x3ff;		// Use only up to 10 bits of data
//				// But we don't want to throw it away...
//				// since queue is empty, just going to reset to front of queue...
//				rx_buffer_head_ = 1;
//				rx_buffer_tail_ = 0;
//				rx_buffer_[1] = c;
//			}
//			__enable_irq();
//			return c;
//		}
//		__enable_irq();
//
//	}
//	if (++tail >= rx_buffer_total_size_) tail = 0;
//	if (tail < rx_buffer_size_) {
//		return rx_buffer_[tail];
//	} else {
//		return rx_buffer_storage_[tail-rx_buffer_size_];
//	}
}
