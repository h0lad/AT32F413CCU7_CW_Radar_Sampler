// SPDX-License-Identifier: AGPL-3.0-or-later
#ifndef RADAR_STREAM_H
#define RADAR_STREAM_H

#include "at32f413_board.h"

typedef enum
{
  STREAM_FAULT_NONE = 0,
  STREAM_FAULT_ADC_EVENT_OVERFLOW,
  STREAM_FAULT_TX_BACKPRESSURE,
  STREAM_FAULT_ADC_DMA_ERROR,
  STREAM_FAULT_UART_DMA_ERROR
} stream_fault_t;

void radar_stream_init(void);
void radar_stream_poll(void);
stream_fault_t radar_stream_fault(void);
uint8_t radar_stream_has_fault(void);

#endif
