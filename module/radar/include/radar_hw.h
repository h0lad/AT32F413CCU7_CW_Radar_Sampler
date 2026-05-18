// SPDX-License-Identifier: AGPL-3.0-or-later
#ifndef RADAR_HW_H
#define RADAR_HW_H

#include "at32f413_board.h"

void radar_hw_init(__IO uint32_t *adc_dma_words);

#endif
