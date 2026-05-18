// SPDX-License-Identifier: AGPL-3.0-or-later
#ifndef RADAR_PROTOCOL_H
#define RADAR_PROTOCOL_H

#include "radar_config.h"
#include "at32f413_board.h"

void radar_write_le16(uint8_t *dst, uint16_t value);
void radar_write_le32(uint8_t *dst, uint32_t value);
uint32_t radar_crc32c_compute(const uint8_t *data, uint32_t length);
uint16_t radar_cobs_encode(const uint8_t *src, uint16_t src_length, uint8_t *dst);

#endif
