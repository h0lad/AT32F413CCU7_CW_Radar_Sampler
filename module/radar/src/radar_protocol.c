// SPDX-License-Identifier: AGPL-3.0-or-later
#include "radar_protocol.h"

static uint32_t crc32c_update_byte(uint32_t crc, uint8_t data)
{
  uint32_t value = crc ^ data;

  for(uint32_t bit = 0; bit < 8U; ++bit)
  {
    if((value & 1U) != 0U)
    {
      value = (value >> 1) ^ CRC32C_POLYNOMIAL_REV;
    }
    else
    {
      value >>= 1;
    }
  }

  return value;
}

void radar_write_le16(uint8_t *dst, uint16_t value)
{
  dst[0] = (uint8_t)(value & 0xFFU);
  dst[1] = (uint8_t)((value >> 8) & 0xFFU);
}

void radar_write_le32(uint8_t *dst, uint32_t value)
{
  dst[0] = (uint8_t)(value & 0xFFU);
  dst[1] = (uint8_t)((value >> 8) & 0xFFU);
  dst[2] = (uint8_t)((value >> 16) & 0xFFU);
  dst[3] = (uint8_t)((value >> 24) & 0xFFU);
}

uint32_t radar_crc32c_compute(const uint8_t *data, uint32_t length)
{
  uint32_t crc = 0xFFFFFFFFU;

  for(uint32_t index = 0; index < length; ++index)
  {
    crc = crc32c_update_byte(crc, data[index]);
  }

  return ~crc;
}

uint16_t radar_cobs_encode(const uint8_t *src, uint16_t src_length, uint8_t *dst)
{
  uint16_t read_index = 0U;
  uint16_t write_index = 1U;
  uint16_t code_index = 0U;
  uint8_t code = 1U;

  while(read_index < src_length)
  {
    if(src[read_index] == 0U)
    {
      dst[code_index] = code;
      code_index = write_index++;
      code = 1U;
    }
    else
    {
      dst[write_index++] = src[read_index];
      ++code;

      if(code == 0xFFU)
      {
        dst[code_index] = code;
        code_index = write_index++;
        code = 1U;
      }
    }

    ++read_index;
  }

  dst[code_index] = code;
  return write_index;
}
