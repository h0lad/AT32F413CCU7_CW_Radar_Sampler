// SPDX-License-Identifier: AGPL-3.0-or-later
#include "at32f413_board.h"
#include "board_clock.h"
#include "radar_stream.h"

int main(void)
{
  board_clock_config();
  at32_board_init();
  nvic_priority_group_config(NVIC_PRIORITY_GROUP_4);

  radar_stream_init();

  while(1)
  {
    if(radar_stream_has_fault() != 0U)
    {
      at32_led_toggle(LED4);
      delay_ms(100);
      continue;
    }

    radar_stream_poll();
  }
}
