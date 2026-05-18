// SPDX-License-Identifier: AGPL-3.0-or-later
#include "at32f413_board.h"
#include "board_clock.h"

#define UART_BAUD_RATE 6000000U

static void gpio_config(void)
{
  gpio_init_type gpio_init_struct;

  crm_periph_clock_enable(CRM_GPIOB_PERIPH_CLOCK, TRUE);
  gpio_default_para_init(&gpio_init_struct);

  gpio_init_struct.gpio_drive_strength = GPIO_DRIVE_STRENGTH_STRONGER;
  gpio_init_struct.gpio_out_type = GPIO_OUTPUT_PUSH_PULL;
  gpio_init_struct.gpio_mode = GPIO_MODE_MUX;
  gpio_init_struct.gpio_pins = GPIO_PINS_10;
  gpio_init_struct.gpio_pull = GPIO_PULL_NONE;
  gpio_init(GPIOB, &gpio_init_struct);

  gpio_init_struct.gpio_mode = GPIO_MODE_INPUT;
  gpio_init_struct.gpio_pins = GPIO_PINS_11;
  gpio_init_struct.gpio_pull = GPIO_PULL_UP;
  gpio_init(GPIOB, &gpio_init_struct);
}

static void usart3_config(void)
{
  crm_periph_clock_enable(CRM_USART3_PERIPH_CLOCK, TRUE);

  usart_init(USART3, UART_BAUD_RATE, USART_DATA_8BITS, USART_STOP_1_BIT);
  usart_parity_selection_config(USART3, USART_PARITY_NONE);
  usart_transmitter_enable(USART3, TRUE);
  usart_receiver_enable(USART3, TRUE);
  usart_enable(USART3, TRUE);
}

static void usart3_write_byte(uint8_t byte)
{
  while(usart_flag_get(USART3, USART_TDBE_FLAG) == RESET)
  {
  }

  usart_data_transmit(USART3, byte);
}

static void usart3_write_string(const char *text)
{
  while(*text != '\0')
  {
    usart3_write_byte((uint8_t)*text);
    ++text;
  }
}

int main(void)
{
  static const char hello_message[] = "hello world\r\n";

  board_clock_config();
  at32_board_init();

  gpio_config();
  usart3_config();

  while(1)
  {
    usart3_write_string(hello_message);
  }
}
