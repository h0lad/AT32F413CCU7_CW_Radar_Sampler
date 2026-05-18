// SPDX-License-Identifier: AGPL-3.0-or-later
#include "radar_hw.h"

#include "radar_config.h"

static void gpio_config(void)
{
  gpio_init_type gpio_init_struct;

  crm_periph_clock_enable(CRM_GPIOA_PERIPH_CLOCK, TRUE);
  crm_periph_clock_enable(CRM_GPIOB_PERIPH_CLOCK, TRUE);

  gpio_default_para_init(&gpio_init_struct);

  gpio_init_struct.gpio_mode = GPIO_MODE_ANALOG;
  gpio_init_struct.gpio_pins = GPIO_PINS_2 | GPIO_PINS_3;
  gpio_init(GPIOA, &gpio_init_struct);

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

static void timer1_trigger_config(void)
{
  crm_clocks_freq_type clocks;
  tmr_output_config_type output_config;
  uint32_t timer_period;

  crm_clocks_freq_get(&clocks);
  crm_periph_clock_enable(CRM_TMR1_PERIPH_CLOCK, TRUE);

  timer_period = (clocks.ahb_freq / ADC_PAIR_RATE_HZ) - 1U;

  tmr_base_init(TMR1, timer_period, 0U);
  tmr_cnt_dir_set(TMR1, TMR_COUNT_UP);
  tmr_clock_source_div_set(TMR1, TMR_CLOCK_DIV1);
  tmr_period_buffer_enable(TMR1, TRUE);

  tmr_output_default_para_init(&output_config);
  output_config.oc_mode = TMR_OUTPUT_CONTROL_PWM_MODE_A;
  output_config.oc_polarity = TMR_OUTPUT_ACTIVE_HIGH;
  output_config.oc_output_state = TRUE;
  output_config.oc_idle_state = FALSE;
  tmr_output_channel_config(TMR1, TMR_SELECT_CHANNEL_1, &output_config);
  tmr_channel_value_set(TMR1, TMR_SELECT_CHANNEL_1, timer_period / 2U);
  tmr_channel_enable(TMR1, TMR_SELECT_CHANNEL_1, TRUE);
  tmr_output_enable(TMR1, TRUE);
  tmr_counter_enable(TMR1, TRUE);
}

static void adc_dma_config(__IO uint32_t *adc_dma_words)
{
  dma_init_type dma_init_struct;

  crm_periph_clock_enable(CRM_DMA1_PERIPH_CLOCK, TRUE);
  nvic_irq_enable(DMA1_Channel1_IRQn, 0, 0);

  dma_reset(DMA1_CHANNEL1);
  dma_default_para_init(&dma_init_struct);
  dma_init_struct.buffer_size = ADC_DMA_WORD_COUNT;
  dma_init_struct.direction = DMA_DIR_PERIPHERAL_TO_MEMORY;
  dma_init_struct.memory_base_addr = (uint32_t)adc_dma_words;
  dma_init_struct.memory_data_width = DMA_MEMORY_DATA_WIDTH_WORD;
  dma_init_struct.memory_inc_enable = TRUE;
  dma_init_struct.peripheral_base_addr = (uint32_t)&ADC1->odt;
  dma_init_struct.peripheral_data_width = DMA_PERIPHERAL_DATA_WIDTH_WORD;
  dma_init_struct.peripheral_inc_enable = FALSE;
  dma_init_struct.priority = DMA_PRIORITY_HIGH;
  dma_init_struct.loop_mode_enable = TRUE;
  dma_init(DMA1_CHANNEL1, &dma_init_struct);

  dma_flexible_config(DMA1, FLEX_CHANNEL1, DMA_FLEXIBLE_ADC1);
  dma_interrupt_enable(DMA1_CHANNEL1, DMA_HDT_INT, TRUE);
  dma_interrupt_enable(DMA1_CHANNEL1, DMA_FDT_INT, TRUE);
  dma_interrupt_enable(DMA1_CHANNEL1, DMA_DTERR_INT, TRUE);
}

static void adc_config(void)
{
  adc_base_config_type adc_base_struct;

  crm_periph_clock_enable(CRM_ADC1_PERIPH_CLOCK, TRUE);
  crm_periph_clock_enable(CRM_ADC2_PERIPH_CLOCK, TRUE);

  adc_reset(ADC1);
  adc_reset(ADC2);
  crm_adc_clock_div_set(CRM_ADC_DIV_6);
  adc_base_default_para_init(&adc_base_struct);
  adc_combine_mode_select(ADC_ORDINARY_SMLT_ONLY_MODE);

  adc_base_struct.sequence_mode = FALSE;
  adc_base_struct.repeat_mode = FALSE;
  adc_base_struct.data_align = ADC_RIGHT_ALIGNMENT;
  adc_base_struct.ordinary_channel_length = 1;

  adc_base_config(ADC1, &adc_base_struct);
  adc_ordinary_channel_set(ADC1, ADC_CHANNEL_2, 1, ADC_SAMPLETIME_13_5);
  adc_ordinary_conversion_trigger_set(ADC1, ADC12_ORDINARY_TRIG_TMR1CH1, TRUE);
  adc_dma_mode_enable(ADC1, TRUE);

  adc_base_config(ADC2, &adc_base_struct);
  adc_ordinary_channel_set(ADC2, ADC_CHANNEL_3, 1, ADC_SAMPLETIME_13_5);
  adc_ordinary_conversion_trigger_set(ADC2, ADC12_ORDINARY_TRIG_TMR1CH1, TRUE);

  adc_enable(ADC1, TRUE);
  adc_enable(ADC2, TRUE);

  adc_calibration_init(ADC1);
  while(adc_calibration_init_status_get(ADC1) != RESET)
  {
  }
  adc_calibration_start(ADC1);
  while(adc_calibration_status_get(ADC1) != RESET)
  {
  }

  adc_calibration_init(ADC2);
  while(adc_calibration_init_status_get(ADC2) != RESET)
  {
  }
  adc_calibration_start(ADC2);
  while(adc_calibration_status_get(ADC2) != RESET)
  {
  }

  dma_channel_enable(DMA1_CHANNEL1, TRUE);
}

static void usart3_dma_tx_config(void)
{
  dma_init_type dma_init_struct;

  crm_periph_clock_enable(CRM_DMA1_PERIPH_CLOCK, TRUE);
  crm_periph_clock_enable(CRM_USART3_PERIPH_CLOCK, TRUE);
  nvic_irq_enable(DMA1_Channel3_IRQn, 1, 0);

  usart_init(USART3, UART_BAUD_RATE, USART_DATA_8BITS, USART_STOP_1_BIT);
  usart_parity_selection_config(USART3, USART_PARITY_NONE);
  usart_transmitter_enable(USART3, TRUE);
  usart_receiver_enable(USART3, TRUE);
  usart_dma_transmitter_enable(USART3, TRUE);
  usart_enable(USART3, TRUE);

  dma_reset(DMA1_CHANNEL3);
  dma_default_para_init(&dma_init_struct);
  dma_init_struct.buffer_size = 0U;
  dma_init_struct.direction = DMA_DIR_MEMORY_TO_PERIPHERAL;
  dma_init_struct.memory_base_addr = 0U;
  dma_init_struct.memory_data_width = DMA_MEMORY_DATA_WIDTH_BYTE;
  dma_init_struct.memory_inc_enable = TRUE;
  dma_init_struct.peripheral_base_addr = (uint32_t)&USART3->dt;
  dma_init_struct.peripheral_data_width = DMA_PERIPHERAL_DATA_WIDTH_BYTE;
  dma_init_struct.peripheral_inc_enable = FALSE;
  dma_init_struct.priority = DMA_PRIORITY_HIGH;
  dma_init_struct.loop_mode_enable = FALSE;
  dma_init(DMA1_CHANNEL3, &dma_init_struct);

  dma_flexible_config(DMA1, FLEX_CHANNEL3, DMA_FLEXIBLE_UART3_TX);
  dma_interrupt_enable(DMA1_CHANNEL3, DMA_FDT_INT, TRUE);
  dma_interrupt_enable(DMA1_CHANNEL3, DMA_DTERR_INT, TRUE);
}

void radar_hw_init(__IO uint32_t *adc_dma_words)
{
  gpio_config();
  usart3_dma_tx_config();
  adc_dma_config(adc_dma_words);
  adc_config();
  timer1_trigger_config();
}
