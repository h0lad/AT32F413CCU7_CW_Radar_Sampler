// SPDX-License-Identifier: AGPL-3.0-or-later
#include "radar_stream.h"

#include "radar_config.h"
#include "radar_hw.h"
#include "radar_protocol.h"

typedef enum
{
  TX_BUFFER_FREE = 0,
  TX_BUFFER_READY,
  TX_BUFFER_ACTIVE
} tx_buffer_state_t;

typedef struct
{
  uint8_t bytes[FRAME_ENCODED_MAX_SIZE];
  uint16_t length;
  volatile tx_buffer_state_t state;
} tx_buffer_t;

static __IO uint32_t g_adc_dma_words[ADC_DMA_WORD_COUNT];
static tx_buffer_t g_tx_buffers[TX_BUFFER_COUNT];
static volatile uint8_t g_adc_event_queue[ADC_EVENT_QUEUE_SIZE];
static volatile uint8_t g_adc_event_read_index;
static volatile uint8_t g_adc_event_write_index;
static volatile uint8_t g_next_fill_buffer;
static volatile uint8_t g_uart_active_buffer = 0xFFU;
static volatile uint8_t g_uart_dma_busy;
static volatile uint32_t g_frame_sequence;
static volatile uint32_t g_control_sequence;
static volatile uint32_t g_next_sample_index;
static volatile stream_fault_t g_stream_fault = STREAM_FAULT_NONE;
static volatile uint32_t g_adc_event_overflow_count;
static volatile uint32_t g_tx_backpressure_count;
static volatile uint32_t g_control_backpressure_count;
static volatile uint32_t g_adc_dma_error_count;
static volatile uint32_t g_uart_dma_error_count;
static volatile uint32_t g_uart_frames_completed;
static volatile uint32_t g_iq_frames_built;
static volatile uint32_t g_next_telemetry_frame = TELEMETRY_FRAME_INTERVAL;
static volatile uint32_t g_next_metadata_frame = META_FRAME_INTERVAL;
static volatile uint8_t g_adc_event_queue_high_watermark;

static void fail_stream(stream_fault_t fault)
{
  if(g_stream_fault == STREAM_FAULT_NONE)
  {
    g_stream_fault = fault;
    tmr_counter_enable(TMR1, FALSE);
    dma_channel_enable(DMA1_CHANNEL1, FALSE);
    dma_channel_enable(DMA1_CHANNEL3, FALSE);
  }
}

static void push_adc_event(uint8_t event_id)
{
  uint8_t next_index = (uint8_t)((g_adc_event_write_index + 1U) % ADC_EVENT_QUEUE_SIZE);
  uint8_t depth;

  if(next_index == g_adc_event_read_index)
  {
    ++g_adc_event_overflow_count;
    return;
  }

  g_adc_event_queue[g_adc_event_write_index] = event_id;
  g_adc_event_write_index = next_index;

  depth = (uint8_t)((g_adc_event_write_index + ADC_EVENT_QUEUE_SIZE - g_adc_event_read_index) % ADC_EVENT_QUEUE_SIZE);
  if(depth > g_adc_event_queue_high_watermark)
  {
    g_adc_event_queue_high_watermark = depth;
  }
}

static int pop_adc_event(uint8_t *event_id)
{
  if(g_adc_event_read_index == g_adc_event_write_index)
  {
    return 0;
  }

  *event_id = g_adc_event_queue[g_adc_event_read_index];
  g_adc_event_read_index = (uint8_t)((g_adc_event_read_index + 1U) % ADC_EVENT_QUEUE_SIZE);
  return 1;
}

static void uart_start_tx_buffer(uint8_t buffer_index)
{
  g_uart_dma_busy = 1U;
  g_uart_active_buffer = buffer_index;
  g_tx_buffers[buffer_index].state = TX_BUFFER_ACTIVE;

  dma_channel_enable(DMA1_CHANNEL3, FALSE);
  DMA1_CHANNEL3->maddr = (uint32_t)g_tx_buffers[buffer_index].bytes;
  dma_data_number_set(DMA1_CHANNEL3, g_tx_buffers[buffer_index].length);
  dma_flag_clear(DMA1_GL3_FLAG);
  dma_channel_enable(DMA1_CHANNEL3, TRUE);
}

static void uart_try_start_next_buffer(void)
{
  if((g_stream_fault != STREAM_FAULT_NONE) || (g_uart_dma_busy != 0U))
  {
    return;
  }

  for(uint8_t index = 0U; index < TX_BUFFER_COUNT; ++index)
  {
    if(g_tx_buffers[index].state == TX_BUFFER_READY)
    {
      uart_start_tx_buffer(index);
      return;
    }
  }
}

static void fill_frame_payload(uint8_t *payload, const __IO uint32_t *src_words)
{
  uint16_t out_index = 0U;

  for(uint32_t sample_index = 0U; sample_index < ADC_DMA_PAIRS_PER_HALF; ++sample_index)
  {
    uint32_t packed = src_words[sample_index];
    uint16_t adc_i = (uint16_t)(packed & 0x0FFFU);
    uint16_t adc_q = (uint16_t)((packed >> 16) & 0x0FFFU);

    radar_write_le16(&payload[out_index], adc_i);
    out_index += 2U;
    radar_write_le16(&payload[out_index], adc_q);
    out_index += 2U;
  }
}

static int reserve_tx_buffer(uint8_t *buffer_index, uint8_t count_as_iq_drop)
{
  for(uint8_t offset = 0U; offset < TX_BUFFER_COUNT; ++offset)
  {
    uint8_t candidate = (uint8_t)((g_next_fill_buffer + offset) % TX_BUFFER_COUNT);
    if(g_tx_buffers[candidate].state == TX_BUFFER_FREE)
    {
      *buffer_index = candidate;
      return 1;
    }
  }

  if(count_as_iq_drop != 0U)
  {
    ++g_tx_backpressure_count;
  }
  else
  {
    ++g_control_backpressure_count;
  }
  return 0;
}

static void finish_tx_frame(uint8_t buffer_index, uint8_t *frame_body, uint16_t frame_body_size)
{
  uint32_t frame_crc;
  uint16_t encoded_length;

  frame_crc = radar_crc32c_compute(frame_body, frame_body_size);
  radar_write_le32(&frame_body[frame_body_size], frame_crc);

  encoded_length = radar_cobs_encode(
    frame_body,
    (uint16_t)(frame_body_size + FRAME_CRC_SIZE),
    g_tx_buffers[buffer_index].bytes
  );
  g_tx_buffers[buffer_index].bytes[encoded_length++] = 0U;
  g_tx_buffers[buffer_index].length = encoded_length;
  g_tx_buffers[buffer_index].state = TX_BUFFER_READY;

  g_next_fill_buffer = (uint8_t)((buffer_index + 1U) % TX_BUFFER_COUNT);
}

static void write_frame_header(
  uint8_t *frame_body,
  uint8_t frame_format,
  uint16_t sample_pairs_or_payload_bytes,
  uint32_t sequence,
  uint32_t sample_index
)
{
  frame_body[0] = FRAME_VERSION;
  frame_body[1] = frame_format;
  radar_write_le16(&frame_body[2], sample_pairs_or_payload_bytes);
  radar_write_le32(&frame_body[4], sequence);
  radar_write_le32(&frame_body[8], sample_index);
  radar_write_le32(&frame_body[12], ADC_PAIR_RATE_HZ);
}

static void build_frame_from_half(uint8_t half_index)
{
  uint8_t buffer_index;
  uint8_t frame_body[FRAME_BODY_SIZE];
  const __IO uint32_t *src_words = &g_adc_dma_words[half_index * ADC_DMA_PAIRS_PER_HALF];

  if(reserve_tx_buffer(&buffer_index, 1U) == 0)
  {
    g_next_sample_index += ADC_DMA_PAIRS_PER_HALF;
    return;
  }

  write_frame_header(
    frame_body,
    FRAME_FORMAT_IQ_U16_LE,
    (uint16_t)ADC_DMA_PAIRS_PER_HALF,
    g_frame_sequence++,
    g_next_sample_index
  );
  fill_frame_payload(&frame_body[FRAME_HEADER_SIZE], src_words);

  finish_tx_frame(buffer_index, frame_body, FRAME_HEADER_SIZE + FRAME_PAYLOAD_SIZE);
  g_next_sample_index += ADC_DMA_PAIRS_PER_HALF;
  ++g_iq_frames_built;
}

static void build_metadata_frame(void)
{
  uint8_t buffer_index;
  uint8_t frame_body[FRAME_HEADER_SIZE + 32U + FRAME_CRC_SIZE];
  uint8_t *payload = &frame_body[FRAME_HEADER_SIZE];

  if(reserve_tx_buffer(&buffer_index, 0U) == 0)
  {
    return;
  }

  write_frame_header(frame_body, FRAME_FORMAT_META, 32U, g_control_sequence++, g_next_sample_index);
  payload[0] = 'R';
  payload[1] = 'A';
  payload[2] = 'D';
  payload[3] = 'R';
  radar_write_le32(&payload[4], FIRMWARE_VERSION);
  radar_write_le32(&payload[8], RADAR_CARRIER_KHZ);
  radar_write_le32(&payload[12], UART_BAUD_RATE);
  radar_write_le32(&payload[16], ADC_PAIR_RATE_HZ);
  radar_write_le16(&payload[20], (uint16_t)ADC_DMA_PAIRS_PER_HALF);
  payload[22] = 12U;
  payload[23] = 2U;
  radar_write_le32(&payload[24], FRAME_HEADER_SIZE);
  radar_write_le32(&payload[28], FRAME_PAYLOAD_SIZE);

  finish_tx_frame(buffer_index, frame_body, FRAME_HEADER_SIZE + 32U);
}

static void build_telemetry_frame(void)
{
  uint8_t buffer_index;
  uint8_t frame_body[FRAME_HEADER_SIZE + 48U + FRAME_CRC_SIZE];
  uint8_t *payload = &frame_body[FRAME_HEADER_SIZE];

  if(reserve_tx_buffer(&buffer_index, 0U) == 0)
  {
    return;
  }

  write_frame_header(frame_body, FRAME_FORMAT_TELEMETRY, 48U, g_control_sequence++, g_next_sample_index);
  radar_write_le32(&payload[0], g_iq_frames_built);
  radar_write_le32(&payload[4], g_uart_frames_completed);
  radar_write_le32(&payload[8], g_adc_event_overflow_count);
  radar_write_le32(&payload[12], g_tx_backpressure_count);
  radar_write_le32(&payload[16], g_control_backpressure_count);
  radar_write_le32(&payload[20], g_adc_dma_error_count);
  radar_write_le32(&payload[24], g_uart_dma_error_count);
  radar_write_le32(&payload[28], g_next_sample_index);
  radar_write_le32(&payload[32], (uint32_t)g_stream_fault);
  payload[36] = g_adc_event_queue_high_watermark;
  payload[37] = g_uart_dma_busy;
  payload[38] = g_adc_event_read_index;
  payload[39] = g_adc_event_write_index;
  radar_write_le32(&payload[40], ADC_EVENT_QUEUE_SIZE);
  radar_write_le32(&payload[44], TX_BUFFER_COUNT);

  finish_tx_frame(buffer_index, frame_body, FRAME_HEADER_SIZE + 48U);
}

void radar_stream_init(void)
{
  radar_hw_init(g_adc_dma_words);
}

void radar_stream_poll(void)
{
  uint8_t event_id;

  while(pop_adc_event(&event_id) != 0)
  {
    build_frame_from_half(event_id);

    if(g_stream_fault != STREAM_FAULT_NONE)
    {
      break;
    }
  }

  if(g_iq_frames_built >= g_next_telemetry_frame)
  {
    build_telemetry_frame();
    g_next_telemetry_frame += TELEMETRY_FRAME_INTERVAL;
  }

  if(g_frame_sequence >= g_next_metadata_frame)
  {
    build_metadata_frame();
    g_next_metadata_frame += META_FRAME_INTERVAL;
  }

  uart_try_start_next_buffer();
}

stream_fault_t radar_stream_fault(void)
{
  return g_stream_fault;
}

uint8_t radar_stream_has_fault(void)
{
  return (g_stream_fault != STREAM_FAULT_NONE) ? 1U : 0U;
}

void DMA1_Channel1_IRQHandler(void)
{
  if(dma_interrupt_flag_get(DMA1_DTERR1_FLAG) != RESET)
  {
    dma_flag_clear(DMA1_DTERR1_FLAG);
    ++g_adc_dma_error_count;
    fail_stream(STREAM_FAULT_ADC_DMA_ERROR);
  }

  if(dma_interrupt_flag_get(DMA1_HDT1_FLAG) != RESET)
  {
    dma_flag_clear(DMA1_HDT1_FLAG);
    push_adc_event(0U);
  }

  if(dma_interrupt_flag_get(DMA1_FDT1_FLAG) != RESET)
  {
    dma_flag_clear(DMA1_FDT1_FLAG);
    push_adc_event(1U);
  }
}

void DMA1_Channel3_IRQHandler(void)
{
  if(dma_interrupt_flag_get(DMA1_DTERR3_FLAG) != RESET)
  {
    dma_flag_clear(DMA1_DTERR3_FLAG);
    ++g_uart_dma_error_count;
    fail_stream(STREAM_FAULT_UART_DMA_ERROR);
  }

  if(dma_interrupt_flag_get(DMA1_FDT3_FLAG) != RESET)
  {
    dma_flag_clear(DMA1_FDT3_FLAG);
    dma_channel_enable(DMA1_CHANNEL3, FALSE);

    if(g_uart_active_buffer < TX_BUFFER_COUNT)
    {
      g_tx_buffers[g_uart_active_buffer].state = TX_BUFFER_FREE;
    }

    g_uart_active_buffer = 0xFFU;
    g_uart_dma_busy = 0U;
    ++g_uart_frames_completed;
  }
}
