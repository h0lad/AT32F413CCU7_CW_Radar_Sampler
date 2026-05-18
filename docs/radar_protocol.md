# Radar Stream Protocol

This project sends COBS-framed binary records over UART.

## Transport

- Baud rate: `6000000`
- UART format: `8N1`
- Frame delimiter: trailing `0x00`
- Payload encoding: COBS
- Integrity: CRC32C on the decoded frame body

## Common Frame Layout

Decoded frame body:

```text
u8   version
u8   frame_format
u16  sample_pairs_or_payload_bytes
u32  sequence
u32  sample_index
u32  sample_rate_hz
u8[] payload
u32  crc32c
```

- `version`: currently `0x02`
- `sequence`: monotonically increasing frame counter
- `sample_index`: index of the first I/Q pair carried by the frame
- `sample_rate_hz`: current ADC pair rate

## Frame Formats

### `0x01` IQ Data

Payload layout:

```text
sample_pairs * [
  u16 i_code_le
  u16 q_code_le
]
```

- ADC codes are unsigned
- Host centers them around `2048`

### `0x02` Metadata

Payload layout:

```text
char[4] magic = "RADR"
u32 firmware_version
u32 carrier_khz
u32 uart_baud
u32 sample_rate_hz
u16 frame_samples
u8  adc_bits
u8  adc_channels
u32 header_size
u32 payload_size
```

Metadata is sent periodically so a receiver can attach at any time.

### `0x03` Device Telemetry

Payload layout:

```text
u32 iq_frames_built
u32 uart_frames_completed
u32 adc_event_overflows
u32 tx_backpressure
u32 control_backpressure
u32 adc_dma_errors
u32 uart_dma_errors
u32 reserved0
u32 stream_fault
u8  queue_high_watermark
u8  reserved1
u8  reserved2
u8  reserved3
u32 reserved4
u32 reserved5
```

Relevant counters:

- `adc_event_overflows`: ADC half/full events could not be queued
- `tx_backpressure`: IQ frames dropped because no TX buffer was available
- `control_backpressure`: metadata or telemetry frame skipped because TX was busy
- `stream_fault`: hard device-side failure state
- `queue_high_watermark`: highest observed ADC event queue depth

## Assumption based on measuring the static VCO voltage

Host and firmware currently use:

```text
24.250 GHz
```

This is used for Doppler-to-velocity conversion:

```text
v_radial = f_doppler * c / (2 * f_carrier)
```
