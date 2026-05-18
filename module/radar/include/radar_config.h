// SPDX-License-Identifier: AGPL-3.0-or-later
#ifndef RADAR_CONFIG_H
#define RADAR_CONFIG_H

#define ADC_PAIR_RATE_HZ                  120000U
#define UART_BAUD_RATE                    6000000U
#define ADC_DMA_PAIRS_PER_HALF            480U
#define ADC_DMA_WORD_COUNT                (ADC_DMA_PAIRS_PER_HALF * 2U)
#define ADC_BYTES_PER_PAIR                4U

#define FRAME_VERSION                     0x02U
#define FRAME_FORMAT_IQ_U16_LE            0x01U
#define FRAME_FORMAT_META                 0x02U
#define FRAME_FORMAT_TELEMETRY            0x03U
#define FRAME_HEADER_SIZE                 16U
#define FRAME_CRC_SIZE                    4U
#define FRAME_PAYLOAD_SIZE                (ADC_DMA_PAIRS_PER_HALF * ADC_BYTES_PER_PAIR)
#define FRAME_BODY_SIZE                   (FRAME_HEADER_SIZE + FRAME_PAYLOAD_SIZE + FRAME_CRC_SIZE)
#define FRAME_ENCODED_MAX_SIZE            (FRAME_BODY_SIZE + (FRAME_BODY_SIZE / 254U) + 3U)

#define TX_BUFFER_COUNT                   6U
#define ADC_EVENT_QUEUE_SIZE              16U
#define CRC32C_POLYNOMIAL_REV             0x82F63B78U
#define RADAR_CARRIER_KHZ                 24250000U
#define FIRMWARE_VERSION                  0x00010000U
#define META_FRAME_INTERVAL               1024U
#define TELEMETRY_FRAME_INTERVAL          128U

#endif
