// SPDX-License-Identifier: AGPL-3.0-or-later
#ifndef AT32F413_CONF_H
#define AT32F413_CONF_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>

#if !defined(HEXT_VALUE)
#define HEXT_VALUE ((uint32_t)8000000)
#endif

#define HEXT_STARTUP_TIMEOUT ((uint16_t)0x3000)
#define HICK_VALUE           ((uint32_t)8000000)
#define LEXT_VALUE           ((uint32_t)32768)

#define CRM_MODULE_ENABLED
#define TMR_MODULE_ENABLED
#define RTC_MODULE_ENABLED
#define BPR_MODULE_ENABLED
#define GPIO_MODULE_ENABLED
#define I2C_MODULE_ENABLED
#define USART_MODULE_ENABLED
#define PWC_MODULE_ENABLED
#define CAN_MODULE_ENABLED
#define ADC_MODULE_ENABLED
#define SPI_MODULE_ENABLED
#define DMA_MODULE_ENABLED
#define DEBUG_MODULE_ENABLED
#define FLASH_MODULE_ENABLED
#define CRC_MODULE_ENABLED
#define WWDT_MODULE_ENABLED
#define WDT_MODULE_ENABLED
#define EXINT_MODULE_ENABLED
#define SDIO_MODULE_ENABLED
#define USB_MODULE_ENABLED
#define ACC_MODULE_ENABLED
#define MISC_MODULE_ENABLED

#ifdef CRM_MODULE_ENABLED
#include "at32f413_crm.h"
#endif
#ifdef TMR_MODULE_ENABLED
#include "at32f413_tmr.h"
#endif
#ifdef RTC_MODULE_ENABLED
#include "at32f413_rtc.h"
#endif
#ifdef BPR_MODULE_ENABLED
#include "at32f413_bpr.h"
#endif
#ifdef GPIO_MODULE_ENABLED
#include "at32f413_gpio.h"
#endif
#ifdef I2C_MODULE_ENABLED
#include "at32f413_i2c.h"
#endif
#ifdef USART_MODULE_ENABLED
#include "at32f413_usart.h"
#endif
#ifdef PWC_MODULE_ENABLED
#include "at32f413_pwc.h"
#endif
#ifdef CAN_MODULE_ENABLED
#include "at32f413_can.h"
#endif
#ifdef ADC_MODULE_ENABLED
#include "at32f413_adc.h"
#endif
#ifdef SPI_MODULE_ENABLED
#include "at32f413_spi.h"
#endif
#ifdef DMA_MODULE_ENABLED
#include "at32f413_dma.h"
#endif
#ifdef DEBUG_MODULE_ENABLED
#include "at32f413_debug.h"
#endif
#ifdef FLASH_MODULE_ENABLED
#include "at32f413_flash.h"
#endif
#ifdef CRC_MODULE_ENABLED
#include "at32f413_crc.h"
#endif
#ifdef WWDT_MODULE_ENABLED
#include "at32f413_wwdt.h"
#endif
#ifdef WDT_MODULE_ENABLED
#include "at32f413_wdt.h"
#endif
#ifdef EXINT_MODULE_ENABLED
#include "at32f413_exint.h"
#endif
#ifdef SDIO_MODULE_ENABLED
#include "at32f413_sdio.h"
#endif
#ifdef ACC_MODULE_ENABLED
#include "at32f413_acc.h"
#endif
#ifdef MISC_MODULE_ENABLED
#include "at32f413_misc.h"
#endif
#ifdef USB_MODULE_ENABLED
#include "at32f413_usb.h"
#endif

#ifdef __cplusplus
}
#endif

#endif
