#pragma once

#include "driver/gpio.h"
#include "esp_adc/adc_oneshot.h"

/**
 * @file pinmap.h
 * @brief XIAO ESP32-S3 Plus pin assignments — single source of truth.
 *
 * Keep wiring on the board in lockstep with this file; changing a pin
 * here without re-soldering will silently break things.
 *
 * @note D-number → GPIO-number is **not** 1:1 on this board. Confirm
 *       against the Seeed pinout (`docs/datasheets/xiao-esp32s3-plus/`)
 *       before adding a new pin.
 */

/** @name Battery monitor
 *
 *  External divider on D5 (220k/220k + 100nF). The XIAO ESP32-S3 Plus
 *  has no internal battery-sense path despite the "ADC_BAT → GPIO10"
 *  pin-table claim; verified on this board rev. Divider ratio 2.0,
 *  ADC sees ≤2.1 V — fits `ADC_ATTEN_DB_12` with margin.
 *  @{
 */
#define PIN_ADC_BAT GPIO_NUM_6
#define ADC_BAT_UNIT ADC_UNIT_1
#define ADC_BAT_CHANNEL ADC_CHANNEL_5
/** @} */

/** Tactile button on D0. Active-low, idle high via internal pull-up.
 *  RTC-capable on ESP32-S3 (deep-sleep wake source). */
#define PIN_BTN GPIO_NUM_1

/** @name BNO085 control lines (SPI mode)
 *
 *  HINT is active-low; the chip asserts it when a report is available.
 *  RSTN is active-low; held low keeps the chip in hardware reset.
 *  @{
 */
#define PIN_BNO_INT GPIO_NUM_3
#define PIN_BNO_RST GPIO_NUM_4
#define PIN_BNO_CS GPIO_NUM_5
/** @} */

/** @name BNO085 SPI bus (SPI2)
 *
 *  BNO085 silkscreen in SPI mode: SCL=SCK, SDA=MISO, DI=MOSI.
 *  XIAO header label → ESP32-S3 GPIO mapping: D8=GPIO7, D9=GPIO8,
 *  D10=GPIO9.
 *  @{
 */
#define PIN_BNO_SCK GPIO_NUM_7
#define PIN_BNO_MISO GPIO_NUM_8
#define PIN_BNO_MOSI GPIO_NUM_9
/** @} */
