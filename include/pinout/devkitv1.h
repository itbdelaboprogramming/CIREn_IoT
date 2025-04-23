/**
 * @file devkitv1.h
 * @brief Pinout for ESP32 DevKit V1
 * @details This file defines the pinout for the ESP32 DevKit V1 board.
 */

#pragma once

// =========================
// General Purpose I/O Pins
// =========================
#define PIN_GPIO0    0   // Input: No | Output: Yes | Strapping pin
#define PIN_GPIO1    1   // TX0 - Serial debug/programming
#define PIN_GPIO2    2   // Onboard LED, boot mode strapping
#define PIN_GPIO3    3   // RX0 - Serial debug/programming
#define PIN_GPIO4    4
#define PIN_GPIO5    5   // Strapping pin
#define PIN_GPIO12  12   // Strapping pin - caution: affects flash voltage
#define PIN_GPIO13  13
#define PIN_GPIO14  14
#define PIN_GPIO15  15   // Strapping pin - affects boot message
#define PIN_GPIO16  16
#define PIN_GPIO17  17
#define PIN_GPIO18  18
#define PIN_GPIO19  19
#define PIN_GPIO21  21
#define PIN_GPIO22  22
#define PIN_GPIO23  23
#define PIN_GPIO25  25
#define PIN_GPIO26  26
#define PIN_GPIO27  27
#define PIN_GPIO32  32
#define PIN_GPIO33  33
#define PIN_GPIO34  34  // Input only
#define PIN_GPIO35  35  // Input only
#define PIN_GPIO36  36  // Input only
#define PIN_GPIO39  39  // Input only

// ==========
// Peripherals
// ==========

// Canbus
#define CAN_TX      PIN_GPIO21
#define CAN_RX      PIN_GPIO22

// Onboard LED
#define LED_BUILTIN   PIN_GPIO2

// UARTs
#define UART0_RX      PIN_GPIO3
#define UART0_TX      PIN_GPIO1
#define UART1_RX      9   // Default: Not usable on most boards
#define UART1_TX      10  // Default: Not usable on most boards
#define UART2_RX      PIN_GPIO16
#define UART2_TX      PIN_GPIO17

// VSPI (default Arduino SPI)
#define SPI_COPI      PIN_GPIO23  // MOSI
#define SPI_CIPO      PIN_GPIO19  // MISO
#define SPI_SCK       PIN_GPIO18
#define SPI_CS        PIN_GPIO5

// HSPI (alternate SPI)
#define HSPI_COPI     PIN_GPIO13
#define HSPI_CIPO     PIN_GPIO12
#define HSPI_SCK      PIN_GPIO14
#define HSPI_CS       PIN_GPIO15

// ADC Channels (ADC1 and ADC2)
// NOTE: GPIOs 36-39 (input-only) are connected to ADC1
#define ADC1_CH0      PIN_GPIO36
#define ADC1_CH3      PIN_GPIO39
#define ADC1_CH6      PIN_GPIO34
#define ADC1_CH7      PIN_GPIO35
#define ADC1_CH4      PIN_GPIO32
#define ADC1_CH5      PIN_GPIO33
#define ADC2_CH0      PIN_GPIO4
#define ADC2_CH1      PIN_GPIO0
#define ADC2_CH2      PIN_GPIO2
#define ADC2_CH3      PIN_GPIO15
#define ADC2_CH4      PIN_GPIO13
#define ADC2_CH5      PIN_GPIO12
#define ADC2_CH6      PIN_GPIO14
#define ADC2_CH7      PIN_GPIO27
#define ADC2_CH8      PIN_GPIO25
#define ADC2_CH9      PIN_GPIO26


