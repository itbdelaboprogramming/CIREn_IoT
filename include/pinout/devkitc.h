/**
 * @file esp32_devkitc_pinout.h
 * @brief Pinout mapping for ESP32-DevKitC-1 (ESP32-WROVER-E)
 */

 #pragma once

 // =======================
 // General GPIO Pin Names
 // =======================
 #define GPIO_PIN0      0
 #define GPIO_PIN1      1
 #define GPIO_PIN2      2
 #define GPIO_PIN3      3
 #define GPIO_PIN4      4
 #define GPIO_PIN5      5
#define GPIO_PIN6      6
 #define GPIO_PIN12    12
 #define GPIO_PIN13    13
 #define GPIO_PIN14    14
 #define GPIO_PIN15    15
 #define GPIO_PIN16    16
 #define GPIO_PIN17    17
 #define GPIO_PIN18    18
 #define GPIO_PIN19    19
 #define GPIO_PIN21    21
 #define GPIO_PIN22    22
 #define GPIO_PIN23    23
 #define GPIO_PIN25    25
 #define GPIO_PIN26    26
 #define GPIO_PIN27    27
 #define GPIO_PIN32    32
 #define GPIO_PIN33    33
 #define GPIO_PIN34    34  // Input only
 #define GPIO_PIN35    35  // Input only
 #define GPIO_PIN36    36  // Input only
 #define GPIO_PIN39    39  // Input only
 
 // =======================
 // Peripheral Assignments
 // =======================
 
 // --- ADC Channels ---
 #define ADC1_CH0_PIN       GPIO_PIN36
 #define ADC1_CH3_PIN       GPIO_PIN39
 #define ADC1_CH4_PIN       GPIO_PIN32
 #define ADC1_CH5_PIN       GPIO_PIN33
 #define ADC1_CH6_PIN       GPIO_PIN34
 #define ADC1_CH7_PIN       GPIO_PIN35
 #define ADC1_CH8_PIN       GPIO_PIN25
 #define ADC2_CH0_PIN       GPIO_PIN4
 #define ADC2_CH1_PIN       GPIO_PIN0
 #define ADC2_CH2_PIN       GPIO_PIN2
 #define ADC2_CH3_PIN       GPIO_PIN15
 #define ADC2_CH4_PIN       GPIO_PIN13
 #define ADC2_CH5_PIN       GPIO_PIN12
 #define ADC2_CH6_PIN       GPIO_PIN14
 #define ADC2_CH7_PIN       GPIO_PIN27
 #define ADC2_CH9_PIN       GPIO_PIN26
 
 // --- DAC Channels ---
 #define DAC1_PIN           GPIO_PIN25
 #define DAC2_PIN           GPIO_PIN26
 
 // --- UART ---
 #define UART0_TX_PIN       GPIO_PIN1
 #define UART0_RX_PIN       GPIO_PIN3
 
 // --- SPI (default to VSPI) ---
 #define SPI_MOSI_PIN       GPIO_PIN23
 #define SPI_MISO_PIN       GPIO_PIN19
 #define SPI_SCK_PIN        GPIO_PIN18
 #define SPI_CS_PIN         GPIO_PIN5
 
 // --- I2C (user-defined) ---
 #define I2C_SDA_PIN        GPIO_PIN21
 #define I2C_SCL_PIN        GPIO_PIN22
 
 // --- Touch Pads ---
 #define TOUCH_CH0_PIN      GPIO_PIN4
 #define TOUCH_CH1_PIN      GPIO_PIN0
 #define TOUCH_CH2_PIN      GPIO_PIN2
 #define TOUCH_CH3_PIN      GPIO_PIN15
 #define TOUCH_CH4_PIN      GPIO_PIN13
 #define TOUCH_CH5_PIN      GPIO_PIN12
 #define TOUCH_CH6_PIN      GPIO_PIN14
 #define TOUCH_CH7_PIN      GPIO_PIN27
 #define TOUCH_CH8_PIN      GPIO_PIN33
 #define TOUCH_CH9_PIN      GPIO_PIN32
 
 // --- LED Indicator ---
 #define LED_BUILTIN        GPIO_PIN13   // Most ESP32 DevKitC boards use GPIO2 for onboard LED
 
 // =======================
 // Special Purpose Pins
 // =======================
 #define EN_PIN             2   // Enable pin (CHIP_PU)
 #define PIN_3V3            3   // 3.3V power supply
 #define PIN_5V0           19   // 5V power supply
 #define PIN_GND1          14   // Ground
 #define PIN_GND2           1   // Ground
 
 // =======================
 // Restricted Usage
 // =======================
 // GPIO6-GPIO11 used for SPI Flash
 // GPIO34-GPIO39 are input-only
 // GPIO0, GPIO2, GPIO5, GPIO12, GPIO15 are strapping pins
 