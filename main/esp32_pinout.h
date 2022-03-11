#ifndef ESP32_PINOUT_H
#define ESP32_PINOUT_H

#define GPIO0 (gpio_num_t) 0 // input pulled up | outputs PWM signal at boot
#define GPIO1 (gpio_num_t) 1 // TX pin | debug output at boot
#define GPIO2 (gpio_num_t) 2
#define GPIO3 (gpio_num_t) 3 // RX pin | HIGH at boot
#define GPIO4 (gpio_num_t) 4
#define GPIO5 (gpio_num_t) 5 // outputs PWM signal at boot
#define GPIO6 (gpio_num_t) 6 // SCK_CLK | connected to the integrated SPI flash
#define GPIO7 (gpio_num_t) 7 // SDO_SD0 | connected to the integrated SPI flash
#define GPIO8 (gpio_num_t) 8 // SDI_SD1 | connected to the integrated SPI flash
#define GPIO9 (gpio_num_t) 9 // SHD_SD2 | connected to the integrated SPI flash
#define GPIO10 (gpio_num_t) 10 // SWP_SD3 | connected to the integrated SPI flash
#define GPIO11 (gpio_num_t) 11 // CSC_CMD | connected to the integrated SPI flash
#define GPIO12 (gpio_num_t) 12 // boot fail if pulled high
#define GPIO13 (gpio_num_t) 13
#define GPIO14 (gpio_num_t) 14 // outputs PWM signal at boot
#define GPIO15 (gpio_num_t) 15 // outputs PWM signal at boot
#define GPIO16 (gpio_num_t) 16
#define GPIO17 (gpio_num_t) 17
#define GPIO18 (gpio_num_t) 18
#define GPIO19 (gpio_num_t) 19
#define GPIO21 (gpio_num_t) 21
#define GPIO22 (gpio_num_t) 22
#define GPIO23 (gpio_num_t) 23
#define GPIO25 (gpio_num_t) 25
#define GPIO26 (gpio_num_t) 26
#define GPIO27 (gpio_num_t) 27
#define GPIO32 (gpio_num_t) 32
#define GPIO33 (gpio_num_t) 33
#define GPIO34 (gpio_num_t) 34 // Input only
#define GPIO35 (gpio_num_t) 35 // Input only
#define GPIO36 (gpio_num_t) 36 // Input only
#define GPIO39 (gpio_num_t) 39 // Input only

// UART
#define TX_PIN GPIO1
#define RX_PIN GPIO3

// Input only pins
#define INPUTONLY34 GPIO34
#define INPUTONLY35 GPIO35
#define INPUTONLY36 GPIO36
#define INPUTONLY39 GPIO39

// SPI flash integrated
#define SCK_CLK GPIO6
#define SDO_SD0 GPIO7
#define SDI_SD1 GPIO8
#define SHD_SD2 GPIO9
#define SWP_SD3 GPIO10
#define CSC_CMD GPIO11

// Capacitive touch pins
#define T0 GPIO4
#define T1 GPIO0
#define T2 GPIO2
#define T3 GPIO15
#define T4 GPIO13
#define T5 GPIO12
#define T6 GPIO14
#define T7 GPIO27
#define T8 GPIO33
#define T9 GPIO32

// Analog to Digital Converter
#define ADC1_CH0 GPIO36
#define ADC1_CH1 GPIO37
#define ADC1_CH2 GPIO38
#define ADC1_CH3 GPIO39
#define ADC1_CH4 GPIO32
#define ADC1_CH5 GPIO33
#define ADC1_CH6 GPIO34
#define ADC1_CH7 GPIO35
#define ADC2_CH0 GPIO4
#define ADC2_CH1 GPIO0
#define ADC2_CH2 GPIO2
#define ADC2_CH3 GPIO15
#define ADC2_CH4 GPIO13
#define ADC2_CH5 GPIO12
#define ADC2_CH6 GPIO14
#define ADC2_CH7 GPIO27
#define ADC2_CH8 GPIO25
#define ADC2_CH9 GPIO26

#endif
