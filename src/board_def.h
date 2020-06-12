#ifndef BOARD_DEF_H
#define BOARD_DEF_H

// Lolin ESP32 lite


#define ENABLE_SSD1306


#define SSD1306_ADDRESS     0x3c
#ifdef ENABLE_SSD1306
#include "SSD1306.h"
#include "OLEDDisplayUi.h"
#define SSD1306_OBJECT()    (SSD1306 oled(SSD1306_ADDRESS, I2C_SDA, I2C_SCL);OLEDDisplayUi ui(&oled))

#else
#define SSD1306_OBJECT()
#endif


//GPS
#include <TinyGPS++.h>
#define UBLOX_GPS_OBJECT()  TinyGPSPlus gps
#define GPS_BAUD_RATE       9600
#define GPS_RX_PIN          16 // RX UART2
#define GPS_TX_PIN          17 // TX UART2
#define BUTTON_PIN          32
#define BUTTON_PIN_MASK     GPIO_SEL_32


#define I2C_SDA             18
#define I2C_SCL             19
#define LED_BUILTIN         22

//#define ENABLE_BUZZER
#define BUZZER_PIN          4

// Battery Monitoring
#define ADC_BAT_PIN         34

// Status
#define IDLE                0
#define WAIT_GPS            1
#define GPS_FAIL            2
#define WAIT_HOME           3
#define OPERATING           4
#define EDIT                5




#endif /*BOARD_DEF_H*/
