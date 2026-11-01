#pragma once

#include "driver/uart.h"
#include "esp_gap_ble_api.h"
#include "soc/uart_pins.h"

/* Override these macros here or through compiler definitions. */
#ifndef BLE_TRANSP_UART_NUM
#define BLE_TRANSP_UART_NUM UART_NUM_0
#endif
#ifndef BLE_TRANSP_UART_BAUD_RATE
#define BLE_TRANSP_UART_BAUD_RATE 115200
#endif
// 配置项见uart_pins.h，默认使用UART0的TXD和RXD引脚
#ifndef BLE_TRANSP_UART_TX_PIN
#define BLE_TRANSP_UART_TX_PIN U0TXD_GPIO_NUM
#endif
#ifndef BLE_TRANSP_UART_RX_PIN
#define BLE_TRANSP_UART_RX_PIN U0RXD_GPIO_NUM
#endif
#ifndef BLE_TRANSP_UART_DATA_BITS
#define BLE_TRANSP_UART_DATA_BITS UART_DATA_8_BITS
#endif
#ifndef BLE_TRANSP_UART_PARITY
#define BLE_TRANSP_UART_PARITY UART_PARITY_DISABLE
#endif
#ifndef BLE_TRANSP_UART_STOP_BITS
#define BLE_TRANSP_UART_STOP_BITS UART_STOP_BITS_1
#endif
#ifndef BLE_TRANSP_UART_FLOW_CTRL
#define BLE_TRANSP_UART_FLOW_CTRL UART_HW_FLOWCTRL_DISABLE
#endif
#ifndef BLE_TRANSP_UART_RTS_PIN
#define BLE_TRANSP_UART_RTS_PIN UART_PIN_NO_CHANGE
#endif
#ifndef BLE_TRANSP_UART_CTS_PIN
#define BLE_TRANSP_UART_CTS_PIN UART_PIN_NO_CHANGE
#endif

/* Local MTU limit (23..517); the peer initiates MTU exchange. */
#ifndef BLE_TRANSP_MTU
#define BLE_TRANSP_MTU 517
#endif

/* Choose ESP_BLE_GAP_PHY_1M_PREF_MASK, _2M_PREF_MASK or _CODED_PREF_MASK. */
#ifndef BLE_TRANSP_PHY
#define BLE_TRANSP_PHY ESP_BLE_GAP_PHY_1M_PREF_MASK
#endif
#ifndef BLE_TRANSP_CODED_PHY_OPTIONS
#define BLE_TRANSP_CODED_PHY_OPTIONS ESP_BLE_GAP_PHY_OPTIONS_NO_PREF
#endif
