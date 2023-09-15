#ifndef _BOARD_CONFIG_V4_LITE_H_
#define _BOARD_CONFIG_V4_LITE_H_

// SPI PIN Map
#define MCP_SCK   -1
#define MCP_MOSI  -1
#define MCP_MISO  -1
#define MCP_CS    -1
#define MCP_INT   -1

// Isolation Control PIN
// #define ISO_PIN   5
#define ISO_OVERRIDE 0

// CAN Controller PIN Map
#define CAN_TX    6
#define CAN_RX    7

// LED PIN Map
#define LED_PIN   8

// CAN ID for OIL Pressure
#define OIL_PRESSURE_CAN_ID 0x662

// Resistors value in kOhm for the voltage divider.
#define OIL_PRESSURE_R1 10
#define OIL_PRESSURE_R2 20

// Characteristics of the oil pressure sensor.
// Voltage in mV.
#define OIL_PRESSURE_VL 500
#define OIL_PRESSURE_VH 4500
#define OIL_PRESSURE_PMAX 150

// Define CANMIM_PRINT_METRICS to print sample data every second to log
#define CANMIM_PRINT_METRICS

#define BLE_DEVICE_NAME_BASE "ECAN"

#endif