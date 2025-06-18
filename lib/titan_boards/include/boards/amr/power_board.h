#ifndef BOARDS__POWER_BOARD_H_
#define BOARDS__POWER_BOARD_H_

#include "boards/mk2/blocks/rp2040_can_block.h"

#ifdef UWRT_BOARD_DEFINED
#error Multiple board types defined
#endif
#define UWRT_BOARD_DEFINED

// This defines which CAN bus this board is connected into
// The CAN bus is defined in the corresponding robot definition files (rate, enable FD, etc.)
#define CAN_BUS_NAME INTERNAL_CAN
// This defines the client ID for this board on that bus
// These are defined in the titan_canmore/.../client_ids.h header file (and implicity included by titan_boards.cmake)
// Ensure that the bus that the client id below belongs to matches the bus selected above
#define CAN_BUS_CLIENT_ID CANMORE_CLIENT_ID_POWER_BOARD

// SHT Sensor
#define SHT_SDA_PIN 4
#define SHT_SCL_PIN 5

// RFID
#define RFID_MOSI_PIN 7
#define RFID_MISO_PIN 8
#define RFID_CS_PIN 9
#define RFID_CLK_PIN 10

// Kill
#define PHYS_KILLSWITCH_PIN 23

// IMU
#define IMU_I2C 0
#define IMU_SDA_PIN 24
#define IMU_SCL_PIN 25
#define IMU_RESET_PIN 11
#define IMU_INT_PIN 12

// Mux
#define MP_S0_PIN 26
#define MP_S1_PIN 27
#define MP_S2_PIN 28
#define MP_DATA_PIN 29

// LIDAR
#define LIDAR_TX_PIN 0
#define LIDAR_RX_PIN 1
#define LIDAR_ENABLE_PIN 2

#ifndef PICO_DEFAULT_I2C
#define PICO_DEFAULT_I2C IMU_I2C
#endif
#ifndef PICO_DEFAULT_I2C_SDA_PIN
#define PICO_DEFAULT_I2C_SDA_PIN IMU_SDA_PIN
#endif
#ifndef PICO_DEFAULT_I2C_SCL_PIN
#define PICO_DEFAULT_I2C_SCL_PIN IMU_SCL_PIN
#endif

#endif
