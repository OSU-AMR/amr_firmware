#ifndef BOARDS__ESC_BOARD_H_
#define BOARDS__ESC_BOARD_H_

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
#define CAN_BUS_CLIENT_ID CANMORE_CLIENT_ID_ESC_BOARD

#define ESC0_1_PIN 0
#define ESC0_2_PIN 1
#define ESC0_3_PIN 2

#define ENC0_A_PIN 3
#define ENC0_B_PIN 4

#define ESC1_1_PIN 8
#define ESC1_2_PIN 9
#define ESC1_3_PIN 10

#define ENC1_A_PIN 11
#define ENC1_B_PIN 12

#define MOTOR_ENABLE_PIN 7

#define IR0_PIN 27
#define IR1_PIN 28
#define IR2_PIN 29

#define MP_S0_PIN 23
#define MP_S1_PIN 24
#define MP_S2_PIN 25
#define MP_DATA_PIN 26

#define MOT0_OVERTEMP_PIN 5
#define MOT1_OVERTEMP_PIN 6

#define MOT0_THERM_MUX_NUM 6
#define MOT1_THERM_MUX_NUM 3
#define ESC0_THERM_MUX_NUM 7
#define ESC1_THERM_MUX_NUM 4

#endif
