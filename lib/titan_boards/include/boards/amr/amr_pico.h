#ifndef BOARDS__AMR__PICO_H_
#define BOARDS__AMR__PICO_H_

#include "boards/mk2/blocks/rp2040_can_block.h"

#ifdef UWRT_BOARD_DEFINED
#error Multiple board types defined
#endif
#define UWRT_BOARD_DEFINED

// This defines which CAN bus this board is connected into
// The CAN bus is defined in the corresponding robot definition files (rate, enable FD, etc.)
// #define CAN_BUS_NAME INTERNAL_CAN
// This defines the client ID for this board on that bus
// These are defined in the titan_canmore/.../client_ids.h header file (and implicity included by titan_boards.cmake)
// Ensure that the bus that the client id below belongs to matches the bus selected above
// #define CAN_BUS_CLIENT_ID CANMORE_CLIENT_ID_LED_BOARD

#define LEFT_ESC_PIN 20
#define RIGHT_ESC_PIN 11

#endif
