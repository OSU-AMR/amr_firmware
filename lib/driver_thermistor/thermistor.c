#include "driver/thermistor.h"

#include <math.h>

#define THERM_BOARD_SERIES_RESISTANCE 10000.0f
#define THERM_BOARD_VREF 3.3f
#define THERM_BOARD_R_25 22000.0f
#define THERM_BOARD_B_25_85 3730.0f
#define THERM_BOARD_NOMINAL_TEMP 298.15f

#define THERM_CHASSIS_SERIES_RESISTANCE 10000.0f
#define THERM_CHASSIS_VREF 3.3f
#define THERM_CHASSIS_R_25 10000.0f
#define THERM_CHASSIS_B_25_85 3984.0f
#define THERM_CHASSIS_NOMINAL_TEMP 298.15f

float thermistor_get_c(float voltage, thermistor_profile profile) {
    float ser_res = 0.0f, vref = 0.0f, r25 = 0.0f, b25_85 = 0.0f, nominal = 0.0f;

    switch (profile) {
    case THERMISTOR_PROFILE_BOARD:
        ser_res = THERM_BOARD_SERIES_RESISTANCE;
        vref = THERM_BOARD_VREF;
        r25 = THERM_BOARD_R_25;
        b25_85 = THERM_BOARD_B_25_85;
        nominal = THERM_BOARD_NOMINAL_TEMP;
        break;
    case THERMISTOR_PROFILE_CHASSIS:
        ser_res = THERM_CHASSIS_SERIES_RESISTANCE;
        vref = THERM_CHASSIS_VREF;
        r25 = THERM_CHASSIS_R_25;
        b25_85 = THERM_CHASSIS_B_25_85;
        nominal = THERM_CHASSIS_NOMINAL_TEMP;
        break;
    }

    float resistance = ser_res * (voltage) / (vref - voltage);

    return 1.0f / (log(resistance / r25) / b25_85 + 1.0f / nominal) - 273.15;
}
