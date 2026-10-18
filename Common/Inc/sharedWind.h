#pragma once

#include <stdint.h>

/////////////////////////////////////////////////////////////////////////////
// Wind
#define WIND_OP_ANGLE_MIN -180
#define WIND_OP_ANGLE_MAX 180

#define SAILZONE_CLOSEHAULED_MIN 30
#define SAILZONE_CLOSEREACH__MIN 50
#define SAILZONE_BEAMREACH___MIN 75
#define SAILZONE_BROADREACH__MIN 105
#define SAILZONE_TRAININGRUN_MIN 130
#define SAILZONE_RUNNING_____MIN 150

#define SAILZONE_R_CLOSEHAULED_MAX (180-SAILZONE_CLOSEHAULED_MIN)
#define SAILZONE_R_CLOSEREACH__MAX (180-SAILZONE_CLOSEREACH__MIN)
#define SAILZONE_R_BEAMREACH___MAX (180-SAILZONE_BEAMREACH___MIN)
#define SAILZONE_R_BROADREACH__MAX (180-SAILZONE_BROADREACH__MIN)
#define SAILZONE_R_TRAININGRUN_MAX (180-SAILZONE_TRAININGRUN_MIN)
#define SAILZONE_R_RUNNING_____MAX (180-SAILZONE_RUNNING_____MIN)

/**
 * Represents the current wind direction relative to the vessel.
 *
 * SENSOR: Wind Vane
 */
typedef struct {
    /**
     * Angle of the measured wind relative to the center line of the vessel.
     *
     * Measured in degrees this value represents the angle of the center line of
     * the vessel from stern to bow relative to a line going directly up wind. 
     * Positive values represent the vessel being turned port out of the wind. Negative
     * values represent the vessel being turned starboard out of the wind. To
     * put it another way, positive values mean the wind is coming from the starboard
     * side, and negative values mean the wind is coming from the port side.
     *
     * - Operational Range: -180 to 180 degrees
     * - Precision: 1 degree
     */
    int16_t ang_wind;
    /**
     * Angle of the measured wind relative to the vessel, but with the frame reversed.
     *
     * Measured in degrees this value represents the angle at which the wind is 
     * attacking the boat where the running position represents 0. Positive values 
     * represent the wind attacking the vessel from the starboard side. Negative 
     * values represent the wind attacking the vessel from the port side.
     *
     * - Operational Range: -180 to 180 degrees
     * - Precision: 1 degree
     */
    int16_t ang_wind_r;
} TelemetryWind_t;

#define TELEMETRY_WIND_STR_SIZE 64
char *TelemetryWind_toString(TelemetryWind_t *telemetry, char *buffer);