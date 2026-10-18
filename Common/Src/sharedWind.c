#include "sharedWind.h"
#include <stdio.h>

/////////////////////////////////////////////////////////////////////////////
// Wind
char *TelemetryWind_toString(TelemetryWind_t *telemetry, char *buffer) {
    snprintf(buffer, TELEMETRY_WIND_STR_SIZE, "ang_wind: %d, ang_wind_r: %d",
             telemetry->ang_wind, telemetry->ang_wind_r);
    return buffer;
}