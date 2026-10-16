#include "sharedSail.h"
#include <stdio.h>

/////////////////////////////////////////////////////////////////////////////
// Main Sail
char *TelemetryMainSail_toString(TelemetryMainSail_t *telemetry, char *str) {
    snprintf(str, TELEMETRY_MAIN_SAIL_STR_SIZE, "ang_forward=%d [ %u | %u ]",
             telemetry->ang_forward, telemetry->ang_starboard, telemetry->ang_port);
    return str;
}



/////////////////////////////////////////////////////////////////////////////
// Tail Sail
char *TelemetryTailSail_toString(TelemetryTailSail_t *telemetry, char *str) {
    snprintf(str, TELEMETRY_TAIL_SAIL_STR_SIZE, "ang_forward=%d, ang_rearward=%d",
             telemetry->ang_forward, telemetry->ang_rearward);
    return str;
}