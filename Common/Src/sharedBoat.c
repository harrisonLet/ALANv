#include "sharedBoat.h"
#include <stdio.h>

/////////////////////////////////////////////////////////////////////////////
// Boat Relative Orientation
char *telemetryBoatRelativeOrientation_toString(const TelemetryBoatRelativeOrientation_t *telemetry, char buffer[TELEMETRY_BOAT_RELATIVE_ORIENTATION_STR_SIZE])
{
    snprintf(buffer, TELEMETRY_BOAT_RELATIVE_ORIENTATION_STR_SIZE, "TelemetryBoatRelative: angV[ %05d | %05d | %05d ] linV[ %05d | %05d | %05d ], linA[ %05d | %05d | %05d ]",
             telemetry->angV_roll, telemetry->angV_pitch, telemetry->angV_yaw,
             telemetry->linV_heave, telemetry->linV_surge, telemetry->linV_sway,
             telemetry->linA_heave, telemetry->linA_surge, telemetry->linA_sway);
    return buffer;
}


/////////////////////////////////////////////////////////////////////////////
// Boat Global Orientation
char *telemetryBoatGlobalOrientation_toString(const TelemetryBoatGlobalOrientation_t *telemetry, char buffer[TELEMETRY_BOAT_GLOBAL_ORIENTATION_STR_SIZE])
{
    snprintf(buffer, TELEMETRY_BOAT_GLOBAL_ORIENTATION_STR_SIZE, "TelemetryBoatGlobal: angP[ %05d | %05d ] head[ %05u ] GPS[ %010.6f | %011.6f ]",
             telemetry->angP_roll, telemetry->angP_pitch, 
             telemetry->angP_heading,
             telemetry->latitude, telemetry->longitude);
    return buffer;
}