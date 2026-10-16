#include "sharedBattery.h"
#include <stdio.h>

/////////////////////////////////////////////////////////////////////////////
// Battery
char *TelemetryBattery_toString(const TelemetryBattery_t *telemetry, char buf[TELEMETRY_BATTERY_STR_SIZE])
{
    snprintf(buf, TELEMETRY_BATTERY_STR_SIZE, "TelemetryBattery: %05umV [ %04u | %04u | %04u | %04u ]mV",
             (unsigned)telemetry->battery_voltage,
             (unsigned)telemetry->cell_voltage[0], (unsigned)telemetry->cell_voltage[1],
             (unsigned)telemetry->cell_voltage[2], (unsigned)telemetry->cell_voltage[3]);
    return buf;
}
