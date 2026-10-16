#ifndef TELEMETRY_H
#define TELEMETRY_H

#include "system.h"
#include "sharedBattery.h"
#include "sharedBoat.h"
#include "sharedRudder.h"
#include "sharedSail.h"
#include "sharedWind.h"

#define TELEMETRY_TASK "[TELEMTRY]"
#define TELEMETRY_TASK_PRIORITY osPriorityLow

#ifdef CM7_TELEMETRY_DEBUG
#define TELEMETRY_TASK_HEAP 500
#define telemetry_printf(fmt, ...) DEBUG_PRINTF(TELEMETRY_TASK, fmt, ##__VA_ARGS__)
#else
#define TELEMETRY_TASK_HEAP 250
#define telemetry_printf(fmt, ...) 
#endif

extern TaskHandle_t telemetry_task;

HardwareInit_e telemetry_hardwareInit(void);
RtosInit_e telemetry_rtosInit(void);
void telemetry_handler(void *argument);

typedef enum {
    TELEMETRY_OK,
    TELEMETRY_ERROR
} TelemetryResult_e;

TelemetryResult_e telemetry_getBattery(TelemetryBattery_t *telemetry);
TelemetryResult_e telemetry_setBattery(TelemetryBattery_t *telemetry);

TelemetryResult_e telemetry_getBoatRelativeOrientation(TelemetryBoatRelativeOrientation_t *telemetry);
TelemetryResult_e telemetry_setBoatRelativeOrientation(TelemetryBoatRelativeOrientation_t *telemetry);

TelemetryResult_e telemetry_getBoatGlobalOrientation(TelemetryBoatGlobalOrientation_t *telemetry);
TelemetryResult_e telemetry_setBoatGlobalOrientation(TelemetryBoatGlobalOrientation_t *telemetry);

TelemetryResult_e telemetry_getRudder(TelemetryRudder_t *telemetry);
TelemetryResult_e telemetry_setRudder(TelemetryRudder_t *telemetry);

TelemetryResult_e telemetry_getMainSail(TelemetryMainSail_t *telemetry);
TelemetryResult_e telemetry_setMainSail(TelemetryMainSail_t *telemetry);

TelemetryResult_e telemetry_getTailSail(TelemetryTailSail_t *telemetry);
TelemetryResult_e telemetry_setTailSail(TelemetryTailSail_t *telemetry);

TelemetryResult_e telemetry_getWind(TelemetryWind_t *telemetry);
TelemetryResult_e telemetry_setWind(TelemetryWind_t *telemetry);

#endif // TELEMETRY_H