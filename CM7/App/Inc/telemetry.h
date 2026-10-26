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
#define TELEMETRY_FREQUENCY 1u // Hz
_Static_assert(TELEMETRY_FREQUENCY > 0u && TELEMETRY_FREQUENCY <= SYSTEM_TICK_RATE_HZ, "TELEMETRY_FREQUENCY must be between 1Hz and the tick rate");
#if (SYSTEM_TICK_RATE_HZ % TELEMETRY_FREQUENCY) != 0u
#warning "TELEMETRY_FREQUENCY does not divide the tick rate, the period will be rounded down"
#endif

#ifdef CM7_TELEMETRY_DEBUG
#define TELEMETRY_TASK_HEAP 400
#define telemetry_printf(fmt, ...) DEBUG_PRINTF(TELEMETRY_TASK, fmt, ##__VA_ARGS__)
#else
#define TELEMETRY_TASK_HEAP 260 // ~100B of extra
#define telemetry_printf(fmt, ...) ((void)0)
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