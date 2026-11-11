#include "processLSP.h"
#include "pidSail.h"
#include "telemetry.h"
#include "arbiterTail.h"

#include <stdlib.h>

TaskHandle_t processLSP_task;

static int16_t aoa_zone(int16_t wind_abs);
static int16_t sail_target(int16_t wind);

HardwareInit_e processLSP_hardwareInit(void)
{
    processLSP_printf("processLSP_hardwareInit start");
    processLSP_printf("processLSP_hardwareInit complete");
    return HARDWARE_INIT_SUCCESS;
}

RtosInit_e processLSP_rtosInit(void)
{
    processLSP_printf("processLSP_rtosInit start");
    processLSP_printf("processLSP_rtosInit complete");
    return RTOS_INIT_SUCCESS;
}

// the handler picks the main sail angle based on the wind direction and the sailing zone
void processLSP_handler(void *argument){
    processLSP_printf("processLSP_handler start");
    TickType_t lastWakeup = xTaskGetTickCount();
    bool have_target = false;
    int16_t last_target = 0;
    uint32_t loop_count = 0;

    // loop that runs at 2 Hz, reads the wind and picks a main sail angle
    for (;;){
        if (!xTaskDelayUntil(&lastWakeup, configTICK_RATE_HZ / PROCESS_LSP_FREQUENCY))
        {
            continue;
        }
        // Only plan the sail in AUTO; in SEMI the ground station sets the target
        if (arbiterTail_getMode() != ARBITER_MODE_AUTO) {
            have_target = false;
            continue;
        }

        // No wind reading yet, so leave pidSail on its last target
        TelemetryWind_t wind;
        if (telemetry_getWind(&wind) != TELEMETRY_OK)
        {
            continue;
        }

        int16_t target = sail_target(wind.ang_wind);

        // Only send a new target when it moved more than the deadband, or when switching back to auto_mode
        if (!have_target || abs(ANG_WRAP_180(target - last_target)) > PROCESS_LSP_DEADBAND) {
            pidSail_setTarget(target);
            last_target = target;
            have_target = true;
        }

        // print everything once a second
        if (++loop_count % PROCESS_LSP_FREQUENCY == 0) {
            processLSP_printf("wind %d sail target %d", wind.ang_wind, last_target);
        }
    }
}

// AoA for how far the wind isoff the bow and 
static int16_t aoa_zone(int16_t off_bow)
{
    if (off_bow < SAILZONE_CLOSEHAULED_MIN) { return PROCESS_LSP_AOA_IRONS; }
    if (off_bow < SAILZONE_CLOSEREACH__MIN) { return PROCESS_LSP_AOA_CLOSEHAULED; }
    if (off_bow < SAILZONE_BEAMREACH___MIN) { return PROCESS_LSP_AOA_CLOSEREACH; }
    if (off_bow < SAILZONE_BROADREACH__MIN) { return PROCESS_LSP_AOA_BEAMREACH; }
    if (off_bow < SAILZONE_TRAININGRUN_MIN) { return PROCESS_LSP_AOA_BROADREACH; }
    if (off_bow < SAILZONE_RUNNING_____MIN) { return PROCESS_LSP_AOA_TRAININGRUN; }
    return PROCESS_LSP_AOA_RUNNING;
}

// Point the sail at the wind, then move it back toward the bow by the zone's AoA
static int16_t sail_target(int16_t wind)
{
int16_t aoa = aoa_zone(abs(wind));
bool starboard_wind = (wind >= 0);

if (starboard_wind) {
    return ANG_WRAP_180(wind - aoa); // smaller angle isc loser to the bow
}
return ANG_WRAP_180(wind + aoa);    // wind from port side so the AoA moves sail to the bow
}