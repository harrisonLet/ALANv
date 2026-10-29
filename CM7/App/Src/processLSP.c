#include "processLSP.h"
#include "pidSail.h"
#include "telemetry.h"

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


// AoA for how far the wind is off the bow and
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
        return ANG_WRAP_180(wind - aoa); // smaller angle is closer to the bow
    }
    return ANG_WRAP_180(wind + aoa);    // wind from port side so the AoA moves sail to the bow
  }