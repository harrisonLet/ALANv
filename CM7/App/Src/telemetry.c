/////////////////////////////////////////////////////////////////////////////////////////
//
/////////////////////////////////////////////////////////////////////////////////////////

#include "projdefs.h"
#include "system.h"

#include "sensorBattery.h"
#include "sensorEncoder.h"
#include "sensorGPS.h"
#include "sensorIMU.h"
#include "sensorWind.h"
#include "telemetry.h"

#include "queue.h"
#include "ipc.h"
#include "loraTelemetry.h"

/////////////////////////////////////////////////////////////////////////////////////////

/////////////////////////////////////////////////////////////////////////////////////////

TaskHandle_t telemetry_task;

/////////////////////////////////////////////////////////////////////////////////////////

static QueueHandle_t TelemetryBattery_queue;
static QueueHandle_t TelemetryBoatRelativeOrientation_queue;
static QueueHandle_t TelemetryBoatGlobalOrientation_queue;
static QueueHandle_t TelemetryRudder_queue;
static QueueHandle_t TelemetryMainSail_queue;
static QueueHandle_t TelemetryTailSail_queue;
static QueueHandle_t TelemetryWind_queue;

static QueueHandle_t telemetry_queueCreate(char* name, size_t itemSize);

static bool telemetry_pack(LoRaTelemetry_t *telemetry, TelemetryBattery_t *telemetry_battery, TelemetryBoatRelativeOrientation_t *telemetry_boatRelative, TelemetryBoatGlobalOrientation_t *telemetry_boatGlobal, TelemetryRudder_t *telemetry_rudder, TelemetryMainSail_t *telemetry_mainSail, TelemetryTailSail_t *telemetry_tailSail, TelemetryWind_t *telemetry_wind);

/////////////////////////////////////////////////////////////////////////////////////////

HardwareInit_e telemetry_hardwareInit(void)
{
    telemetry_printf("telemetry_hardwareInit start");
    telemetry_printf("telemetry_hardwareInit complete");
    return HARDWARE_INIT_SUCCESS;
}



RtosInit_e telemetry_rtosInit(void) {
    telemetry_printf("telemetry_rtosInit start");

    TelemetryBattery_queue = telemetry_queueCreate("TelemetryBattery_queue", sizeof(TelemetryBattery_t));
    TelemetryBoatRelativeOrientation_queue = telemetry_queueCreate("TelemetryBoatRelativeOrientation_queue", sizeof(TelemetryBoatRelativeOrientation_t));
    TelemetryBoatGlobalOrientation_queue = telemetry_queueCreate("TelemetryBoatGlobalOrientation_queue", sizeof(TelemetryBoatGlobalOrientation_t));
    TelemetryRudder_queue = telemetry_queueCreate("TelemetryRudder_queue", sizeof(TelemetryRudder_t));
    TelemetryMainSail_queue = telemetry_queueCreate("TelemetryMainSail_queue", sizeof(TelemetryMainSail_t));
    TelemetryTailSail_queue = telemetry_queueCreate("TelemetryTailSail_queue", sizeof(TelemetryTailSail_t));
    TelemetryWind_queue = telemetry_queueCreate("TelemetryWind_queue", sizeof(TelemetryWind_t));

    telemetry_printf("telemetry_rtosInit complete");
    return RTOS_INIT_SUCCESS;
}
static QueueHandle_t telemetry_queueCreate(char* name, size_t itemSize) {
    QueueHandle_t queue = xQueueCreate(1, itemSize);
    #ifdef CM7_TELEMETRY_DEBUG
    if (queue == NULL) {
        telemetry_printf("ERROR (%s): Failed to create telemetry queue, free heap=%u", name, (unsigned)xPortGetFreeHeapSize());
    } else {
        telemetry_printf("Telemetry queue (%s) created, free heap=%u", name, (unsigned)xPortGetFreeHeapSize());
    }
    #endif
    return queue;
}



void telemetry_handler(void *argument) 
{
    telemetry_printf("Starting telemetry handler.");
    TickType_t lastWake = xTaskGetTickCount();
    for (;;)
    {
        vTaskDelayUntil(&lastWake, 10000);

        LoRaTelemetry_t telemetry = {0};

        TelemetryBattery_t telemetryBattery;
        TelemetryBoatRelativeOrientation_t telemetryRelative;
            
        
        #ifdef CM7_TELEMETRY_DEBUG
        #ifdef CM7_SENSOR_BATTERY_ENABLE
        {
            char telemetryStr[TELEMETRY_BATTERY_STR_SIZE];
            telemetry_printf("%s", TelemetryBattery_toString(&telemetry, telemetryStr));
        }
        #endif
        #if defined(CM7_FUSION_ORIENTATION_ENABLE) && defined(CM7_SENSOR_IMU_ENABLE) && defined(CM7_SENSOR_GPS_ENABLE)
        {
            char telemetryStr[TELEMETRY_BOAT_RELATIVE_ORIENTATION_STR_SIZE];
            telemetry_printf("%s", telemetryBoatRelativeOrientation_toString(&telemetryRelative, telemetryStr));

            TelemetryBoatGlobalOrientation_t telemetryGlobal;
            telemetry_getBoatGlobalOrientation(&telemetryGlobal);
            char telemetryStrGlobal[TELEMETRY_BOAT_GLOBAL_ORIENTATION_STR_SIZE];
            telemetry_printf("%s", telemetryBoatGlobalOrientation_toString(&telemetryGlobal, telemetryStrGlobal));
        }
        #endif
        #ifdef CM7_SENSOR_ENCODER_ENABLE
        {
            TelemetryMainSail_t telemetry;
            telemetry_getMainSail(&telemetry);
            char telemetryStr[TELEMETRY_MAIN_SAIL_STR_SIZE];
            telemetry_printf("%s", TelemetryMainSail_toString(&telemetry, telemetryStr));
        }
        #endif
        #ifdef CM7_SENSOR_WIND_ENABLE
        {
            TelemetryWind_t telemetry;
            telemetry_getWind(&telemetry);
            char telemetryStr[TELEMETRY_WIND_STR_SIZE];
            telemetry_printf("%s", TelemetryWind_toString(&telemetry, telemetryStr));
        }
        #endif
        #endif
    }
}


static bool telemetry_pack(
    LoRaTelemetry_t *telemetry, 
    TelemetryBattery_t *telemetry_battery, 
    TelemetryBoatRelativeOrientation_t *telemetry_boatRelative, 
    TelemetryBoatGlobalOrientation_t *telemetry_boatGlobal, 
    TelemetryRudder_t *telemetry_rudder, 
    TelemetryMainSail_t *telemetry_mainSail, 
    TelemetryTailSail_t *telemetry_tailSail, 
    TelemetryWind_t *telemetry_wind)
{
    bool battery_ok    = telemetry_getBattery(telemetry_battery) == TELEMETRY_OK;
    telemetry_getBoatRelativeOrientation(telemetry_boatRelative); // LoRaTelemetry_t has no linear velocity/acceleration fields
    bool boatGlobal_ok = telemetry_getBoatGlobalOrientation(telemetry_boatGlobal) == TELEMETRY_OK;
    bool rudder_ok     = telemetry_getRudder(telemetry_rudder) == TELEMETRY_OK;
    bool mainSail_ok   = telemetry_getMainSail(telemetry_mainSail) == TELEMETRY_OK;
    bool tailSail_ok   = telemetry_getTailSail(telemetry_tailSail) == TELEMETRY_OK;
    bool wind_ok       = telemetry_getWind(telemetry_wind) == TELEMETRY_OK;

    // A failed get leaves its struct untouched (nothing published yet), so only copy the ones that succeeded
    if (battery_ok) {
        telemetry->battery_voltage = telemetry_battery->battery_voltage;
        for (uint16_t i = 0; i < BATTERY_CELL_COUNT; i++) { telemetry->cell_voltage[i] = telemetry_battery->cell_voltage[i]; }
    }
    if (mainSail_ok) {
        telemetry->sail_ang_forward = telemetry_mainSail->ang_forward;
    }
    if (wind_ok) {
        telemetry->ang_wind = telemetry_wind->ang_wind;
    }
    if (boatGlobal_ok) {
        telemetry->latitude  = telemetry_boatGlobal->latitude;
        telemetry->longitude = telemetry_boatGlobal->longitude;
        telemetry->heading   = telemetry_boatGlobal->angP_heading;
        telemetry->roll      = telemetry_boatGlobal->angP_roll;
        telemetry->pitch     = telemetry_boatGlobal->angP_pitch;
    }
    if (rudder_ok) {
        telemetry->rudder_servo_angle = telemetry_rudder->ang_forward;
    }
    if (tailSail_ok) {
        telemetry->sail_servo_angle = telemetry_tailSail->ang_forward; // The sail servo drives the tail sail
    }

    MetadataBattery_t metadata_battery;
    bool metadata_battery_okay = sensorBattery_getMetadata(&metadata_battery);

    // true only if every source above had data
    return battery_ok && boatGlobal_ok && rudder_ok && mainSail_ok && tailSail_ok && wind_ok;
}

/////////////////////////////////////////////////////////////////////////////////////////

TelemetryResult_e telemetry_getBattery(TelemetryBattery_t *telemetry) { return (xQueuePeek(TelemetryBattery_queue, telemetry, 0) == pdTRUE) ?   TELEMETRY_OK : TELEMETRY_ERROR; }
TelemetryResult_e telemetry_setBattery(TelemetryBattery_t *telemetry) { return (xQueueOverwrite(TelemetryBattery_queue, telemetry) == pdTRUE) ? TELEMETRY_OK : TELEMETRY_ERROR; }

TelemetryResult_e telemetry_getBoatRelativeOrientation(TelemetryBoatRelativeOrientation_t *telemetry) { return (xQueuePeek(TelemetryBoatRelativeOrientation_queue, telemetry, 0) == pdTRUE) ?   TELEMETRY_OK : TELEMETRY_ERROR; }
TelemetryResult_e telemetry_setBoatRelativeOrientation(TelemetryBoatRelativeOrientation_t *telemetry) { return (xQueueOverwrite(TelemetryBoatRelativeOrientation_queue, telemetry) == pdTRUE) ? TELEMETRY_OK : TELEMETRY_ERROR; }

TelemetryResult_e telemetry_getBoatGlobalOrientation(TelemetryBoatGlobalOrientation_t *telemetry) { return (xQueuePeek(TelemetryBoatGlobalOrientation_queue, telemetry, 0) == pdTRUE) ?   TELEMETRY_OK : TELEMETRY_ERROR; }
TelemetryResult_e telemetry_setBoatGlobalOrientation(TelemetryBoatGlobalOrientation_t *telemetry) { return (xQueueOverwrite(TelemetryBoatGlobalOrientation_queue, telemetry) == pdTRUE) ? TELEMETRY_OK : TELEMETRY_ERROR; }

TelemetryResult_e telemetry_getRudder(TelemetryRudder_t *telemetry) { return (xQueuePeek(TelemetryRudder_queue, telemetry, 0) == pdTRUE) ?   TELEMETRY_OK : TELEMETRY_ERROR; }
TelemetryResult_e telemetry_setRudder(TelemetryRudder_t *telemetry) { return (xQueueOverwrite(TelemetryRudder_queue, telemetry) == pdTRUE) ? TELEMETRY_OK : TELEMETRY_ERROR; }

TelemetryResult_e telemetry_getMainSail(TelemetryMainSail_t *telemetry) { return (xQueuePeek(TelemetryMainSail_queue, telemetry, 0) == pdTRUE) ?   TELEMETRY_OK : TELEMETRY_ERROR; }
TelemetryResult_e telemetry_setMainSail(TelemetryMainSail_t *telemetry) { return (xQueueOverwrite(TelemetryMainSail_queue, telemetry) == pdTRUE) ? TELEMETRY_OK : TELEMETRY_ERROR; }

TelemetryResult_e telemetry_getTailSail(TelemetryTailSail_t *telemetry) { return (xQueuePeek(TelemetryTailSail_queue, telemetry, 0) == pdTRUE) ?   TELEMETRY_OK : TELEMETRY_ERROR; }
TelemetryResult_e telemetry_setTailSail(TelemetryTailSail_t *telemetry) { return (xQueueOverwrite(TelemetryTailSail_queue, telemetry) == pdTRUE) ? TELEMETRY_OK : TELEMETRY_ERROR; }

TelemetryResult_e telemetry_getWind(TelemetryWind_t *telemetry) { return (xQueuePeek(TelemetryWind_queue, telemetry, 0) == pdTRUE) ?   TELEMETRY_OK : TELEMETRY_ERROR; }
TelemetryResult_e telemetry_setWind(TelemetryWind_t *telemetry) { return (xQueueOverwrite(TelemetryWind_queue, telemetry) == pdTRUE) ? TELEMETRY_OK : TELEMETRY_ERROR; }