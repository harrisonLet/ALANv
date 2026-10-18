/////////////////////////////////////////////////////////////////////////////////////////
//
/////////////////////////////////////////////////////////////////////////////////////////

#include "projdefs.h"
#include "system.h"

#include "telemetry.h"

#include "queue.h"

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
        
        #ifdef CM7_TELEMETRY_DEBUG
        #ifdef CM7_SENSOR_BATTERY_ENABLE
        {
            TelemetryBattery_t telemetry;
            telemetry_getBattery(&telemetry);
            char telemetryStr[TELEMETRY_BATTERY_STR_SIZE];
            telemetry_printf("%s", TelemetryBattery_toString(&telemetry, telemetryStr));
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