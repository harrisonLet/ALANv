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

/////////////////////////////////////////////////////////////////////////////////////////

void telemetry_init(void) {
    telemetry_printf("Initialize telemetry");

    TelemetryBattery_queue = xQueueCreate(1, sizeof(TelemetryBattery_t));
    TelemetryBoatRelativeOrientation_queue = xQueueCreate(1, sizeof(TelemetryBoatRelativeOrientation_t));
    TelemetryBoatGlobalOrientation_queue = xQueueCreate(1, sizeof(TelemetryBoatGlobalOrientation_t));
    TelemetryRudder_queue = xQueueCreate(1, sizeof(TelemetryRudder_t));
    TelemetryMainSail_queue = xQueueCreate(1, sizeof(TelemetryMainSail_t));
    TelemetryTailSail_queue = xQueueCreate(1, sizeof(TelemetryTailSail_t));
    TelemetryWind_queue = xQueueCreate(1, sizeof(TelemetryWind_t));

    telemetry_printf("Telemetry queues created");
}

void telemetry_handler(void *argument) 
{
    telemetry_printf("Starting telemetry handler.");
    TickType_t lastWake = xTaskGetTickCount();
    for (;;)
    {
        vTaskDelayUntil(&lastWake, pdMS_TO_TICKS(5000));
        
        #ifdef CM7_TELEMETRY_DEBUG
        #ifdef CM7_SENSOR_BATTERY_ENABLE
        {
            TelemetryBattery_t telemetry;
            telemetry_getBattery(&telemetry);
            char telemetryStr[TELEMETRY_BATTERY_STR_SIZE];
            live_printf("%s", TelemetryBattery_toString(&telemetry, telemetryStr));
        }
        #endif
        #ifdef CM7_SENSOR_ENCODER_ENABLE
        {
            TelemetryMainSail_t telemetry;
            telemetry_getMainSail(&telemetry);
            char telemetryStr[TELEMETRY_MAIN_SAIL_STR_SIZE];
            live_printf("%s", TelemetryMainSail_toString(&telemetry, telemetryStr));
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