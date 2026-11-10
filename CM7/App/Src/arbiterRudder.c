#include "arbiterRudder.h"
#include "servoRudder.h"
#include "telemetry.h"

#include "queue.h"

typedef struct 
{
    ArbiterRudderSource_e source;
    int16_t turn;
    uint32_t timestamp;
} ArbiterRudderRequest_t;

TaskHandle_t arbiterRudder_task;

static QueueHandle_t request_queue [ARBITER_RUDDER_SOURCE_COUNT];
static volatile uint8_t current_mode;
static volatile int16_t current_turn;
static int16_t clamped_turn(int16_t requested_turn, int16_t low, int16_t high);
static bool get_request(ArbiterRudderSource_e source, TickType_t max_wait, int16_t *turn);

HardwareInit_e arbiterRudder_hardwareInit(void) 
{
    arbiterRudder_printf("arbiterRudder_hardwareInit start");
    arbiterRudder_printf("arbiterRudder_hardwareInit complete");
    return HARDWARE_INIT_SUCCESS;
}

RtosInit_e arbiterRudder_rtosInit(void) 
{
    arbiterRudder_printf("arbiterRudder_rtosInit start");
    for (int i = 0; i < ARBITER_RUDDER_SOURCE_COUNT; i++) {
        request_queue[i] = xQueueCreate(1, sizeof(ArbiterRudderRequest_t));
        if (request_queue[i] == NULL) { return RTOS_INIT_FAIL; }
    }
    current_mode = ARBITER_MODE_AUTO;
    current_turn = 0;      // leave the rudder centered until first request
    arbiterRudder_printf("arbiterRudder_rtosInit complete");
    return RTOS_INIT_SUCCESS;
}

void arbiterRudder_handler(void *argument) 
{
arbiterRudder_printf("arbiterRudder_handler start");
TickType_t lastWakeup = xTaskGetTickCount();
for(;;){
    if (!xTaskDelayUntil(&lastWakeup, configTICK_RATE_HZ / ARBITER_RUDDER_FREQUENCY)) 
    {
        continue;
    }

        // center unless the active source has a request
        int16_t target = 0;
        if (current_mode == ARBITER_MODE_MANUAL) {
            get_request(ARBITER_RUDDER_SOURCE_MANUAL, 0, &target);
        } else if (current_mode == ARBITER_MODE_AUTO || current_mode == ARBITER_MODE_SEMI) {
            get_request(ARBITER_RUDDER_SOURCE_PID, pdMS_TO_TICKS(ARBITER_RUDDER_STALE), &target);
        }

        // Clamp first so the step never goes past the limit
        target = clamped_turn(target, -ARBITER_RUDDER_LIMIT, ARBITER_RUDDER_LIMIT);
        int16_t step = clamped_turn(target - current_turn, -ARBITER_RUDDER_MAX_STEP, ARBITER_RUDDER_MAX_STEP);
        if (step == 0) { continue; }
        current_turn += step;

        // invert the rudder angle so it is more intuitive 
        TelemetryRudder_t rudder = {
            .ang_forward = -current_turn,
            .ang_turn = current_turn
        };
        servoRudder_setAngle(ARBITER_RUDDER_SIGN * rudder.ang_forward);
        telemetry_setRudder(&rudder);
    }
}

void arbiterRudder_request(ArbiterRudderSource_e source, int16_t turn)
{
    // The queue is NULL when this arbiter is disabled in shared_configuration.h
    if (source >= ARBITER_RUDDER_SOURCE_COUNT || request_queue[source] == NULL) { return; }
    ArbiterRudderRequest_t request = { .source = source, .turn = turn, .timestamp =
xTaskGetTickCount() };
    xQueueOverwrite(request_queue[source], &request);
}

void arbiterRudder_setMode(ArbiterMode_e mode)
{
    current_mode = mode;
}

static int16_t clamped_turn(int16_t requested_turn, int16_t low, int16_t high)
{
    if (requested_turn < low)  { return low; }
    if (requested_turn > high) { return high; }
    return requested_turn;
}

// max_wait of 0 means the request never goes stale
static bool get_request(ArbiterRudderSource_e source, TickType_t max_wait, int16_t *turn)
{
    ArbiterRudderRequest_t request;
    if (xQueuePeek(request_queue[source], &request, 0) != pdTRUE) { return false; }
    if (max_wait != 0 && (xTaskGetTickCount() - request.timestamp) > max_wait) { return
false; }
    *turn = request.turn;
    return true;
}