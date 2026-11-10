#include "arbiterTail.h"
#include "servoSail.h"
#include "telemetry.h"

#include "queue.h"

typedef struct
{
    ArbiterTailSource_e source;
    int16_t angle;
    uint32_t timestamp;
} ArbiterTailRequest_t;

TaskHandle_t arbiterTail_task;

static QueueHandle_t request_queue[ARBITER_TAIL_SOURCE_COUNT];
static volatile uint8_t current_mode;
static int16_t current_angle;
static int16_t clamped_angle(int16_t requested_angle, int16_t low, int16_t high);
static bool get_request(ArbiterTailSource_e source, TickType_t max_wait, int16_t *angle);

HardwareInit_e arbiterTail_hardwareInit(void)
{
    arbiterTail_printf("arbiterTail_hardwareInit start");
    arbiterTail_printf("arbiterTail_hardwareInit complete");
    return HARDWARE_INIT_SUCCESS;
}

RtosInit_e arbiterTail_rtosInit(void)
{
    arbiterTail_printf("arbiterTail_rtosInit start");
    for (int i = 0; i < ARBITER_TAIL_SOURCE_COUNT; i++) {
        request_queue[i] = xQueueCreate(1, sizeof(ArbiterTailRequest_t));
        if (request_queue[i] == NULL) { return RTOS_INIT_FAIL; }
    }
    current_mode = ARBITER_MODE_AUTO;
    current_angle = 0;
    arbiterTail_printf("arbiterTail_rtosInit complete");
    return RTOS_INIT_SUCCESS;
}

void arbiterTail_handler(void *argument)
{
    arbiterTail_printf("arbiterTail_handler start");
    TickType_t lastWakeup = xTaskGetTickCount();
    for (;;)
    {
        if (!xTaskDelayUntil(&lastWakeup, configTICK_RATE_HZ / ARBITER_TAIL_FREQUENCY)) {
            continue;
        }

        // go to the safe angle unless the active source has a request. Manual holds where it is until the first servo command
        int16_t target = ARBITER_TAIL_SAFE;
        if (current_mode == ARBITER_MODE_MANUAL) {
            target = current_angle;
            get_request(ARBITER_TAIL_SOURCE_MANUAL, 0, &target);
        } else if (current_mode == ARBITER_MODE_AUTO || current_mode == ARBITER_MODE_SEMI) {
            get_request(ARBITER_TAIL_SOURCE_PID, pdMS_TO_TICKS(ARBITER_TAIL_STALE), &target);
        }

        // Clamp first so the step never goes past the limit
        target = clamped_angle(target, -ARBITER_TAIL_LIMIT, ARBITER_TAIL_LIMIT);
        int16_t step = clamped_angle(target - current_angle, -ARBITER_TAIL_MAX_STEP, ARBITER_TAIL_MAX_STEP);
        if (step == 0) { continue; }
        current_angle += step;

        // ang_rearward is the same position measured from the other end of the tail
        TelemetryTailSail_t tail = {
            .ang_forward = current_angle,
            .ang_rearward = (current_angle > 0) ? current_angle - 180 : current_angle + 180
        };
        servoSail_setAngle(ARBITER_TAIL_SIGN * tail.ang_forward);
        telemetry_setTailSail(&tail);
    }
}

void arbiterTail_request(ArbiterTailSource_e source, int16_t angle)
{
    // The queue is NULL when this arbiter is disabled in shared_configuration.h
    if (source >= ARBITER_TAIL_SOURCE_COUNT || request_queue[source] == NULL) { return; }
    ArbiterTailRequest_t request = { .source = source, .angle = angle, .timestamp = xTaskGetTickCount() };
    xQueueOverwrite(request_queue[source], &request);
}

void arbiterTail_setMode(ArbiterMode_e mode)
{
    // Drop the last manual angle on the way into manual, so the tail holds instead of jumping to an old command
    if (mode == ARBITER_MODE_MANUAL && current_mode != ARBITER_MODE_MANUAL &&
        request_queue[ARBITER_TAIL_SOURCE_MANUAL] != NULL) {
        xQueueReset(request_queue[ARBITER_TAIL_SOURCE_MANUAL]);
    }
    current_mode = mode;
}

ArbiterMode_e arbiterTail_getMode(void)
{
    return current_mode;
}

static int16_t clamped_angle(int16_t requested_angle, int16_t low, int16_t high)
{
    if (requested_angle < low)
    {
        return low;
    }
    if (requested_angle > high)
    {
        return high;
    }
    return requested_angle;
}

// max_wait of 0 means the request never goes stale
static bool get_request(ArbiterTailSource_e source, TickType_t max_wait, int16_t *angle)
{
    ArbiterTailRequest_t request;
    if (xQueuePeek(request_queue[source], &request, 0) != pdTRUE) { return false; }
    if (max_wait != 0 && (xTaskGetTickCount() - request.timestamp) > max_wait) 
    {
        return false;
    }
    *angle = request.angle;
    return true;
}