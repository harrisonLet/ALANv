/////////////////////////////////////////////////////////////////////////////////////////
//
/////////////////////////////////////////////////////////////////////////////////////////

#include "projdefs.h"
#include "sharedBoat.h"
#include "sharedSensor.h"
#include "system.h"

#include "sensorBattery.h"
#include "sensorEncoder.h"
#include "sensorGPS.h"
#include "sensorIMU.h"
#include "sensorWind.h"
#include "telemetry.h"

#include <math.h>

#include "queue.h"
#include "ipc.h"
#include "loraTelemetry.h"
#include "commander.h"
#include "pidHeading.h"
#include "navigationTask.h"

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
        if (!xTaskDelayUntil(&lastWake, configTICK_RATE_HZ / TELEMETRY_FREQUENCY)) {
            continue;
        }

        LoRaTelemetry_t telemetry = {0};

        TelemetryBattery_t telemetry_battery;
        TelemetryBoatRelativeOrientation_t telemetry_boatRelative;
        TelemetryBoatGlobalOrientation_t telemetry_boatGlobal;
        TelemetryRudder_t telemetry_rudder;
        TelemetryMainSail_t telemetry_mainSail;
        TelemetryTailSail_t telemetry_tailSail;
        TelemetryWind_t telemetry_wind;

        bool pack_result = telemetry_pack(&telemetry, &telemetry_battery, &telemetry_boatRelative, &telemetry_boatGlobal, &telemetry_rudder, &telemetry_mainSail, &telemetry_tailSail, &telemetry_wind);
        if (!pack_result) { telemetry_printf("Telemetry pack failed"); }
        ipc_status_t ipc_status = ipc_telem_write(&telemetry);
        if (ipc_status == IPC_STATUS_OK) { telemetry_printf("IPC write success"); }
        else if (ipc_status == IPC_STATUS_BUSY) { telemetry_printf("IPC busy, this is acceptable, will retry in next round."); }
        else { telemetry_printf("IPC write failed for a weird reason. %02X", ipc_status); }

        #ifdef CM7_TELEMETRY_DEBUG
        #ifdef CM7_SENSOR_BATTERY_ENABLE
        {
            char telemetryStr[TELEMETRY_BATTERY_STR_SIZE];
            telemetry_printf("%s", TelemetryBattery_toString(&telemetry_battery, telemetryStr));
        }
        #endif
        #if defined(CM7_FUSION_ORIENTATION_ENABLE) && defined(CM7_SENSOR_IMU_ENABLE) && defined(CM7_SENSOR_GPS_ENABLE)
        {
            // char telemetryStr[TELEMETRY_BOAT_RELATIVE_ORIENTATION_STR_SIZE];
            // telemetry_printf("%s", telemetryBoatRelativeOrientation_toString(&telemetry_boatRelative, telemetryStr));

            char telemetryStrGlobal[TELEMETRY_BOAT_GLOBAL_ORIENTATION_STR_SIZE];
            telemetry_printf("%s", telemetryBoatGlobalOrientation_toString(&telemetry_boatGlobal, telemetryStrGlobal));
        }
        #endif
        #ifdef CM7_SENSOR_ENCODER_ENABLE
        {
            char telemetryStr[TELEMETRY_MAIN_SAIL_STR_SIZE];
            telemetry_printf("%s", TelemetryMainSail_toString(&telemetry_mainSail, telemetryStr));
        }
        #endif
        #ifdef CM7_SENSOR_WIND_ENABLE
        {
            char telemetryStr[TELEMETRY_WIND_STR_SIZE];
            telemetry_printf("%s", TelemetryWind_toString(&telemetry_wind, telemetryStr));
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
    MetadataBattery_t metadata_battery;
    if (sensorBattery_getMetadata(&metadata_battery))
    {
        if (metadata_battery.valid != 0 && metadata_battery.valid < 0x10) { telemetry->battery_valid |= LORA_TELEMETRY_METADATA_VALID; }
    }

    MetadataEncoder_t metadata_encoder;
    if (sensorEncoder_getMetadata(&metadata_encoder))
    {
        telemetry->sail_encoder_magnitude = metadata_encoder.magnitude;
        telemetry->sail_encoder_status = metadata_encoder.status;
        if (metadata_encoder.valid != 0 && metadata_encoder.valid < 0x10) { telemetry->encoder_valid |= LORA_TELEMETRY_METADATA_VALID; }
    }

    MetadataWind_t metadata_wind;
    if (sensorWind_getMetadata(&metadata_wind))
    {
        if (metadata_wind.valid != 0 && metadata_wind.valid < 0x10) { telemetry->wind_valid |= LORA_TELEMETRY_METADATA_VALID; }
    }

    // 0 would read as stopped and as north, so velocity starts unknown
    telemetry->sog = LORA_GPS_VEL_NONE;
    telemetry->cog = LORA_GPS_VEL_NONE;

    MetadataGPS_t metadata_gps;
    if (sensorGPS_getMetadata(&metadata_gps))
    {
        // RMC velocity has its own validity and timestamp, separate from GGA's. Signed age like navigationTask's recent()
        bool vel_fresh = (int32_t)(HAL_GetTick() - metadata_gps.vel_timestamp) <= (int32_t)SENSOR_GPS_STALE;
        if (metadata_gps.vel_valid && vel_fresh)
        {
            long sog = lroundf(metadata_gps.sog_mps * (float)LORA_GPS_SOG_SCALE);
            telemetry->sog = (uint16_t)((sog < 0) ? 0 : (sog >= (long)LORA_GPS_VEL_NONE) ? (long)LORA_GPS_VEL_NONE - 1 : sog);
            telemetry->cog = (metadata_gps.cog_deg < 360U) ? metadata_gps.cog_deg : LORA_GPS_VEL_NONE;
        }

        telemetry->altitude = (int16_t)lroundf(metadata_gps.altitude * LORA_GPS_ALT_SCALE);
        telemetry->sats_fix = LORA_GPS_SATS_FIX(metadata_gps.satellites, metadata_gps.fix_valid);
        telemetry->utc_hours = metadata_gps.hours;
        telemetry->utc_minutes = metadata_gps.minutes;
        telemetry->utc_seconds = metadata_gps.seconds;
        if (metadata_gps.valid != 0 && metadata_gps.valid < 0x10) { telemetry->gps_valid |= LORA_TELEMETRY_METADATA_VALID; }
    }

    MetadataIMU_t metadata_imu;
    if (sensorIMU_getMetadata(&metadata_imu))
    {
        telemetry->imu_cal = metadata_imu.cal[0];
        telemetry->imu_temp = metadata_imu.tmp[0];
        if (metadata_imu.valid != 0 && metadata_imu.valid < 0x10) 
        { 
            telemetry->imu_valid |= LORA_TELEMETRY_METADATA_VALID; 
            telemetry->orientation_valid |= LORA_TELEMETRY_METADATA_VALID; 
        }
    }

    if (telemetry_getBattery(telemetry_battery) == TELEMETRY_OK) 
    {
        telemetry->battery_voltage = telemetry_battery->battery_voltage;
        for (uint16_t i = 0; i < BATTERY_CELL_COUNT; i++) { telemetry->cell_voltage[i] = telemetry_battery->cell_voltage[i]; }
        telemetry->battery_valid |= LORA_TELEMETRY_TELEMETRY_VALID;
    }
    if (telemetry_getBoatRelativeOrientation(telemetry_boatRelative) == TELEMETRY_OK) 
    {
        // There are no packed telemetry fields for boat relative orientation, but we still need to get the data for later in the task
    }
    if (telemetry_getMainSail(telemetry_mainSail) == TELEMETRY_OK) {
        telemetry->sail_ang_forward = telemetry_mainSail->ang_forward;
        telemetry->encoder_valid |= LORA_TELEMETRY_TELEMETRY_VALID;
    }
    if (telemetry_getWind(telemetry_wind) == TELEMETRY_OK) {
        telemetry->ang_wind = telemetry_wind->ang_wind;
        telemetry->wind_valid |= LORA_TELEMETRY_TELEMETRY_VALID;
    }
    if (telemetry_getBoatGlobalOrientation(telemetry_boatGlobal) == TELEMETRY_OK) {
        // Rounded to 1e-7 deg only here, the CM7 keeps doubles for the fusion and navigation math
        telemetry->latitude  = (int32_t)lround(telemetry_boatGlobal->latitude * LORA_GPS_DEG_SCALE);
        telemetry->longitude = (int32_t)lround(telemetry_boatGlobal->longitude * LORA_GPS_DEG_SCALE);
        telemetry->heading   = telemetry_boatGlobal->angP_heading;
        telemetry->roll      = telemetry_boatGlobal->angP_roll;
        telemetry->pitch     = telemetry_boatGlobal->angP_pitch;
        telemetry->imu_valid |= LORA_TELEMETRY_TELEMETRY_VALID;
        telemetry->orientation_valid |= LORA_TELEMETRY_TELEMETRY_VALID;
        telemetry->gps_valid |= LORA_TELEMETRY_TELEMETRY_VALID;
    }
    // Servo angles have no validity field: 0 until the arbiter first moves the servo
    if (telemetry_getRudder(telemetry_rudder) == TELEMETRY_OK) {
        telemetry->rudder_servo_angle = telemetry_rudder->ang_forward;
    }
    if (telemetry_getTailSail(telemetry_tailSail) == TELEMETRY_OK) {
        telemetry->sail_servo_angle = telemetry_tailSail->ang_forward; // The sail servo drives the tail sail
    }

    telemetry->mode = commander_getMode();

    // The heading the PID is steering to: LTP's in AUTO (to within NAV_HEADING_DEADBAND_DEG), the ground
    // station's in semi-auto, and the last one held during an LTP fault. 0xFFFF = no target yet (0 is a
    // real heading, north)
    #ifdef CM7_PID_HEADING_ENABLE
    int16_t target_heading = pidHeading_getTarget();
    telemetry->desired_heading = (target_heading < 0) ? 0xFFFFu : (uint16_t)target_heading;
    #else
    telemetry->desired_heading = 0xFFFFu;
    #endif

    // From LTP. No leg (0xFF) and no drift when it has no route, is not in AUTO, or is not built in
    TelemetryNavigation_t navigation;
    if (navigationTask_getTelemetry(&navigation))
    {
        telemetry->route_segment = navigation.route_segment;
        telemetry->drift_distance = navigation.drift_distance;
    }
    else
    {
        telemetry->route_segment = NAV_SEGMENT_NONE;
        telemetry->drift_distance = 0;
    }
    telemetry->last_cmd_seq = commander_getLastSeq();
    telemetry->last_cmd_result = commander_getLastResult();
    
    telemetry->uptime_ms = HAL_GetTick();

    // true only if every source above had data
    return true;
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