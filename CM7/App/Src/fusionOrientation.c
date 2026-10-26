/////////////////////////////////////////////////////////////////////////////////////////
//
/////////////////////////////////////////////////////////////////////////////////////////

#include "sensorGPS.h"
#include "sensorIMU.h"
#include "fusionOrientation.h"
#include "objRing.h"
#include "sharedBoat.h"
#include "system.h"
#include "telemetry.h"

#include "queue.h"

#include <math.h>

/////////////////////////////////////////////////////////////////////////////////////////

typedef enum {
    RESULT_SUCCESS,
    RESULT_FAILURE
} FusionResult_e;

#define MAGNORTH_DECLINATION_SLC 11 // degrees, actual value is 10.85 but we use ints

// Change the below value to set the magnetic declination
#define MAGNORTH_DECLINATION MAGNORTH_DECLINATION_SLC

#define GPS_LAT2MM 111320000.0

/////////////////////////////////////////////////////////////////////////////////////////

TaskHandle_t fusionOrientation_task;

/////////////////////////////////////////////////////////////////////////////////////////

static uint8_t imu_sequence_prev;
static uint32_t imu_timestamp_prev;
static int16_t imu_quat[SENSOR_IMU_WINDOW][4];
static Ring_o  imu_quat_ring;
static int32_t imu_quat_sum[4];

static uint8_t gps_sequence_prev;
static double  gps_lat_cache;
static double  gps_lat[SENSOR_GPS_WINDOW];
static Ring_o  gps_lat_ring;
static double  gps_lon_cache;
static double  gps_lon[SENSOR_GPS_WINDOW];
static Ring_o  gps_lon_ring;
static int32_t gps_timestamp[SENSOR_GPS_WINDOW];
static Ring_o  gps_timestamp_ring;

static float fusion_linV[3];

static FusionResult_e quaternion_align(int16_t quat[4]);
static FusionResult_e quaternion_normalize(int32_t quat[4], float quat_norm[4]);
static FusionResult_e quaternion_toEuler(float quat[4], int16_t euler[3]);

static FusionResult_e globalToRelative_linVSurge(double gps_delta[2], double gps_lat, int32_t timestamp_delta, float *linV_surge);

/////////////////////////////////////////////////////////////////////////////////////////

HardwareInit_e fusionOrientation_hardwareInit(void)
{
    fusionOrientation_printf("fusionOrientation_hardwareInit start");
    fusionOrientation_printf("fusionOrientation_hardwareInit complete");
    return HARDWARE_INIT_SUCCESS;
}



RtosInit_e fusionOrientation_rtosInit(void)
{
    fusionOrientation_printf("fusionOrientation_rtosInit start");

    fusionOrientation_printf("    - init imu_quat ring buffer");
    imu_sequence_prev = 255;
    imu_timestamp_prev = 0;
    objRing_init(&imu_quat_ring, imu_quat, sizeof(imu_quat[0]), SENSOR_IMU_WINDOW);
    imu_quat_sum[0] = imu_quat_sum[1] = imu_quat_sum[2] = imu_quat_sum[3] = 0;

    fusionOrientation_printf("    - init gps ring buffer");
    gps_sequence_prev = 255;
    objRing_init(&gps_lat_ring, gps_lat, sizeof(gps_lat[0]), SENSOR_GPS_WINDOW);
    gps_lat_cache = 0;
    objRing_init(&gps_lon_ring, gps_lon, sizeof(gps_lon[0]), SENSOR_GPS_WINDOW);
    gps_lon_cache = 0;
    objRing_init(&gps_timestamp_ring, gps_timestamp, sizeof(gps_timestamp[0]), SENSOR_GPS_WINDOW);

    fusionOrientation_printf("    - init fused values");
    fusion_linV[0] = fusion_linV[1] = fusion_linV[2] = 0.0f;

    fusionOrientation_printf("fusionOrientation_rtosInit complete");
    return RTOS_INIT_SUCCESS;
}



void fusionOrientation_handler(void *argument)
{
    fusionOrientation_printf("fusionOrientation_handler start");
    for(;;)
    {
        if (ulTaskNotifyTake(pdTRUE, pdMS_TO_TICKS(SENSOR_IMU_STALE)) == 0) { fusionOrientation_printf("Fused Orientation stale."); }
        else
        {
            TelemetryBoatRelativeOrientation_t relativeOrientation = {0};
            TelemetryBoatGlobalOrientation_t globalOrientation = {0};

            MetadataIMU_t imu = {0};
            if (!sensorIMU_getMetadata(&imu)) { fusionOrientation_printf("Failed to get IMU metadata"); continue; }
            if (imu.valid != 0 && imu.valid < 0x10  && imu.sequence != imu_sequence_prev)
            {
                imu_sequence_prev = imu.sequence;

                int16_t quat[4] = {imu.quat[0], imu.quat[1], imu.quat[2], imu.quat[3]};
                if (quaternion_align(quat) != RESULT_SUCCESS) { fusionOrientation_printf("Failed to align quaternion"); continue; }
                for (int i = 0; i < 4; i++) { imu_quat_sum[i] += quat[i]; }
                int16_t quat_old[4];
                if (objRing_push(&imu_quat_ring, &quat, &quat_old)) { for (int i = 0; i < 4; i++) { imu_quat_sum[i] -= quat_old[i]; } }

                if (imu_timestamp_prev != 0) 
                {
                    // TODO: which axis goes to what will change once the chip is officially mounted
                    int32_t imu_timestamp_delta = imu.timestamp - imu_timestamp_prev;
                    fusion_linV[0] += imu.lnAc[2] * 10.0f * imu_timestamp_delta / 1000.0f; // up/down
                    fusion_linV[1] += imu.lnAc[1] * 10.0f * imu_timestamp_delta / 1000.0f; // forward/backward
                    fusion_linV[2] += imu.lnAc[0] * 10.0f * imu_timestamp_delta / 1000.0f; // sideways
                }
                imu_timestamp_prev = imu.timestamp;
            }

            float quat_norm[4];
            if (quaternion_normalize(imu_quat_sum, quat_norm) != RESULT_SUCCESS) { fusionOrientation_printf("Failed to normalize quaternion"); continue; }
            int16_t euler[3];
            if (quaternion_toEuler(quat_norm, euler) != RESULT_SUCCESS) { fusionOrientation_printf("Failed to convert quaternion to euler"); continue; }
            
            relativeOrientation.linV_heave = (int16_t)lround(fusion_linV[0]);
            relativeOrientation.linV_surge = (int16_t)lround(fusion_linV[1]);
            relativeOrientation.linV_sway = (int16_t)lround(fusion_linV[2]);
            relativeOrientation.linA_heave = imu.lnAc[2]; // TODO: this section has the same problem as the above TODO
            relativeOrientation.linA_surge = imu.lnAc[1];
            relativeOrientation.linA_sway = imu.lnAc[0];
            telemetry_setBoatRelativeOrientation(&relativeOrientation);

            globalOrientation.angP_roll = ANG_WRAP_180(euler[0]);
            globalOrientation.angP_pitch = ANG_WRAP_180(euler[1]);
            globalOrientation.angP_heading = ANG_WRAP_360(euler[2] + MAGNORTH_DECLINATION);

            MetadataGPS_t gps = {0};
            if (!sensorGPS_getMetadata(&gps)) { fusionOrientation_printf("Failed to get GPS metadata"); continue; }
            if (gps.valid != 0 && gps.valid < 0x10 && gps.sequence != gps_sequence_prev)
            {
                gps_sequence_prev = gps.sequence;

                gps_lat_cache = gps.latitude;
                double gps_lat_old;
                bool lat_result;
                lat_result = objRing_push(&gps_lat_ring, &gps.latitude, &gps_lat_old);
                
                gps_lon_cache = gps.longitude;
                double gps_lon_old;
                bool lon_result;
                lon_result = objRing_push(&gps_lon_ring, &gps.longitude, &gps_lon_old);
                
                int32_t gps_timestamp_old;
                bool timestamp_result;
                timestamp_result = objRing_push(&gps_timestamp_ring, &gps.timestamp, &gps_timestamp_old);

                if (lat_result && lon_result && timestamp_result)
                {
                    double gps_delta[2] = {gps.latitude - gps_lat_old, gps.longitude - gps_lon_old};
                    int32_t timestamp_delta = gps.timestamp - gps_timestamp_old;
                    fusion_linV[0] = 0.0f;
                    if (globalToRelative_linVSurge(gps_delta, gps.latitude, timestamp_delta, &fusion_linV[1]) != RESULT_SUCCESS) { fusionOrientation_printf("Failed to convert GPS change into linear velocity"); }
                    fusion_linV[2] = 0.0f;
                }
            }

            globalOrientation.latitude = gps_lat_cache;
            globalOrientation.longitude = gps_lon_cache;

            telemetry_setBoatGlobalOrientation(&globalOrientation);
        }
    }
}

/////////////////////////////////////////////////////////////////////////////////////////

static FusionResult_e quaternion_align(int16_t quat[4])
{
    int16_t quat_prev[4];
    if (objRing_get(&imu_quat_ring, 0, &quat_prev))
    {
        if (quat[0]*quat_prev[0] + quat[1]*quat_prev[1] + quat[2]*quat_prev[2] + quat[3]*quat_prev[3] < 0)
        {
            quat[0] = -quat[0];
            quat[1] = -quat[1];
            quat[2] = -quat[2];
            quat[3] = -quat[3];
        }
    }
    return RESULT_SUCCESS;
}

static FusionResult_e quaternion_normalize(int32_t quat[4], float quat_norm[4])
{
    float quat_f[4] = {quat[0], quat[1], quat[2], quat[3]};
    float sum_sq = quat_f[0]*quat_f[0] + quat_f[1]*quat_f[1] + quat_f[2]*quat_f[2] + quat_f[3]*quat_f[3];
    float norm = sqrtf(sum_sq);
    if (norm < 1e-6)
        return RESULT_FAILURE;

    float norm_inv = 1 / norm;
    quat_norm[0] = quat[0] * norm_inv;
    quat_norm[1] = quat[1] * norm_inv;
    quat_norm[2] = quat[2] * norm_inv;
    quat_norm[3] = quat[3] * norm_inv;
    return RESULT_SUCCESS;
}

static FusionResult_e quaternion_toEuler(float quat[4], int16_t euler[3])
{
    float roll = atan2f(2.0f * (quat[0]*quat[1] + quat[2]*quat[3]), 1.0f - 2.0f * (quat[1]*quat[1] + quat[2]*quat[2]));
    float sinp = 2.0f * (quat[0]*quat[2] - quat[3]*quat[1]);
    if (sinp >  1.0f) sinp =  1.0f;
    if (sinp < -1.0f) sinp = -1.0f;
    float pitch = asinf(sinp);
    float yaw = atan2f(2.0f * (quat[0]*quat[3] + quat[1]*quat[2]), 1.0f - 2.0f * (quat[2]*quat[2] + quat[3]*quat[3]));
    
    euler[0] = (int16_t)(roll * RAD2DEG);
    euler[1] = (int16_t)(pitch * RAD2DEG);
    euler[2] = (int16_t)(yaw * RAD2DEG);
    
    return RESULT_SUCCESS;
}

static FusionResult_e globalToRelative_linVSurge(double gps_delta[2], double gps_lat, int32_t timestamp_delta, float *linV_surge)
{
    if (timestamp_delta <= 0) { return RESULT_FAILURE; }
    double north_mm = gps_delta[0] * GPS_LAT2MM;
    double east_mm  = gps_delta[1] * GPS_LAT2MM * cos(gps_lat * DEG2RAD);
    *linV_surge = sqrt(north_mm*north_mm + east_mm*east_mm) / timestamp_delta * 1000.0f;
    return RESULT_SUCCESS;
}