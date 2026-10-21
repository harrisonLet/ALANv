#pragma once

/////////////////////////////////////////////////////////////////////////////////////////
// Shared Configuration
/////////////////////////////////////////////////////////////////////////////////////////
#define DUAL_CORE_BOOT_SYNC_SEQUENCE
#define IPC_ENABLE // Inter-Processor Communication (CM7 <-> CM4)


/////////////////////////////////////////////////////////////////////////////////////////
// CM4 Configuration
/////////////////////////////////////////////////////////////////////////////////////////
#define CM4_ENABLE
// #define CM4_SERVOS_ENABLE



#ifdef CM4_ENABLE
#define CM4_DEBUG
#endif


/////////////////////////////////////////////////////////////////////////////////////////
// CM7 Configuration
/////////////////////////////////////////////////////////////////////////////////////////
#define CM7_ENABLE
#define CM7_LIVE_ENABLE
// #define CM7_TELEMETRY_ENABLE
// #define CM7_SENSOR_BATTERY_ENABLE
// #define CM7_SENSOR_ENCODER_ENABLE
// #define CM7_SENSOR_GPS_ENABLE
// #define CM7_SENSOR_IMU_ENABLE
// #define CM7_SENSOR_WIND_ENABLE
// #define CM7_FUSION_ORIENTATION_ENABLE // This module requires data from the SENSOR_GPS and SENSOR_IMU tasks, it will not run if they are not also enabled



#ifdef CM7_ENABLE
#define CM7_DEBUG
#endif

#ifdef CM7_LIVE_ENABLE
#define CM7_LIVE_DEBUG
#define CM7_LIVE_DEBUG_STACK // CM7_LIVE_DEBUG must be enabled
#endif

#ifdef CM7_TELEMETRY_ENABLE
//#define CM7_TELEMETRY_DEBUG
#endif

#ifdef CM7_SENSOR_BATTERY_ENABLE
// #define CM7_SENSOR_BATTERY_DEBUG
#endif

#ifdef CM7_SENSOR_ENCODER_ENABLE
// #define CM7_SENSOR_ENCODER_DEBUG
#endif

#ifdef CM7_SENSOR_GPS_ENABLE
// #define CM7_SENSOR_GPS_DEBUG
#endif

#ifdef CM7_SENSOR_IMU_ENABLE
// #define CM7_SENSOR_IMU_DEBUG
#define CM7_SENSOR_IMU_PRE_CALIBRATED // If active the IMU will use the hard-coded calibration offsets
// #define CM7_SENSOR_IMU_FULL_DATA // If active the IMU will provide data for all individual sensors in additon to the fused data, if inactive it will provide only fused data
#endif

#ifdef CM7_SENSOR_WIND_ENABLE
// #define CM7_SENSOR_WIND_DEBUG
#endif

#ifdef CM7_FUSION_ORIENTATION_ENABLE
//#define CM7_FUSION_ORIENTATION_DEBUG
#endif
