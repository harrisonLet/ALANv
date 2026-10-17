#pragma once

/////////////////////////////////////////////////////////////////////////////////////////
// Shared Configuration
/////////////////////////////////////////////////////////////////////////////////////////
#define DUAL_CORE_BOOT_SYNC_SEQUENCE


/////////////////////////////////////////////////////////////////////////////////////////
// CM4 Configuration
/////////////////////////////////////////////////////////////////////////////////////////
// #define CM4_ENABLE
// #define CM4_SERVOS_ENABLE



#ifdef CM4_ENABLE
#define CM4_DEBUG
#endif


/////////////////////////////////////////////////////////////////////////////////////////
// CM7 Configuration
/////////////////////////////////////////////////////////////////////////////////////////
#define CM7_ENABLE
#define CM7_LIVE_ENABLE
#define CM7_TELEMETRY_ENABLE
#define CM7_SENSOR_BATTERY_ENABLE
// #define CM7_SENSOR_ENCODER_ENABLE
// #define CM7_SENSOR_GPS_ENABLE
#define CM7_SENSOR_IMU_ENABLE
// #define CM7_SENSOR_WIND_ENABLE



#ifdef CM7_ENABLE
#define CM7_DEBUG
#endif

#ifdef CM7_LIVE_ENABLE
// #define CM7_LIVE_DEBUG
// #define CM7_LIVE_DEBUG_STACK // CM7_LIVE_DEBUG must be enabled
#endif

#ifdef CM7_TELEMETRY_ENABLE
// #define CM7_TELEMETRY_DEBUG
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
#define CM7_SENSOR_IMU_DEBUG
#define CM7_SENSOR_IMU_PRE_CALIBRATED // If active the IMU will use the hard-coded calibration offsets
#endif

#ifdef CM7_SENSOR_WIND_ENABLE
#define CM7_SENSOR_WIND_DEBUG
#endif