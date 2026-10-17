#pragma once

#include <stdint.h>
#include "sharedBattery.h"

/////////////////////////////////////////////////////////////////////////////
// Shared Constants

/**
 * Validity enumeration for sensor readings.
 */
typedef enum {
    VALIDITY_UNKNOWN                        = 0x00, /**< Sensor validity is unknown, this was set to 0 as newly initialized data blocks will be 0 */
    
    VALIDITY_VALID                          = 0x01, /**< Sensor validity is confirmed to be valid */
    VALIDITY_VALID_CALIBRATING              = 0x02, /**< Sensor is currently calibrating, such as an IMU performing self-calibration */
    VALIDITY_VALID_WARMING                  = 0x03, /**< Sensor is currently warming up, such as GPS waiting for a lock */
    VALIDITY_VALID_WINDOW_FILLING           = 0x04, /**< Sensor is currently filling its measurement window, such as the averaging window of the wind sensor */
    VALIDITY_VALID_DISABLED                 = 0x05, /**< Sensor is currently disabled */
    VALIDITY_VALID_PARTIAL_DATA             = 0x06, /**< This really only exists for the GPS sensor which provides all of its data in separate messages, it is possible for use to have valid data but not new data in every field */

    VALIDITY_UNTRUSTED                      = 0x10, /**< Sensor is untrusted */
    VALIDITY_UNTRUSTED_VALUE_OUT_OF_RANGE   = 0x11, /**< Sensor value is out of range */
    VALIDITY_UNTRUSTED_VALUE_DEGRADED       = 0x12, /**< Sensor value is degraded from the accuracy we expect */
    VALIDITY_UNTRUSTED_VALUE_IMPLAUSIBLE    = 0x13, /**< Sensor value is implausible for what it is measureing, such as the battery being at 0 volts */
    VALIDITY_UNTRUSTED_VALUE_STATIC         = 0x14, /**< Sensor value has remained static for to long */
    VALIDITY_UNTRUSTED_OVER_SATURATION      = 0x15, /**< Sensor is experiencing over saturation, such as a railed sensor reading */
    VALIDITY_UNTRUSTED_UNDER_SATURATION     = 0x16, /**< Sensor is experiencing under saturation, such as a sensor reading below its minimum detectable value */

    VALIDITY_ERROR                          = 0x20, /**< Sensor has encountered an error */
    VALIDITY_ERROR_UNREACHABLE              = 0x21, /**< Sensor is unreachable */
    VALIDITY_ERROR_PARSE_FAIL               = 0x22, /**< Sensor data could not be parsed */
    VALIDITY_ERROR_UNKNOWN                  = 0x23, /**< Sensor has encountered an unknown error */

    VALIDITY_INVALID_BATTERY                = 0x80, /**< Generic validity failure for the battery sensor */
    VALIDITY_INVALID_ENCODER                = 0x90, /**< Generic validity failure for the encoder sensor */
    VALIDITY_INVALID_GPS                    = 0xA0, /**< Generic validity failure for the GPS sensor */
    VALIDITY_INVALID_GPS_STALE              = 0xA1, /**< GPS data has not been updated recently (stale) */
    VALIDITY_INVALID_GPS_NO_FIX             = 0xA2, /**< GPS has no fix */
    VALIDITY_INVALID_GPS_CHECKSUM           = 0xA3, /**< GPS sentence checksum failed */
    VALIDITY_INVALID_IMU                    = 0xB0, /**< Generic validity failure for the IMU sensor */
    VALIDITY_INVALID_WIND                   = 0xC0, /**< Generic validity failure for the wind sensor */
} Validity_e;



/////////////////////////////////////////////////////////////////////////////
// Battery Monitor Sensor

/**
 * Represents metadata associated with the battery monitor readings.
 *
 * SENSOR: Battery Monitor
 */
typedef struct {
    uint16_t raw_cell[BATTERY_CELL_COUNT]; /**< Raw ADC readings for each battery cell */

    uint32_t timestamp; /**< Timestamp for when the measurement was taken, measured in ms */
    uint8_t valid; /**< Indicates if the measurement is valid */
    uint8_t sequence; /**< Sequence number of the measurement, used to ensure a measurment is actually new */
} MetadataBattery_t;

#define METADATA_BATTERY_STR_SIZE 128u
char *MetadataBattery_toString(const MetadataBattery_t *metadata, char buf[METADATA_BATTERY_STR_SIZE]);



/////////////////////////////////////////////////////////////////////////////
// Encoder Sensor

/**
 * Represents metadata associated with the encoder readings.
 *
 * SENSOR: Encoder
 */
typedef struct {
    uint16_t raw_angle;   /**< 0-4095, unscaled angle */
    uint16_t angle;       /**< 0-4095, scaled by ZPOS/MPOS/MANG (same as raw_angle until those are programmed) */
    uint16_t magnitude;   /**< internal CORDIC magnitude */
    uint8_t  status;      /**< MD/ML/MH bits */
    uint8_t  agc;         /**< gain: 0-255 at 5V, 0-128 at 3.3V (aim for the middle) */

    uint32_t timestamp;   /**< Timestamp for when the measurement was taken, measured in ms */
    uint8_t valid;        /**< Indicates if the measurement is valid */
    uint8_t sequence;     /**< Sequence number of the measurement, used to ensure a measurment is actually new */
} MetadataEncoder_t;

#define METADATA_ENCODER_STR_SIZE 128u
char *MetadataEncoder_toString(const MetadataEncoder_t *metadata, char buf[METADATA_ENCODER_STR_SIZE]);



/////////////////////////////////////////////////////////////////////////////
// GPS Sensor
#define METADATA_GPS_RAW_STR_SIZE 128u

/**
 * Represents metadata associated with the GPS readings.
 *
 * SENSOR: Global Positioning System (GPS)
 */
typedef struct {
    char raw[METADATA_GPS_RAW_STR_SIZE]; /**< Raw NMEA string from the GPS module */
    double latitude;  /**< Latitude in degrees */
    double longitude; /**< Longitude in degrees */
    float altitude;   /**< Altitude in meters */
    uint8_t satellites; /**< Number of satellites in view */
    uint8_t fix_valid; /**< Indicates if the GPS fix is valid */
    uint8_t hours;     /**< Hours part of the timestamp */
    uint8_t minutes;   /**< Minutes part of the timestamp */
    uint8_t seconds;   /**< Seconds part of the timestamp */

    uint32_t timestamp; /**< Timestamp for when the measurement was taken, measured in ms */
    uint8_t valid; /**< Indicates if the measurement is valid */
    uint8_t sequence; /**< Sequence number of the measurement, used to ensure a measurment is actually new */
} MetadataGPS_t;

#define METADATA_GPS_STR_SIZE 256u
char *MetadataGPS_toString(const MetadataGPS_t *metadata, char buf[METADATA_GPS_STR_SIZE]);



/////////////////////////////////////////////////////////////////////////////
// IMU Sensor

/**
 * Represents metadata associated with the IMU readings.
 *
 * SENSOR: Inertial Measurement Unit (IMU)
 */
typedef struct {
    int16_t quat[4];   /**< Fused quaternion readings */
    int16_t lnAc[3];   /**< Fused linear acceleration readings */
    int8_t tmp[1];     /**< Temperature reading */
    
    uint32_t timestamp; /**< Timestamp for when the measurement was taken, measured in ms */
    uint8_t valid; /**< Indicates if the measurement is valid */
    uint8_t sequence; /**< Sequence number of the measurement, used to ensure a measurment is actually new */
} MetadataIMU_t;

#define METADATA_IMU_STR_SIZE 78u
char *MetadataIMU_toString(const MetadataIMU_t *metadata, char buf[METADATA_IMU_STR_SIZE]);



/////////////////////////////////////////////////////////////////////////////
// Wind Sensor

/**
 * Represents metadata associated with the wind sensor readings.
 *
 * SENSOR: Wind Sensor
 */
typedef struct {

    uint32_t timestamp; /**< Timestamp for when the measurement was taken, measured in ms */
    uint8_t valid; /**< Indicates if the measurement is valid */
    uint8_t sequence; /**< Sequence number of the measurement, used to ensure a measurment is actually new */
} MetadataWind_t;

#define METADATA_WIND_STR_SIZE 128u
char *MetadataWind_toString(const MetadataWind_t *metadata, char buf[METADATA_WIND_STR_SIZE]);