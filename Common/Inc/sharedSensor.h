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
    uint16_t raw_cell[BATTERY_CELL_COUNT];
    uint16_t voltage_cell[BATTERY_CELL_COUNT];

    uint32_t timestamp; /**< Timestamp for when the measurement was taken, measured in ms */
    uint8_t valid; /**< Indicates if the measurement is valid */
    uint8_t sequence; /**< Sequence number of the measurement, used to ensure a measurment is actually new */
} MetadataBattery_t;



/////////////////////////////////////////////////////////////////////////////
// Encoder Sensor

/**
 * Represents metadata associated with the encoder readings.
 *
 * SENSOR: Encoder
 */
typedef struct {

    uint32_t timestamp; /**< Timestamp for when the measurement was taken, measured in ms */
    uint8_t valid; /**< Indicates if the measurement is valid */
    uint8_t sequence; /**< Sequence number of the measurement, used to ensure a measurment is actually new */
} MetadataEncoder_t;



/////////////////////////////////////////////////////////////////////////////
// GPS Sensor

/**
 * Represents metadata associated with the GPS readings.
 *
 * SENSOR: Global Positioning System (GPS)
 */
typedef struct {

    uint32_t timestamp; /**< Timestamp for when the measurement was taken, measured in ms */
    uint8_t valid; /**< Indicates if the measurement is valid */
    uint8_t sequence; /**< Sequence number of the measurement, used to ensure a measurment is actually new */
} MetadataGPS_t;



/////////////////////////////////////////////////////////////////////////////
// IMU Sensor

/**
 * Represents metadata associated with the IMU readings.
 *
 * SENSOR: Inertial Measurement Unit (IMU)
 */
typedef struct {

    uint32_t timestamp; /**< Timestamp for when the measurement was taken, measured in ms */
    uint8_t valid; /**< Indicates if the measurement is valid */
    uint8_t sequence; /**< Sequence number of the measurement, used to ensure a measurment is actually new */
} MetadataIMU_t;



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