/////////////////////////////////////////////////////////////////////////////
// Shared Constants
#define SENSOR_INVALID 0
#define SENSOR_VALID 1



/////////////////////////////////////////////////////////////////////////////
// Battery Monitor Sensor

/**
 * Represents metadata associated with the battery monitor readings.
 *
 * SENSOR: Battery Monitor
 */
typedef struct {

    uint32_t timestamp; /**< Timestamp for when the measurement was taken */
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

    uint32_t timestamp; /**< Timestamp for when the measurement was taken */
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

    uint32_t timestamp; /**< Timestamp for when the measurement was taken */
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

    uint32_t timestamp; /**< Timestamp for when the measurement was taken */
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

    uint32_t timestamp; /**< Timestamp for when the measurement was taken */
    uint8_t valid; /**< Indicates if the measurement is valid */
    uint8_t sequence; /**< Sequence number of the measurement, used to ensure a measurment is actually new */
} MetadataWind_t;