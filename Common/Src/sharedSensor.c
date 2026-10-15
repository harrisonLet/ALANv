#include "sharedSensor.h"
#include <stdio.h>

/////////////////////////////////////////////////////////////////////////////
// Battery Monitor Sensor
char *MetadataBattery_toString(const MetadataBattery_t *metadata, char buf[METADATA_BATTERY_STR_SIZE])
{
    snprintf(buf, METADATA_BATTERY_STR_SIZE, "MetadataBattery (%08lu, %02X, %03u): [ %05u:%04u | %05u:%04u | %05u:%04u | %05u:%04u | R:mV ]",
        (unsigned long)metadata->timestamp, (unsigned)metadata->valid, (unsigned)metadata->sequence,
        (unsigned)metadata->raw_cell[0], (unsigned)metadata->voltage_cell[0],
        (unsigned)metadata->raw_cell[1], (unsigned)metadata->voltage_cell[1],
        (unsigned)metadata->raw_cell[2], (unsigned)metadata->voltage_cell[2],
        (unsigned)metadata->raw_cell[3], (unsigned)metadata->voltage_cell[3]
    );
    return buf;
}



/////////////////////////////////////////////////////////////////////////////
// Encoder Sensor
char *MetadataEncoder_toString(const MetadataEncoder_t *metadata, char buf[METADATA_ENCODER_STR_SIZE])
{
    return buf;
}



/////////////////////////////////////////////////////////////////////////////
// GPS Sensor
char *MetadataGPS_toString(const MetadataGPS_t *metadata, char buf[METADATA_GPS_STR_SIZE])
{
    return buf;
}



/////////////////////////////////////////////////////////////////////////////
// IMU Sensor
char *MetadataIMU_toString(const MetadataIMU_t *metadata, char buf[METADATA_IMU_STR_SIZE])
{
    return buf;
}



/////////////////////////////////////////////////////////////////////////////
// Wind Sensor
char *MetadataWind_toString(const MetadataWind_t *metadata, char buf[METADATA_WIND_STR_SIZE])
{
    return buf;
}