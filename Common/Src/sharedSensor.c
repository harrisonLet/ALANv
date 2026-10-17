#include "sharedSensor.h"
#include <stdio.h>

/////////////////////////////////////////////////////////////////////////////
// Battery Monitor Sensor
char *MetadataBattery_toString(const MetadataBattery_t *metadata, char buf[METADATA_BATTERY_STR_SIZE])
{
    snprintf(buf, METADATA_BATTERY_STR_SIZE, "MetadataBattery (%08lu, %02X, %03u): [ %05u | %05u | %05u | %05u ]",
        (unsigned long)metadata->timestamp, (unsigned)metadata->valid, (unsigned)metadata->sequence,
        (unsigned)metadata->raw_cell[0],
        (unsigned)metadata->raw_cell[1],
        (unsigned)metadata->raw_cell[2],
        (unsigned)metadata->raw_cell[3]
    );
    return buf;
}



/////////////////////////////////////////////////////////////////////////////
// Encoder Sensor
char *MetadataEncoder_toString(const MetadataEncoder_t *metadata, char buf[METADATA_ENCODER_STR_SIZE])
{
    snprintf(buf, METADATA_ENCODER_STR_SIZE, "MetadataEncoder (%08lu, %02X, %03u): %04uRA  %04uA  %05uM  ( %02X, %02X )",
        (unsigned long)metadata->timestamp, (unsigned)metadata->valid, (unsigned)metadata->sequence,
        (unsigned)metadata->raw_angle,
        (unsigned)metadata->angle,
        (unsigned)metadata->magnitude,
        (unsigned)metadata->status, (unsigned)metadata->agc
    );
    return buf;
}



/////////////////////////////////////////////////////////////////////////////
// GPS Sensor
char *MetadataGPS_toString(const MetadataGPS_t *metadata, char buf[METADATA_GPS_STR_SIZE])
{
    snprintf(buf, METADATA_GPS_STR_SIZE, "MetadataGPS (%08lu, %02X, %03u): [ %+10.6f | %+11.6f | %+8.2f ] %03dS %02XF - %02d:%02d:%02d - %s",
        (unsigned long)metadata->timestamp, (unsigned)metadata->valid, (unsigned)metadata->sequence,
        metadata->latitude, metadata->longitude, metadata->altitude,
        (unsigned)metadata->satellites, (unsigned)metadata->fix_valid,
        (unsigned)metadata->hours, (unsigned)metadata->minutes, (unsigned)metadata->seconds,
        metadata->raw
    );
    return buf;
}



/////////////////////////////////////////////////////////////////////////////
// IMU Sensor
char *MetadataIMU_toString(const MetadataIMU_t *metadata, char buf[METADATA_IMU_STR_SIZE])
{
    snprintf(buf, METADATA_IMU_STR_SIZE, "MetadataIMU (%08lu, %02X, %03u): [ %+05d | %+05d | %+05d | %+05d ] ( %+03d )",
        (unsigned long)metadata->timestamp, (unsigned)metadata->valid, (unsigned)metadata->sequence,
        metadata->quat[0], metadata->quat[1], metadata->quat[2], metadata->quat[3],
        metadata->tmp[0]
    );
    return buf;
}



/////////////////////////////////////////////////////////////////////////////
// Wind Sensor
char *MetadataWind_toString(const MetadataWind_t *metadata, char buf[METADATA_WIND_STR_SIZE])
{
    return buf;
}