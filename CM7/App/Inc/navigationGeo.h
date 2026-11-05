#ifndef NAVIGATION_GEO_H
#define NAVIGATION_GEO_H

// Pure geometry for navigation. Never include system.h here (it pulls in HAL and FreeRTOS),
// so this file also builds on a laptop for CM7/Test/navigation.

#include <stddef.h>

#define NAV_PI              3.14159265358979323846
#define NAV_DEG2RAD         (NAV_PI / 180.0)
#define NAV_RAD2DEG         (180.0 / NAV_PI)
#define NAV_M_PER_DEG_LAT   111320.0 // same scale as GPS_LAT2MM in fusionOrientation.c

/**
 * A point in a local flat-earth frame, in meters from an origin (normally the leg start).
 * Accurate enough for legs under a few km.
 */
typedef struct {
    double north; // m, + = north of the origin
    double east;  // m, + = east of the origin
} NavLocal_t;

/**
  * @brief  Wrap an angle to [0, 360).
  * @param  deg  Angle in degrees, any value.
  * @retval Angle in degrees, 0 <= result < 360.
  */
double navGeo_wrap360(double deg);

/**
  * @brief  Wrap an angle to [-180, 180), the same range as ANG_WRAP_180 in system.h.
  * @param  deg  Angle in degrees, any value.
  * @retval Angle in degrees, -180 <= result < 180.
  */
double navGeo_wrap180(double deg);

/**
  * @brief  Convert a GPS position to meters north and east of an origin.
  * @param  lat0, lon0  Origin, degrees.
  * @param  lat, lon    Position to convert, degrees.
  * @retval Position in the origin's local frame.
  */
NavLocal_t navGeo_toLocal(double lat0, double lon0, double lat, double lon);

/**
  * @brief  Bearing from one point to another.
  * @retval Degrees true, 0..360 clockwise from north. 0 when the points are equal.
  */
double navGeo_bearing(NavLocal_t from, NavLocal_t to);

/**
  * @brief  Straight-line distance between two points.
  * @retval Meters.
  */
double navGeo_distance(NavLocal_t from, NavLocal_t to);

/**
  * @brief  Signed distance from the boat to the line through start and end.
  * @retval Meters, + = starboard of the path (right, looking from start to end).
  *         0 when start and end are the same point.
  */
double navGeo_crossTrack(NavLocal_t start, NavLocal_t end, NavLocal_t boat);

/**
  * @brief  Distance travelled along the leg: the boat projected onto the line through start and end.
  * @retval Meters from start toward end. Negative behind start, at or past the leg length beyond end.
  *         0 when start and end are the same point.
  */
double navGeo_alongTrack(NavLocal_t start, NavLocal_t end, NavLocal_t boat);

/**
  * @brief  Circular mean of a set of angles, so 350 and 10 average to 0, not 180.
  * @param  deg    Angles in degrees.
  * @param  count  Number of angles.
  * @retval Degrees, 0..360. 0 when count is 0 or the angles cancel out (e.g. 0 and 180).
  */
double navGeo_circularMean(const double *deg, size_t count);

#endif // NAVIGATION_GEO_H
