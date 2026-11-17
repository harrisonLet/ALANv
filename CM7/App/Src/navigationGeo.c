/////////////////////////////////////////////////////////////////////////////////////////
// Navigation geometry: local meters, bearings, track errors, angle wrapping.
// Pure C, never include system.h. Tested on a laptop by CM7/Test/navigation.
/////////////////////////////////////////////////////////////////////////////////////////

#include "navigationGeo.h"

#include <math.h>

/////////////////////////////////////////////////////////////////////////////////////////

double navGeo_wrap360(double deg)
{
    double r = fmod(deg, 360.0);
    if (r < 0.0) { r += 360.0; }
    if (r >= 360.0) { r -= 360.0; } // a tiny negative r rounds up to 360 above
    return r;
}



double navGeo_wrap180(double deg)
{
    return navGeo_wrap360(deg + 180.0) - 180.0;
}



NavLocal_t navGeo_toLocal(double lat0, double lon0, double lat, double lon)
{
    NavLocal_t p;
    p.north = (lat - lat0) * NAV_M_PER_DEG_LAT;
    p.east  = (lon - lon0) * NAV_M_PER_DEG_LAT * cos(lat0 * NAV_DEG2RAD);
    return p;
}



double navGeo_bearing(NavLocal_t from, NavLocal_t to)
{
    return navGeo_wrap360(atan2(to.east - from.east, to.north - from.north) * NAV_RAD2DEG);
}



double navGeo_distance(NavLocal_t from, NavLocal_t to)
{
    return hypot(to.north - from.north, to.east - from.east);
}



// Leg vector d = end - start, boat vector b = boat - start: xte = (d x b) / |d|
double navGeo_crossTrack(NavLocal_t start, NavLocal_t end, NavLocal_t boat)
{
    double dn = end.north - start.north, de = end.east - start.east;
    double bn = boat.north - start.north, be = boat.east - start.east;
    double len = hypot(dn, de);
    if (len <= 0.0) { return 0.0; }
    return (dn * be - de * bn) / len;
}



// along = (d . b) / |d|
double navGeo_alongTrack(NavLocal_t start, NavLocal_t end, NavLocal_t boat)
{
    double dn = end.north - start.north, de = end.east - start.east;
    double bn = boat.north - start.north, be = boat.east - start.east;
    double len = hypot(dn, de);
    if (len <= 0.0) { return 0.0; }
    return (dn * bn + de * be) / len;
}



double navGeo_circularMean(const double *deg, size_t count)
{
    double sum_sin = 0.0, sum_cos = 0.0;
    for (size_t i = 0; i < count; i++)
    {
        sum_sin += sin(deg[i] * NAV_DEG2RAD);
        sum_cos += cos(deg[i] * NAV_DEG2RAD);
    }
    return navGeo_wrap360(atan2(sum_sin, sum_cos) * NAV_RAD2DEG);
}
