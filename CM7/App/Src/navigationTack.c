/////////////////////////////////////////////////////////////////////////////////////////
// Live Tack Planning step. No RTOS calls, never include system.h.
//
// STUB: compiles only. The body comes with the LTP step of the design plan.
/////////////////////////////////////////////////////////////////////////////////////////

#include "navigationTack.h"
#include "navigationGeo.h"
#include <math.h>
#include "sharedSensor.h"

/////////////////////////////////////////////////////////////////////////////////////////

void navTack_step(const NavTackInput_t *in, NavTackState_t *state, TelemetryNavigation_t *out)
{
    (void)in;
    (void)state;
    (void)out;
}


static double wind_from(const NavTackInput_t *in) {
    double awd = navGeo_wrap360(in->heading + in->ang_wind);          // apparent, "from"
    double W = NAV_TRUE_WIND_ASSUMED_MPS;

    if (W <= 0.0 || !in->vel_valid || in->sog_mps < NAV_SOG_MIN_MPS) return awd;

    double to = (awd + 180.0) * NAV_DEG2RAD, c = in->cog_deg * NAV_DEG2RAD;
    double ux = sin(to), uy = cos(to);                                 // air direction (east, north)
    double vx = in->sog_mps * sin(c), vy = in->sog_mps * cos(c);       // boat velocity
    double uv = ux * vx + uy * vy, d = uv * uv - (vx * vx + vy * vy) + W * W;
    
    if (d < 0.0) return awd;                                           // assumed wind too light: give up
    double A = -uv + sqrt(d);                                          // apparent wind speed
    double tx = A * ux + vx, ty = A * uy + vy;                         // true air velocity
    return navGeo_wrap360(atan2(tx, ty) * NAV_RAD2DEG + 180.0);
}
