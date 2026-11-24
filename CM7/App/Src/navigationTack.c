/////////////////////////////////////////////////////////////////////////////////////////
// Live Tack Planning step. No RTOS calls, never include system.h.
//
// Sides: -1 = wind from port, +1 = wind from starboard. The upwind heading for a side is
// twd - side * NAV_NO_GO_DEG, where twd is the averaged wind direction ("from", deg true).
/////////////////////////////////////////////////////////////////////////////////////////

#include "navigationTack.h"
#include "navigationGeo.h"
#include <math.h>
#include "sharedSensor.h"

/////////////////////////////////////////////////////////////////////////////////////////

static double upwind_heading(double twd, int8_t side)
{
    return navGeo_wrap360(twd - side * NAV_NO_GO_DEG);
}

// Which tack reaches a waypoint at this bearing relative to the wind
static int8_t side_of_wp(double rel_wp)
{
    return (rel_wp > 0.0) ? -1 : 1;
}

static void set_upwind(NavTackState_t *state, int8_t side)
{
    state->tack_side = side;
    state->nav_state = (side < 0) ? NAV_STATE_UPWIND_PORT : NAV_STATE_UPWIND_STBD;
}

static void start_tack(const NavTackInput_t *in, NavTackState_t *state, int8_t side)
{
    state->tack_side = side;
    state->tack_ms = in->now_ms;
    state->nav_state = NAV_STATE_TACKING;
}

// IDLE, FAULT and ARRIVED publish no heading and are on neither tack
static void stop(NavTackState_t *state, uint8_t nav_state)
{
    state->nav_state = nav_state;
    state->tack_side = 0;
}

static bool tack_allowed(const NavTackInput_t *in, const NavTackState_t *state)
{
    return (uint32_t)(in->now_ms - state->tack_ms) >= NAV_TACK_MIN_INTERVAL_MS;
}

// Near the edge of the corridor, on a heading that takes the boat further out
static bool past_tack_trigger(const NavTackLeg_t *leg, double heading)
{
    if (fabs(leg->xte) < NAV_TACK_XTE_FRACTION * leg->margin) { return false; }
    return leg->xte * navGeo_wrap180(heading - leg->leg_brg) > 0.0;
}

// The age is read as signed, so it survives the tick wrapping and a sample taken just after now_ms
static bool fresh(uint32_t now_ms, uint32_t timestamp, uint32_t stale_ms)
{
    return (int32_t)(now_ms - timestamp) <= (int32_t)stale_ms;
}

/////////////////////////////////////////////////////////////////////////////////////////

NAV_TACK_HELPER bool inputs_ok(const NavTackInput_t *in)
{
    bool gps_valid = (in->gps_valid == VALIDITY_VALID || in->gps_valid == VALIDITY_VALID_PARTIAL_DATA);
    if (!gps_valid || in->imu_valid != VALIDITY_VALID || in->wind_valid != VALIDITY_VALID) { return false; }
    // Fusion reports 0, 0 until its first position, even with the GPS already valid
    if (in->lat == 0.0 && in->lon == 0.0) { return false; }

    return fresh(in->now_ms, in->gps_timestamp, NAV_GPS_STALE_MS) &&
           fresh(in->now_ms, in->imu_timestamp, NAV_IMU_STALE_MS) &&
           fresh(in->now_ms, in->wind_timestamp, NAV_WIND_STALE_MS);
}

// Apparent wind = true wind - boat velocity. With the true wind's speed assumed to be W, that gives
// the true wind direction. Returns the apparent direction when W is 0 or the boat's velocity is unusable.
NAV_TACK_HELPER double wind_estimate(const NavTackInput_t *in, double W)
{
    double awd = navGeo_wrap360(in->heading + in->ang_wind);          // apparent, "from"

    if (W <= 0.0 || !in->vel_valid || in->sog_mps < NAV_SOG_MIN_MPS || in->cog_deg >= 360u) { return awd; }

    double to = (awd + 180.0) * NAV_DEG2RAD, c = in->cog_deg * NAV_DEG2RAD;
    double ux = sin(to), uy = cos(to);                                 // air direction (east, north)
    double vx = in->sog_mps * sin(c), vy = in->sog_mps * cos(c);       // boat velocity
    double uv = ux * vx + uy * vy, d = uv * uv - (vx * vx + vy * vy) + W * W;

    if (d < 0.0) { return awd; }                                       // assumed wind too light: give up
    double A = -uv + sqrt(d);                                          // apparent wind speed
    double tx = A * ux + vx, ty = A * uy + vy;                         // true air velocity
    return navGeo_wrap360(atan2(tx, ty) * NAV_RAD2DEG + 180.0);
}

NAV_TACK_HELPER double wind_from(const NavTackInput_t *in)
{
    return wind_estimate(in, NAV_TRUE_WIND_ASSUMED_MPS);
}

// Adds this cycle's sample and returns the averaged wind direction. No sample while TACKING,
// because the vane lags the turn.
NAV_TACK_HELPER double wind_update(const NavTackInput_t *in, NavTackState_t *state, bool ok)
{
    if (ok && state->nav_state != NAV_STATE_TACKING)
    {
        if (state->wind_next >= NAV_WIND_WINDOW) { state->wind_next = 0; }
        state->wind[state->wind_next] = wind_from(in);
        state->wind_next = (uint8_t)((state->wind_next + 1u) % NAV_WIND_WINDOW);
        if (state->wind_count < NAV_WIND_WINDOW) { state->wind_count++; }
    }
    return navGeo_circularMean(state->wind, state->wind_count);
}

NAV_TACK_HELPER NavTackLeg_t leg_geometry(const NavTackInput_t *in, const NavTackState_t *state, double twd)
{
    const NavWaypoint_t *wp = &in->route->wp[state->leg];
    NavLocal_t s = {0.0, 0.0};
    NavLocal_t e = navGeo_toLocal(state->leg_start_lat, state->leg_start_lon, wp->lat, wp->lon);
    NavLocal_t b = navGeo_toLocal(state->leg_start_lat, state->leg_start_lon, in->lat, in->lon);

    NavTackLeg_t leg;
    leg.xte     = navGeo_crossTrack(s, e, b);
    leg.along   = navGeo_alongTrack(s, e, b);
    leg.leg_len = navGeo_distance(s, e);
    leg.leg_brg = navGeo_bearing(s, e);
    leg.brg_wp  = navGeo_bearing(b, e);
    leg.rel_wp  = navGeo_wrap180(leg.brg_wp - twd);
    leg.dist_wp = navGeo_distance(b, e);
    leg.margin  = wp->margin_m;
    return leg;
}

// Along the leg, angled back toward the line by up to NAV_RETURN_ANGLE_DEG at the margin
NAV_TACK_HELPER double direct_heading(const NavTackLeg_t *leg, double twd)
{
    double ratio = (leg->margin > 0.0) ? leg->xte / leg->margin : 0.0;
    if (ratio > 1.0)  { ratio = 1.0; }
    if (ratio < -1.0) { ratio = -1.0; }

    double heading = navGeo_wrap360(leg->leg_brg - NAV_RETURN_ANGLE_DEG * ratio);

    // Never steer into the no-go zone: stop at its edge on the waypoint's side
    if (fabs(navGeo_wrap180(heading - twd)) < NAV_NO_GO_DEG) { heading = upwind_heading(twd, side_of_wp(leg->rel_wp)); }
    return heading;
}

// From DIRECT, or right after a new leg
NAV_TACK_HELPER double direct_step(const NavTackInput_t *in, NavTackState_t *state, const NavTackLeg_t *leg, double twd)
{
    if (fabs(leg->rel_wp) >= NAV_NO_GO_DEG)
    {
        state->nav_state = NAV_STATE_DIRECT;
        state->tack_side = 0;
        return direct_heading(leg, twd);
    }

    // The waypoint is upwind. Start on the tack closer to the current heading, so no turn through the wind
    double off_port = fabs(navGeo_wrap180(in->heading - upwind_heading(twd, -1)));
    double off_stbd = fabs(navGeo_wrap180(in->heading - upwind_heading(twd, 1)));
    int8_t side = (off_port <= off_stbd) ? -1 : 1;

    // Unless that tack heads out of a corridor the boat is already at the edge of: then tack now
    if (past_tack_trigger(leg, upwind_heading(twd, side)))
    {
        start_tack(in, state, (int8_t)-side);
        return upwind_heading(twd, state->tack_side);
    }

    set_upwind(state, side);
    return upwind_heading(twd, side);
}

// From UPWIND_PORT or UPWIND_STBD
NAV_TACK_HELPER double upwind_step(const NavTackInput_t *in, NavTackState_t *state, const NavTackLeg_t *leg, double twd)
{
    // Layline: the waypoint can be sailed to directly
    if (fabs(leg->rel_wp) >= NAV_NO_GO_DEG + NAV_NO_GO_HYST_DEG)
    {
        int8_t side_wp = side_of_wp(leg->rel_wp);
        if (side_wp == state->tack_side)
        {
            // Bear away on the same tack
            state->nav_state = NAV_STATE_DIRECT;
            state->tack_side = 0;
            return direct_heading(leg, twd);
        }
        if (tack_allowed(in, state))
        {
            // The waypoint is on the other tack. DIRECT follows once the tack is done
            start_tack(in, state, side_wp);
            return upwind_heading(twd, state->tack_side);
        }
    }

    // Corridor: tack back before the edge
    if (past_tack_trigger(leg, in->heading) && tack_allowed(in, state))
    {
        start_tack(in, state, (int8_t)-state->tack_side);
        return upwind_heading(twd, state->tack_side);
    }

    set_upwind(state, state->tack_side);
    return upwind_heading(twd, state->tack_side);
}

// Hold the new tack's heading until the boat gets there
NAV_TACK_HELPER double tacking_step(const NavTackInput_t *in, NavTackState_t *state, double twd, uint8_t *flags)
{
    double target = upwind_heading(twd, state->tack_side);

    if (fabs(navGeo_wrap180(in->heading - target)) <= NAV_TACK_DONE_DEG)
    {
        set_upwind(state, state->tack_side);
    }
    else if ((uint32_t)(in->now_ms - state->tack_ms) > NAV_TACK_SLOW_MS)
    {
        *flags |= NAV_FLAG_TACK_SLOW;
    }
    return target;
}

/////////////////////////////////////////////////////////////////////////////////////////

static int16_t drift_m(double xte)
{
    if (xte > INT16_MAX) { return INT16_MAX; }
    if (xte < INT16_MIN) { return INT16_MIN; }
    return (int16_t)lround(xte);
}

// One cycle. Leaves the result in state, and in out for everything except nav_state and tack_side.
static void plan(const NavTackInput_t *in, NavTackState_t *state, TelemetryNavigation_t *out)
{
    if (!in->enabled)
    {
        // The wind is not sampled while disabled, so what is stored would be stale on the way back in
        state->wind_count = 0;
        state->wind_next = 0;
        stop(state, NAV_STATE_IDLE);
        return;
    }

    bool ok = inputs_ok(in);
    double twd = wind_update(in, state, ok);

    const NavRoute_t *route = in->route;
    if (in->route_version == 0 || route == NULL || route->count == 0)
    {
        stop(state, NAV_STATE_IDLE);
        return;
    }

    if (in->route_version != state->route_version)
    {
        // The first leg starts at the boat, so wait for a good position. Tried again next cycle
        if (!ok) { stop(state, NAV_STATE_FAULT); return; }

        state->route_version = in->route_version;
        state->leg = 0;
        state->leg_start_lat = in->lat;
        state->leg_start_lon = in->lon;
        state->corridor_fault = false;
        state->tack_side = 0;
        state->tack_ms = in->now_ms - NAV_TACK_MIN_INTERVAL_MS; // a first tack is allowed right away
        state->nav_state = NAV_STATE_DIRECT;
    }

    // Neither of these latches: planning picks up again when the inputs come back
    if (!ok || state->wind_count < NAV_WIND_MIN_SAMPLES) { stop(state, NAV_STATE_FAULT); return; }

    if (state->corridor_fault) { stop(state, NAV_STATE_FAULT); return; }       // holds until a new route
    if (state->leg >= route->count) { stop(state, NAV_STATE_ARRIVED); return; }

    // Coming back from IDLE or FAULT: start over from DIRECT, which picks the tack again if needed
    bool sailing = (state->nav_state == NAV_STATE_DIRECT) ||
                   ((state->nav_state == NAV_STATE_UPWIND_PORT || state->nav_state == NAV_STATE_UPWIND_STBD ||
                     state->nav_state == NAV_STATE_TACKING) && state->tack_side != 0);
    if (!sailing)
    {
        state->nav_state = NAV_STATE_DIRECT;
        state->tack_side = 0;
    }

    // Arrival, then the same checks on the next leg
    NavTackLeg_t leg;
    for (;;)
    {
        leg = leg_geometry(in, state, twd);

        bool reached = (leg.dist_wp <= route->arrive_m) ||
                       (leg.along >= leg.leg_len && fabs(leg.xte) <= leg.margin);
        if (!reached) { break; }

        state->leg_start_lat = route->wp[state->leg].lat;
        state->leg_start_lon = route->wp[state->leg].lon;
        state->leg++;
        state->tack_side = 0;
        state->nav_state = NAV_STATE_DIRECT;

        if (state->leg >= route->count) { stop(state, NAV_STATE_ARRIVED); return; }
    }

    out->route_segment = state->leg;
    out->drift_distance = drift_m(leg.xte);

    if (fabs(leg.xte) > leg.margin + route->margin_warning_m)
    {
        state->corridor_fault = true;
        stop(state, NAV_STATE_FAULT);
        return;
    }

    double heading;
    switch (state->nav_state)
    {
        case NAV_STATE_TACKING:     heading = tacking_step(in, state, twd, &out->flags); break;
        case NAV_STATE_UPWIND_PORT:
        case NAV_STATE_UPWIND_STBD: heading = upwind_step(in, state, &leg, twd); break;
        default:                    heading = direct_step(in, state, &leg, twd); break;
    }

    out->desired_heading = (uint16_t)(lround(navGeo_wrap360(heading)) % 360);
}

void navTack_step(const NavTackInput_t *in, NavTackState_t *state, TelemetryNavigation_t *out)
{
    out->desired_heading = NAV_HEADING_NONE;
    out->route_segment = NAV_SEGMENT_NONE;
    out->drift_distance = 0;
    out->flags = 0;
    out->timestamp = in->now_ms;
    out->valid = VALIDITY_VALID;

    plan(in, state, out);

    out->nav_state = state->nav_state;
    out->tack_side = state->tack_side;
    if (state->corridor_fault) { out->flags |= NAV_FLAG_CORRIDOR; }
}
