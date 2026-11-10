#ifndef ARBITER_MODE_H
#define ARBITER_MODE_H

/**
 * Which source an arbiter listens to. Each arbiter has its own mode, so the
 * rudder and the tail can be in different arbiter modes. Commander works this
 * out from its SYSTEM_STATE_* bits (sharedCommand.h).
 */
typedef enum {
    ARBITER_MODE_MANUAL, /**< Follow manual requests. */
    ARBITER_MODE_AUTO,   /**< Follow PID requests. */
    ARBITER_MODE_SAFE,   /**< Ignore all requests and hold the safe position. */
    /**
     * Follow PID requests, with the PID target set from the ground station
     * (COMMAND_SAIL_PID / COMMAND_RUDDER_PID) instead of the planner.
     * The arbiters treat it the same as AUTO.
     */
    ARBITER_MODE_SEMI,
} ArbiterMode_e;

#endif // ARBITER_MODE_H
