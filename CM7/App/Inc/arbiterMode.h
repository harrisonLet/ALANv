#ifndef ARBITER_MODE_H
#define ARBITER_MODE_H

/**
 * Which source an arbiter listens to. Each arbiter has its own mode, so one
 * system mode can put the rudder and the tail in different arbiter modes.
 * Commander converts system_mode_t (sharedCommand.h) into this. The numbers
 * do not match system_mode_t, so never cast between the two.
 */
typedef enum {
    ARBITER_MODE_MANUAL, /**< Follow manual requests. */
    ARBITER_MODE_AUTO,   /**< Follow PID requests. */
    ARBITER_MODE_SAFE,   /**< Ignore all requests and hold the safe position. */
} ArbiterMode_e;

#endif // ARBITER_MODE_H
