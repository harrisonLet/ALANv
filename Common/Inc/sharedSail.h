/////////////////////////////////////////////////////////////////////////////
// Main Sail

/**
 * Represents the current position and rotation of the sail relative to the vessel. The 
 * actual measurement represents the direction which the sail is pointing.
 * - Maximum Operational Range: -180 to 180 degrees
 * - Safe Operational Range: -90 to 90 degrees
 *
 * SENSOR: Sail Encoder
 */
typedef struct {

} TelemetryMainSail_t;



/////////////////////////////////////////////////////////////////////////////
// Tail Sail

/**
 * Represents the current position and rotation of the tail sail relative to the main sail.
 * - Maximum Operational Range: -90 to 90 degrees
 */
typedef struct {

} TelemetryTailSail_t;
