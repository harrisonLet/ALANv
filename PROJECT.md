# AUTONOMOUS SAILBOAT

## PROCESS: Sensor Pulling

* Core: CM7
* Inputs: None
* Outputs:
    * Wind Sensor Data
    * GPS Sensor Data
    * Magnetometer Data
    * Encoder Data

## PROCESS: Route Planning

* Core: Local
* Inputs:
    * Point A, starting point
    * Point B, destination point
* Outputs:
    * Vector Series, a series of vectors representing the path segments the boat needs to follow in order to reach the destination
    * Margin Series, a series of margins representing the allowable deviation from the planned path for each path segment

## PROCESS: Live Tack Planning (LTP)

* Core: CM7
* Inputs:
    * Wind Sensor Data
    * GPS Sensor Data
    * Vector Series
    * Margin Series
* Outputs:
    * Vector Heading
    * Vector Sail, relative to the vector heading

## PROCESS: PID Sail

* Core: CM7
* Inputs:
    * Vector Sail
    * Encoder Data
* Outputs:
    * Sail Servo Position

Given the current optimal angle of attack for the sail and the current angle of the sail relative to the boat, we need to adjust the position of the tail sail so that the sail achieves the optimal angle of attack.

## PROCESS: PID Heading

* Core: CM7
* Inputs:
    * Vector Heading
    * Magnetometer Data
* Outputs:
    * Rudder Servo Position

Given the current optimal heading for the boat and the current heading of the boat, we need to adjust the position of the rudder so that the boat achieves the optimal heading.

## PROCESS: Telemetry Save

* Core: CM7
* Inputs:
    * Wind Sensor Data
    * GPS Sensor Data
    * Magnetometer Data
    * Encoder Data
* Outputs:
    * Saved Telemetry Data
    * New Telemetry data flag

At some regular but infrequient interval, the telemetry data from the sensors is saved and the new telemetry data flag is updated so that LoRa will send it out.

## PROCESS: LoRa Communication In

* Core: CM4
* Inputs:
    * Ground Station Commands
    * Vector Series
    * Margin Series
* Outputs:
    * Commands to CM7

## PROCESS: LoRa Communication Out

* Core: CM4
* Inputs:
    * Saved Telemetry Data
    * New Telemetry data flag
* Outputs:
    * LoRa Saved Telemetry Data

## PROCESS: Ground Station

* Core: Ground Station
* Inputs:
    * Saved Telemetry Data from LoRa Communication
* Outputs:
    * Commands to LoRa Communication

The ground station needs to have an effective interface to monitor the telemetry data from the sailboat and send appropriate commands back to it.

## INTERUPTIVE: Tack

* Core: CM7
* Inputs:
    * Tack signal from LTP
* Outputs:
    * Rudder Servo Position
    * Sail Servo Position

## INTERUPTIVE: Corse Recovery

* Core: CM7
* Inputs:
    * Corse Recovery signal from LTP
* Outputs:
    * Vector Heading
    * Vector Sail