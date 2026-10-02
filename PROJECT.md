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

## PROCESS: PID Sail

* Core: CM7
* Inputs:
    * Wind Sensor Data
    * Encoder Data
* Outputs:
    * Sail Servo Position

## PROCESS: PID Heading

* Core: CM7
* Inputs:
    * Vector Heading
    * Magnetometer Data
* Outputs:
    * Rudder Servo Position

## PROCESS: Telemetry Save

* Core: CM7
* Inputs:
    * Wind Sensor Data
    * GPS Sensor Data
    * Magnetometer Data
    * Encoder Data
* Outputs:
    * Saved Telemetry Data

## PROCESS: LoRa Communication

* Core: CM4
* Inputs:
    * Vector Series
    * Margin Series
* Outputs:
    * Saved Telemetry Data

## INTERUPTIVES: Tack

* Core: CM7
* Inputs:
* Outputs:
    * Rudder Servo Position
    * Sail Servo Position