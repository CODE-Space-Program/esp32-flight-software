/* HEADERS */
#include "sensors.h"
#include "KalmanFilter.h"
#include <ESP32Servo.h>
#include "tvc.h"


/* DATA STRUCTURES */
// `State` represents all states of the flight and has an additional "Boot" and "Error" state
enum class State {

    Boot,
    Ready,
    Flight,
    Chute,
    Land,

    Error,

} STATE;

/* SETUP  

    This function only runs once when the flight computer is turned on;
    Here we can initialize all the sensors 

*/
void setup() {
    Serial.begin(9600);

    // initialize all sensors and the kalman filter
    setup_sensors();
    initializeTVC();

    // initial state
    STATE = State::Boot;
    Serial.println("Setup completed!");
}


/* LOOP

    This funtion will run for the whole duration of the flight

*/
void loop() {

    unsigned long currentTime = millis();

    // update sensors every updateinterval milliseconds
    if (currentTime - lastUpdateTime >= updateInterval) {
        update_sensors();
        lastUpdateTime = currentTime;
    }

    float pitch = estimated_pitch;
    float yaw = estimated_yaw;

    // switch case function for all the flight case scenarios

    switch (STATE)  {

        case State::Boot:
            STATE = State::Ready;
            Serial.println("Ready for liftoff! Entering `Ready` state.");
            break;
    
        case State::Ready:
            // check for launch condition e.g. significant height increase
            if (kalman_filter.get_estimated_height() > 5.0) { // threshold for launch detection
                STATE = State::Flight;
                Serial.println("Liftoff detected! Entering `Flight` state. ");
            }
            break;

        case State::Flight:
            // upadate Kalman filter and control TVC
            controlTVC(pitch, yaw);

            // check if rocket is descending to deploy chute
            if (kalman_filter.get_estimated_velocity() < -5.0) { // threshold descent velocity
                STATE = State::Chute;
                Serial.println("Descent detected! Deploying chute. Entering `Chute` state.");
            }
            break;
    
        case State::Chute:
            // logic for deploying the chute 
            if (kalman_filter.get_estimated_height() < 1.0) {
                STATE = State::Land;
                Serial.println("Landed! Entering `Land` state.");
            }
            break;
    
        case State::Land:
            // final logging
            Serial.println("Flight complete. Rocket landed successfully. ");
            break;

        case State::Error:
            // handle error 
            break;
        }
}