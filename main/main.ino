/* HEADERS */
#include "sensors.h"
#include "SimpleKalmanFilter.h"
#include <ESP32Servo.h>
#include "tvc.h"
#include "groundControl.h"

/* DATA STRUCTURES */
// `State` represents all states of the flight and has an additional "Boot" and "Error" state
enum class State
{

    Boot,
    Ready,
    PreLaunch,
    Flight,
    PoweredLanding,
    Landed,

    Error,

} STATE;

GroundControl groundControl("https://spaceprogram.bolls.dev");

bool connectToGroundControl()
{
    Serial.println("Setting up GroundControl...");
    groundControl.connect();
    Serial.println("Connected to GroundControl");
    return true;
}

bool receiveCommand()
{
    Serial.println("Checking for commands...");
    bool commandReceived = false;

    groundControl.subscribe([&commandReceived](const String &command)
                            {
        Serial.println("Received command: " + command);
        if (command == "start") {
            commandReceived = true;
        } });

    return commandReceived;
}

/* SETUP

    This function only runs once when the flight computer is turned on;
    Here we can initialize all the sensors

*/
void setup()
{
    Serial.begin(9600);
    // initialize all sensors
    connectWifi();
    setup_sensors();
    initializeTVC();
    connectToGroundControl();

    // initial state
    STATE = State::Boot;
    Serial.println("Setup completed!");
}

/* LOOP

    This funtion will run for the whole duration of the flight

*/
void loop()
{

    unsigned long currentTime = millis();

    // update sensors every updateinterval milliseconds
    if (currentTime - lastUpdateTime >= updateInterval)
    {
        update_sensors();
        lastUpdateTime = currentTime;
    }

    float pitch = estimated_pitch;
    float yaw = estimated_yaw;
    // switch case function for all the flight case scenarios

    switch (STATE)
    {

    case State::Boot:
        STATE = State::Ready;
        Serial.println("Boot Completed! Entering `Ready` state.");
        break;

    case State::Ready:
        // check if all systems are go and we are ready to transition to `PreLaunch check`
        if (allSystemsGo())
        {
            STATE = State::PreLaunch;
            Serial.println("All systems are go, transitioning into `PreLaunch` state");
        };
        break;

    case State::PreLaunch:
        // All systems are go at this point, waiting for manual confirmation
        if (receiveCommand())
        {
            Serial.println("Manual Confirmation Received, We are go for launch, initiating countdown");
            delay(10000); // countdown for 10 seconds

            /* We should probably add some fall back here in case something goes wrong during the countdown to be able to abort */
            motorIgnite();
            if (datapoint.estimated_altitude > 1.0)
            {
                STATE = State::Flight;
            };
        }
        break;

    case State::Flight:
        // flight mode, control the TVC
        controlTVC(pitch, yaw);
        Serial.println("Rocket in flight");

        // check if the rocket is descending to enter the `PoweredLanding` state
        if (datapoint.estimated_altitude < 1.0)
        {
        }
        break;

    case State::PoweredLanding:
        controlTVC(pitch, yaw);

        // logic for the flight computer to calculate when to start the landing burn
        if (datapoint.estimated_altitude < 1.0)
        {
            STATE = State::Landed;
        };

        break;

    case State::Landed:
        // final logging
        Serial.println("Flight complete. Rocket landed successfully. ");
        break;

    case State::Error:
        // handle error
        break;
    }
}
