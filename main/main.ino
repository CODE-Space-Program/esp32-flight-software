/* HEADERS */
#include "sensors.h"
#include "SimpleKalmanFilter.h"
#include <ESP32Servo.h>
#include "tvc.h"
#include "groundControl.h"
#include "tvcTest.h"
#include <ArduinoJson.h>

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

/* SETUP

    This function only runs once when the flight computer is turned on;
    Here we can initialize all the sensors

*/
bool commandReceived = false;

TvcTest tvcTest;

void sendTelemetryTask(void *parameter)
{
    Serial.println("Telemetry is working on core...");
    Serial.println(xPortGetCoreID());
    while (true)
    {
        groundControl.sendTelemetry(datapoint);
        vTaskDelay(pdMS_TO_TICKS(100));
    }
}

void setup()
{
    Serial.begin(9600);
    // initialize all sensors
    connectWifi();
    setup_sensors();
    initializeTVC();
    pyroInit();
    calibrateMpu6050();

    groundControl.connect();                                                     // Connect first
    groundControl.subscribe([](const String &command, const JsonVariant &args) { // Subscribe after connection
        Serial.println("received command from ground control: " + command);

        serializeJsonPretty(args, Serial);
        Serial.println();

        if (command == "start")
        {
            Serial.println("Received start command");

            commandReceived = true;
        }
        if (command == "test_tvc")
        {
            Serial.println("Starting TVC test");

            if (args.isNull())
            {
                Serial.println("args are null");

                tvcTest.start(20.0, 10.0);
            }
            else
            {
                Serial.println("args are not null");

                serializeJsonPretty(args, Serial);
                Serial.println();

                float maxDegrees = args["maxDegrees"] | 20.0;
                float stepDegrees = args["stepDegrees"] | 10.0;

                tvcTest.start(maxDegrees, stepDegrees);
            }
        }
    });

    xTaskCreatePinnedToCore(
        sendTelemetryTask,   // Task function
        "SendTelemetryTask", // Task name
        8192,                // Stack size
        NULL,                // Task parameters
        1,                   // Task priority
        NULL,                // Task handle
        0                    // Core ID (0 = first core, 1 = second core)
    );

    // initial state
    STATE = State::Boot;
    Serial.println("Setup working on core...");
    Serial.println(xPortGetCoreID());
    Serial.println("Setup completed!");
}

/* LOOP

    This funtion will run for the whole duration of the flight

*/
void loop()
{
    if (tvcTest.isInProgress())
    {
        float newPitch = tvcTest.getNewPitch();
        float newYaw = tvcTest.getNewYaw();

        Serial.println("[TVC Test]: New pitch: " + String(newPitch) + ", New yaw: " + String(newYaw));

        moveServos(newPitch, newYaw);

        return;
    }

    unsigned long currentTime = millis();

    // update sensors every updateinterval milliseconds
    if (currentTime - lastUpdateTime >= updateInterval)
    {
        update_sensors();
        lastUpdateTime = currentTime;
    }

    float pitch = gx;
    float yaw = gz;
    // switch case function for all the flight case scenarios

    switch (STATE)
    {

    case State::Boot:
        STATE = State::Ready;
        Serial.println("Boot Completed! Entering `Ready` state.");
        break;

    case State::Ready:
        // check if all systems are go and we are ready to transition to `PreLaunch check`
        if (allSystemsCheck())
        {
            controlTVC(pitch, yaw);
            STATE = State::PreLaunch;
            Serial.println("All systems are go, transitioning into `PreLaunch` state");
        };
        break;

    case State::PreLaunch:
        // All systems are go at this point, waiting for manual confirmation
        controlTVC(pitch, yaw);
        if (commandReceived)
        {
            Serial.println("Manual Confirmation Received, We are go for launch, initiating countdown");
            controlTVC(pitch, yaw);
            ascendingMotorIgnite();
            if (datapoint.estimated_altitude > 0.3)
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
        if (datapoint.estimated_altitude > 200)
        {
            STATE = State::PoweredLanding;
            Serial.println("Rocket is descending, entering `PoweredLanding` state");
        }
        break;

    case State::PoweredLanding:
        controlTVC(pitch, yaw);

        // logic for the flight computer to calculate when to start the landing burn
        if (datapoint.estimated_altitude < 200)
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
