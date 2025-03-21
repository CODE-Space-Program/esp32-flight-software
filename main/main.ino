#include "sensors.h"
#include "SimpleKalmanFilter.h"
#include <ESP32Servo.h>
#include "tvc.h"
#include "groundControl.h"
#include "tvcTest.h"
#include <ArduinoJson.h>

// `State` represents all states of the flight and has an additional "Boot" and "Error" state

enum class State
{

    Boot, // initial state when the computer is turned on, switches to `Ready` after the `setup()` function is done
    Ready, // blocks proceeding to the next step until the sensors send correct values
    PreLaunch, // ready to receive the start command from ground control, switches to `Flight` once the start command is received
    Flight,
    PoweredLanding,
    Landed,

    Error,

} STATE;

/**
 * string representation of the state to be sent to the ground control
 */
const char *stateStrings[] = {
    "Boot",
    "Ready",
    "PreLaunch",
    "Flight",
    "PoweredLanding",
    "Landed",
    "Error"};

GroundControl groundControl("https://spaceprogram.bolls.dev");
Servos servos(-65, 65);
Tvc tvc(servos, 0, 1);
TvcTest tvcTest;

void sendTelemetryTask(void *parameter)
{
    Serial.println("Telemetry is working on core...");
    Serial.println(xPortGetCoreID());
    while (true)
    {
        datapoint.state = stateStrings[static_cast<int>(STATE)];
        datapoint.nominalYawServoDegrees = tvc.yaw;
        datapoint.nominalPitchServoDegrees = tvc.pitch;
    
        groundControl.sendTelemetry(datapoint);
        vTaskDelay(pdMS_TO_TICKS(100));
    }
}


/* SETUP
    This function only runs once when the flight computer is turned on;
    Here we can initialize all the sensors
*/
void setup()
{
    Serial.begin(9600);

    tvc.initialize();    
    
    connectWifi();
    setup_sensors();
    pyroInit();
    calibrateMpu6050();

    groundControl.connect();
    groundControl.subscribe([](const String &command, const JsonVariant &args) {
        Serial.println("received command from ground control: " + command);

        serializeJsonPretty(args, Serial);
        Serial.println();

        if (command == "start")
        {
            Serial.println("Received start command");
            
            STATE = State::Flight;

            ascendingMotorIgnite();
        }
        if (command == "test_tvc")
        {
            Serial.println("Starting TVC test");

            if (args.isNull())
            {
                tvcTest.start(20.0, 10.0);
            }
            else
            {
                serializeJsonPretty(args, Serial);
                Serial.println();

                float maxDegrees = args["maxDegrees"] | 20.0;
                float stepDegrees = args["stepDegrees"] | 10.0;

                tvcTest.start(maxDegrees, stepDegrees);
            }
        }
        if (command == "zero_tvc")
        {
            Serial.println("Zeroing TVC");
            tvc.moveRaw(0, 0);
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

        tvc.moveRaw(newPitch, newYaw);

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
            tvc.move(pitch, yaw);

            STATE = State::PreLaunch;
            Serial.println("All systems are go, transitioning into `PreLaunch` state");
        };
        break;

    case State::PreLaunch:
        // All systems are go at this point, waiting for manual confirmation
        tvc.move(pitch, yaw);
        break;

    case State::Flight:
        // flight mode, control the TVC
        tvc.move(pitch, yaw);
        Serial.println("Rocket in flight");

        // check if the rocket is descending to enter the `PoweredLanding` state
        if (datapoint.estimated_altitude > 200)
        {
            STATE = State::PoweredLanding;
            Serial.println("Rocket is descending, entering `PoweredLanding` state");
        }
        break;

    case State::PoweredLanding:
        tvc.move(pitch, yaw);

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
        break;
    }
}
