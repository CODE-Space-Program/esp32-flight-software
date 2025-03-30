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
// The PCA9685 uses 12-bit resolution (0-4095) to control servos via pulse width modulation.
// The servos expect 500µs to 2400µs pulses at 50Hz (20ms period).
// The PCA9685 generates 4096 steps per 20ms, so 4.88µs per step
// 500µs/4.88µs = 102.5 - rounded to 110 to avoid damaging servos
// 2400/4.88µs = 491.8 - rounded to 480
Servos servos(110, 480);
Tvc tvc(servos, 0, 1);
TvcTest tvcTest;

bool CONTROL_TVC = false;

void sendTelemetryTask(void *parameter)
{
    Serial.println("Telemetry is working on core...");
    Serial.println(xPortGetCoreID());
    while (true)
    {
        datapoint.state = stateStrings[static_cast<int>(STATE)];
        datapoint.nominalYawServoDegrees = tvc.yaw;
        datapoint.nominalPitchServoDegrees = tvc.pitch;
        datapoint.servosLocked = tvc.servosLocked;
    
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

            tvc.initialize();
            
            STATE = State::Flight;

            ascendingMotorIgnite();
        }
        if (command == "test_tvc")
        {
            Serial.println("Starting TVC test");

            tvc.initialize();

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
                int duration = args["duration"] | 5000;

                tvcTest.start(maxDegrees, stepDegrees, duration);
            }
        }
        if (command == "zero_tvc")
        {
            Serial.println("Zeroing TVC");
            tvc.initialize();
            tvc.moveRaw(90, 90);
            tvc.uninitialize();
        }
        if (command == "disconnect_wifi") {
            disconnectWifi();
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

int lastTime = 0;

/* LOOP

    This funtion will run for the whole duration of the flight

*/
void loop()
{
    if (tvcTest.isInProgress())
    {
        lastTime++;
        if (lastTime % 100 == 0) {

            float newPitch = tvcTest.getNewPitch();
            float newYaw = tvcTest.getNewYaw();

            Serial.println("[TVC Test]: New pitch: " + String(newPitch) + ", New yaw: " + String(newYaw));

            tvc.moveRaw(newPitch, newYaw);
        }
        return;
    } else if (STATE < State::Flight) {
        tvc.uninitialize();
    }

    update_sensors();

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
            if (CONTROL_TVC) {
                tvc.move(pitch+90, yaw+90);
            }
            STATE = State::PreLaunch;
            Serial.println("All systems are go, transitioning into `PreLaunch` state");
        };
        break;

    case State::PreLaunch:
        // All systems are go at this point, waiting for manual confirmation
        if (CONTROL_TVC) {
            tvc.move(pitch+90, yaw+90);
        }
        break;

    case State::Flight:
        // flight mode, control the TVC
        tvc.move(pitch+90, yaw+90);
        Serial.println("Rocket in flight, pitch & yaw are:");
        Serial.println(pitch);
        Serial.println(yaw);

        // check if the rocket is descending to enter the `PoweredLanding` state
        if (datapoint.estimated_altitude > 200)
        {
            STATE = State::PoweredLanding;
            Serial.println("Rocket is descending, entering `PoweredLanding` state");
        }
        break;

    case State::PoweredLanding:
        tvc.move(pitch+90, yaw+90);

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
