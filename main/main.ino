/* HEADERS */
#include "sensors.h"
#include "SimpleKalmanFilter.h"
#include <ESP32Servo.h>
#include "tvc.h"
#include "groundControl.h"
#include "PIDController.h"

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

// Global declaration of PID controllers
PIDController pitchPID(1.0, 0.1, 0.01, 0.0); // Example coefficients for pitch
PIDController yawPID(1.0, 0.1, 0.01, 0.0); // Example coefficients for yaw

/* SETUP

    This function only runs once when the flight computer is turned on;
    Here we can initialize all the sensors

*/
bool commandReceived = false;

void sendTelemetryTask(void *parameter) {
    Serial.println("Telemetry is working on core...");
    Serial.println(xPortGetCoreID());
    while (true) {
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

    groundControl.connect(); // Connect first
    groundControl.subscribe([](const String &command) { // Subscribe after connection
        Serial.println("Received command: " + command);
        if (command == "start") {
            commandReceived = true;
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

    This function will run for the whole duration of the flight

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

    // Compute PID outputs
    float pitchControlOutput = pitchPID.compute(pitch);
    float yawControlOutput = yawPID.compute(yaw);

    // Use PID outputs to control TVC
    controlTVC(pitchControlOutput, yawControlOutput);

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
            controlTVC(pitchControlOutput, yawControlOutput);
            STATE = State::PreLaunch;
            Serial.println("All systems are go, transitioning into `PreLaunch` state");
        };
        break;

    case State::PreLaunch:
        // All systems are go at this point, waiting for manual confirmation
        controlTVC(pitchControlOutput, yawControlOutput);
        if (commandReceived)
        {
            Serial.println("Manual Confirmation Received, We are go for launch, initiating countdown");
            controlTVC(pitchControlOutput, yawControlOutput);
            ascendingMotorIgnite();
            if (datapoint.estimated_altitude > 0.3)
            {
                STATE = State::Flight;
            };
        }
        break;

    case State::Flight:
        // flight mode, control the TVC
        controlTVC(pitchControlOutput, yawControlOutput);
        Serial.println("Rocket in flight");

        // check if the rocket is descending to enter the `PoweredLanding` state
        if (datapoint.estimated_altitude > 200) {
            STATE = State::PoweredLanding;
            Serial.println("Rocket is descending, entering `PoweredLanding` state");
        }
        break;

    case State::PoweredLanding:
        controlTVC(pitchControlOutput, yawControlOutput);

        // logic for the flight computer to calculate when to start the landing burn
        if (datapoint.estimated_altitude < 200)
        {
            STATE = State::Landed;
        };

        break;

    case State::Landed:
        // final logging
        Serial.println("Flight complete. Rocket landed successfully.");
        break;

    case State::Error:
        // handle error
        break;
    }
}
