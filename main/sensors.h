#pragma once

/* include all the needed libraries */
#include <Adafruit_BMP3XX.h>
#include <Wire.h>
#include <SimpleKalmanFilter.h>
#include <Adafruit_MPU6050.h>
#include <WiFi.h>
#include <Arduino.h>
#include <vector>
#include <functional>

// #define IGNITION_PIN 5 // change this to the GPIO pin connected to the ESP32

// Wifi connection variables
const char *ssid = "CODE";
const char *password = "Code!University";

extern float estimated_pitch;
extern float estimated_yaw;

extern float estimated_height;
extern float estimated_velocity;

float estimated_height = 0;
float estimated_velocity = 0;
// constants
extern const float SEA_LEVEL_PRESSURE = 998.88; // example for sea level pressure in Berlin
extern const float AMBIENT_TEMPERATURE = 22.4;  // To be changed on the day
const float G = 9.81;                           // gravity constant
const unsigned long updateInterval = 100;       // Sensor update interval (ms)
unsigned long lastUpdateTime = 0;

// Data Structure for a single datapoint
struct Data
{

    long time; // time in ms

    struct Gyro
    {

        float x;
        float y;
        float z;

    } gyro; // Gyroscope data

    struct Acc
    {

        float x;
        float y;
        float z;

    } acc; // Accelerometer data

    float pressure;    // pressure in mbar
    float temperature; // Temperature in Celsius
    float altitude;
    float estimated_altitude; // Filtered height
    float velocity;

    char const null_terminator = 0; // Null terminator to avoid overflow

} datapoint;

/* Initialize sensor objects here */
Adafruit_MPU6050 mpu6050;
Adafruit_BMP3XX bmp;

/* Initialize kalman filter */
// SimpleKalmanFilter for height and velocity
SimpleKalmanFilter heightKalmanFilter(0.03, 1.0, 0.01); // tuning values: (e_mea, e_est, q)
SimpleKalmanFilter velocityKalmanFilter(0.03, 1.0, 0.01);

void setup_sensors()
{

    Wire.begin(21, 22);
    /* Initialize I2C Commnication */
    if (!mpu6050.begin())
    {
        Serial.println("Could not find a valid MPU6050 sensor, check wiring");
        while (1)
            ;
    }
    else
    {
        Serial.println("MPU6050 Initialized successfully");
    };

    /* Initialize BMP390 */
    while (!bmp.begin_I2C(0x76))
    {
        Serial.println("Could not find a valid BMP390 sensor, check wiring!");
        delay(1000);
    };
    Serial.println("BMP390 Initialized successfully");

    delay(100);

    // Recommended initialization by the manufacturer
    bmp.setTemperatureOversampling(BMP3_OVERSAMPLING_8X);
    bmp.setPressureOversampling(BMP3_OVERSAMPLING_4X);
    bmp.setIIRFilterCoeff(BMP3_IIR_FILTER_COEFF_3);
    bmp.setOutputDataRate(BMP3_ODR_100_HZ);

    /* Initialize MPU6050 */
    mpu6050.setAccelerometerRange(MPU6050_RANGE_8_G);
    mpu6050.setGyroRange(MPU6050_RANGE_500_DEG);
    mpu6050.setFilterBandwidth(MPU6050_BAND_44_HZ);
}

/* Print data from the Data struct to the SD card */
void print_data()
{

    /* code here */
}

float estimated_pitch = 0;
float estimated_yaw = 0;

/* Update sensor readings and log data */
void update_sensors()
{

    sensors_event_t a, g, temp;
    mpu6050.getEvent(&a, &g, &temp);

    // BMP390 sensor reading
    if (!bmp.performReading())
    {
        Serial.println("Failed to read from the BMP390 sensor!");
        return;
    }

    // Print raw accelerometer readings
    // Serial.print("Raw acceleration X: "); Serial.println(a.acceleration.x);
    // Serial.print("Raw acceleration Y: "); Serial.println(a.acceleration.y);
    // Serial.print("Raw acceleration Z: "); Serial.println(a.acceleration.z);

    estimated_pitch = g.gyro.x * 180 / PI;
    estimated_yaw = g.gyro.z * 180 / PI;

    float raw_height = bmp.readAltitude(SEA_LEVEL_PRESSURE);
    float raw_velocity = a.acceleration.z;
    /*Serial.println("raw height:");
    Serial.println(raw_height);
    Serial.println("raw velocity:");
    Serial.println(raw_velocity);*/

    float height = (heightKalmanFilter.updateEstimate(raw_height) - 44); // code 1st floor height for testing
    float estimated_velocity = velocityKalmanFilter.updateEstimate(raw_velocity);

    datapoint.estimated_altitude = height;

    Serial.print("Estimated height: ");
    Serial.print(height);
    Serial.print("Estimated velocity: ");
    Serial.println(estimated_velocity);
}

void connectWifi()
{
    WiFi.mode(WIFI_STA);
    // Connect to wifi
    WiFi.begin(ssid, password);
    while (WiFi.status() != WL_CONNECTED)
    {
        delay(1000);
        Serial.println("Connecting to WiFi...");
    }
    Serial.println("Connected to WiFi");
}

bool checkSensors()
{
    // performs reading on the bmp390
    if (!bmp.performReading())
    {
        Serial.println("Could not read from the BMP390, sensor checking failed.");
        return false;
    }

    // performs reading on the mpu6050
    sensors_event_t a, g, temp;
    mpu6050.getEvent(&a, &g, &temp);

    if (isnan(a.acceleration.x) || isnan(g.gyro.x))
    {
        Serial.println("MPU6050 readings failed, sensor checking failed");
        return false;
    }

    Serial.println("All sensors are nominal.");
    return true;
}

bool allSystemsGo()
{
    if (checkSensors())
    {
        Serial.println("All systems are go");
        return true;
    }
    else
    {
        Serial.println("Not go for launch, check for errors");
        return false;
    }
}

void motorIgnite()
{
    // logic for igniting the motor here
}

/*void pyroInit() {
    pinMode(IGNITION_PIN, OUTPUT);
    digitalWrite(IGNITION_PIN, LOW);
}*/
