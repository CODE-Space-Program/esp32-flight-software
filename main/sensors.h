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

#define ASCENDING_MOTOR_IGNITION_PIN 5
#define DESCENDING_MOTOR_IGNITION_PIN 35

// sample freq in Hz
#define FREQ 400.0

// Wifi connection variables
const char *ssid = "VIRUS";
// const char *password = "Code!University";

extern float estimated_pitch;
extern float estimated_yaw;

extern float estimated_height;
extern float estimated_velocity;

float estimated_height = 0;
float estimated_velocity = 0;
// constants
extern const float SEA_LEVEL_PRESSURE = 766; // example for sea level pressure in Berlin
extern const float AMBIENT_TEMPERATURE = 22.4;  // To be changed on the day
const float G = 9.81;                           // gravity constant
const unsigned long updateInterval = 100;       // Sensor update interval (ms)
unsigned long lastUpdateTime = 0;

// Data Structure for a single datapoint
struct Data
{
    String state = "Ready";

    long time; // time in ms

    // float pressure;    // pressure in mbar
    float temperature; // Temperature in Celsius
    float raw_altitude;
    float estimated_altitude; // Filtered height
    float velocity;
    float estimated_pitch;
    float estimated_yaw;
    float estimated_roll;

    char const null_terminator = 0; // Null terminator to avoid overflow

    float nominalYawServoDegrees = 0;
    float nominalPitchServoDegrees = 0;

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
    Wire.setClock(400000);
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

    delay(1000);

    // Recommended initialization by the manufacturer
    bmp.setTemperatureOversampling(BMP3_OVERSAMPLING_8X);
    bmp.setPressureOversampling(BMP3_OVERSAMPLING_4X);
    bmp.setIIRFilterCoeff(BMP3_IIR_FILTER_COEFF_3);
    bmp.setOutputDataRate(BMP3_ODR_100_HZ);

    /* Initialize MPU6050 */
    mpu6050.setAccelerometerRange(MPU6050_RANGE_8_G);
    mpu6050.setGyroRange(MPU6050_RANGE_500_DEG);
    mpu6050.setFilterBandwidth(MPU6050_BAND_44_HZ);
    mpu6050.setSampleRateDivisor(0); // 1 Khz
}

float estimated_pitch = 0;
float estimated_yaw = 0;

double gx = 0, gy = 0, gz = 0;
double gyrX = 0, gyrY = 0, gyrZ = 0;
double accX = 0, accY = 0, accZ = 0;

int printCounter = 0;

/* Update sensor readings and log data */
void update_sensors()
{

    double ax, ay, az;
    static unsigned long lastTime = 0; // Store previous loop time
    unsigned long currentTime = micros();
    float dt = (currentTime - lastTime) / 1000000.0;  // Convert µs to seconds
    lastTime = currentTime;

    if (printCounter % 100 == 0) {  // Print every 10 iterations (~40Hz instead of 400Hz)
        Serial.print("dt: ");
        Serial.println(dt, 6);
    }
    printCounter++;

    sensors_event_t a, g, temp;
    mpu6050.getEvent(&a, &g, &temp);

    // Print raw accelerometer readings
    // Serial.print("Raw acceleration X: "); Serial.println(a.acceleration.x);
    // Serial.print("Raw acceleration Y: "); Serial.println(a.acceleration.y);
    // Serial.print("Raw acceleration Z: "); Serial.println(a.acceleration.z);

    accX = a.acceleration.x - 0.41;
    accY = a.acceleration.y + 0.14; // calibration values based on inherent innacuracies in BPM sensor
    accZ = a.acceleration.z + 0.72;

    gyrX = g.gyro.x;
    gyrY = g.gyro.y;
    gyrZ = g.gyro.z;

    // angles based on accelerometer
    ax = atan2(-accX, sqrt(pow(accY, 2) + pow(accZ, 2))) * 180 / M_PI;
    ay = atan2(accY, sqrt(pow(accX, 2) + pow(accZ, 2))) * 180 / M_PI;
    az = atan2(accZ, sqrt(pow(accX, 2) + pow(accY, 2))) * 180 / M_PI;

    // angles based on the gyroscope
    gx = gx + gyrX * dt;
    gy = gy + gyrY * dt;
    gz = gz + gyrZ * dt;

    // complementary filter
    gx = gx * 0.96 + ax * 0.04;
    gz = gz * 0.96 + az * 0.04;

    float raw_height = bmp.readAltitude(SEA_LEVEL_PRESSURE);
    float raw_velocity = a.acceleration.y;
    float raw_velocity_ms2 = raw_velocity * G; // convert to m/s2

    float height = (heightKalmanFilter.updateEstimate(raw_height)); // code 1st floor height for testing
    float estimated_velocity = velocityKalmanFilter.updateEstimate(raw_velocity_ms2);

    datapoint.raw_altitude = raw_height;
    datapoint.estimated_altitude = height;
    datapoint.velocity = estimated_velocity;
    datapoint.estimated_yaw = gx;
    datapoint.estimated_pitch = gz;
    datapoint.estimated_roll = gy;

    //end_time = millis();

    //delay(((1 / FREQ) * 1000) - (end_time - start_time));

    //Serial.println("yaw angle:  ");
    //Serial.println(gx);
    //Serial.println("pitch angle");
    //Serial.println(gz);
    //Serial.print("Estimated height: ");
    //Serial.print(height);
    //Serial.print("Estimated velocity: ");
    //Serial.println(estimated_velocity);
}

void connectWifi()
{
    WiFi.mode(WIFI_STA);
    // Connect to wifi
    WiFi.begin(ssid);
    while (WiFi.status() != WL_CONNECTED)
    {
        delay(1000);
        Serial.println("Connecting to WiFi...");
    }
    Serial.println("Connected to WiFi");
    Serial.println(WiFi.RSSI());
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

bool allSystemsCheck()
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

void ascendingMotorIgnite()
{
    digitalWrite(ASCENDING_MOTOR_IGNITION_PIN, HIGH);
    delay(1000);
    digitalWrite(ASCENDING_MOTOR_IGNITION_PIN, LOW);
    Serial.println("Ascending Motor ignited");
}

void descendingMotorIgnite()
{
    digitalWrite(DESCENDING_MOTOR_IGNITION_PIN, HIGH);
    delay(1000);
    digitalWrite(DESCENDING_MOTOR_IGNITION_PIN, LOW);
    Serial.println("Descending motor ignited");
}

void pyroInit()
{
    pinMode(ASCENDING_MOTOR_IGNITION_PIN, OUTPUT);
    digitalWrite(ASCENDING_MOTOR_IGNITION_PIN, LOW);

    pinMode(DESCENDING_MOTOR_IGNITION_PIN, OUTPUT); // uncomment this before flight
    digitalWrite(DESCENDING_MOTOR_IGNITION_PIN, LOW);
    Serial.println("Pyro channels initialized");
}

double gyrXoffs = 0.0, gyrYoffs = 0.0, gyrZoffs = 0.0;

void calibrateMpu6050()
{
    // Calibration routine
    int x;
    long xSum = 0, ySum = 0, zSum = 0;
    int num = 500;

    for (x = 0; x < num; x++)
    {
        sensors_event_t a, g, temp;
        mpu6050.getEvent(&a, &g, &temp);

        xSum += g.gyro.x;
        ySum += g.gyro.y;
        zSum += g.gyro.z;
    }
    gyrXoffs = xSum / num;
    gyrYoffs = ySum / num;
    gyrZoffs = zSum / num;
    Serial.println("MPU6050 calibrated...");
}
