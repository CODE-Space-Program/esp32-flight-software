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

#define ASCENDING_MOTOR_IGNITION_PIN 5 // change this to the GPIO pin connected to the ESP32
#define DESCENDING_MOTOR_IGNITION_PIN 6 

// sample freq in Hz
#define FREQ 120.0

// Wifi connection variables
const char *ssid = "VIRUS";
//const char *password = "Code!University";

extern float estimated_pitch;
extern float estimated_yaw;

extern float estimated_height;
extern float estimated_velocity;

float estimated_height = 0;
float estimated_velocity = 0;
// constants
extern const float SEA_LEVEL_PRESSURE = 1020.57; // example for sea level pressure in Berlin
extern const float AMBIENT_TEMPERATURE = 22.4;  // To be changed on the day
const float G = 9.81;                           // gravity constant
const unsigned long updateInterval = 100;       // Sensor update interval (ms)
unsigned long lastUpdateTime = 0;

// Data Structure for a single datapoint
struct Data
{

    long time; // time in ms

    /*struct Gyro
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

    } acc; // Accelerometer data*/

    //float pressure;    // pressure in mbar
    //float temperature; // Temperature in Celsius
    float raw_altitude;
    float estimated_altitude; // Filtered height
    float velocity;
    float estimated_pitch;
    float estimated_yaw;

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

/* Print data from the Data struct to the SD card */
void print_data()
{

    /* code here */
}


const float alpha = 0.98; // Complementary filter constant
unsigned long lastTime = 0;
float estimated_pitch = 0;
float estimated_yaw = 0;

double gx = 0, gy = 0, gz = 0;
double gyrX = 0, gyrY = 0, gyrZ = 0;
int16_t accX = 0, accY = 0, accZ = 0;

/* Update sensor readings and log data */
void update_sensors()
{

    double ax, ay, az;
    unsigned long start_time, end_time;

    start_time = millis();

    sensors_event_t a, g, temp;
    mpu6050.getEvent(&a, &g, &temp);

    // Print raw accelerometer readings
    // Serial.print("Raw acceleration X: "); Serial.println(a.acceleration.x);
    // Serial.print("Raw acceleration Y: "); Serial.println(a.acceleration.y);
    // Serial.print("Raw acceleration Z: "); Serial.println(a.acceleration.z);

    accX = a.acceleration.x;
    accY = a.acceleration.y;
    accZ = a.acceleration.z;

    gyrX = g.gyro.x;
    gyrY = g.gyro.y;
    gyrZ = g.gyro.z;

    // angles based on accelerometer
    ay = atan2(accX, sqrt(pow(accY, 2) + pow(accZ, 2))) * 180 / M_PI;
    ax = atan2(accY, sqrt(pow(accX, 2) + pow(accZ, 2))) * 180 / M_PI;
    az = atan2(accZ, sqrt(pow(accX, 2) + pow(accY, 2))) * 180 / M_PI;

    // angles based on the gyroscope
    gx = gx + gyrX / FREQ;
    gy = gy - gyrY / FREQ;
    gz = gz + gyrZ / FREQ;

    // complementary filter
    gx = gx * 0.96 + ax * 0.04;
    gz = gz * 0.96 + az * 0.04;

    //unsigned long currentTime = millis();
    //float dt = (currentTime - lastTime) / 1000.0; // converts to seconds
    //lastTime = currentTime;

    //float gyroPitchRate = g.gyro.x * 180 / PI; // convert to degrees per second
    //float gyroYawRate = g.gyro.y * 180 / PI;

    //estimated_pitch = alpha * (estimated_pitch + gyroPitchRate * dt) + (1 - alpha) * atan2(a.acceleration.y, a.acceleration.z) * 180 / PI;
    //estimated_yaw = alpha * (estimated_yaw + gyroYawRate * dt) + (1 - alpha) * atan2(a.acceleration.x, a.acceleration.z) * 180 / PI;

    float raw_height = bmp.readAltitude(SEA_LEVEL_PRESSURE);
    float raw_velocity = a.acceleration.z;
    float raw_velocity_ms2 = raw_velocity * G; // convert to m/s2
    /*Serial.println("raw height:");
    Serial.println(raw_height);
    Serial.println("raw velocity:");
    Serial.println(raw_velocity);*/

    float height = (heightKalmanFilter.updateEstimate(raw_height)); // code 1st floor height for testing
    float estimated_velocity = velocityKalmanFilter.updateEstimate(raw_velocity_ms2);

    datapoint.raw_altitude = raw_height;
    datapoint.estimated_altitude = height;
    datapoint.velocity = estimated_velocity;
    //datapoint.gyro.x = g.gyro.x;
    //datapoint.gyro.y = g.gyro.y;
    //datapoint.gyro.z = g.gyro.z;
    datapoint.estimated_pitch = gx;
    datapoint.estimated_yaw = gz;

    end_time = millis();

    delay(((1 / FREQ) * 1000) - (end_time - start_time));

    Serial.println("pitch angle:  ");
    Serial.println(gx);
    Serial.println("yaw angle");
    Serial.println(gz);
    Serial.print("Estimated height: ");
    Serial.print(height);
    Serial.print("Estimated velocity: ");
    Serial.println(estimated_velocity);
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

void descendingMotorIgnite() {
    digitalWrite(DESCENDING_MOTOR_IGNITION_PIN, HIGH);
    delay(1000);
    digitalWrite(DESCENDING_MOTOR_IGNITION_PIN, LOW);
    Serial.println("Descending motor ignited");
}


void pyroInit() {
    pinMode(ASCENDING_MOTOR_IGNITION_PIN, OUTPUT);
    digitalWrite(ASCENDING_MOTOR_IGNITION_PIN, LOW);

    //pinMode(DESCENDING_MOTOR_IGNITION_PIN, OUTPUT);
    //digitalWrite(DESCENDING_MOTOR_IGNITION_PIN, LOW);
    Serial.println("Pyro channels initialized");
}

double gyrXoffs = 0.0, gyrYoffs = 0.0, gyrZoffs = 0.0;

void calibrateMpu6050() {
// Calibration routine
  int x;
  long xSum = 0, ySum = 0, zSum = 0;
  int num = 500;

  for (x = 0; x < num; x++) {
    sensors_event_t a, g, temp;
    mpu6050.getEvent(&a, &g, &temp);

    xSum += g.gyro.x;
    ySum += g.gyro.y;
    zSum += g.gyro.z;
  }
  gyrXoffs = xSum / num;
  gyrYoffs = ySum / num;
  gyrZoffs = zSum / num;   
}
