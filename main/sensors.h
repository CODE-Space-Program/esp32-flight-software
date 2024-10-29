#pragma once

/* include all the needed libraries */
#include <Adafruit_BMP3XX.h>
#include <Wire.h>
#include <cppQueue.h>
#include "KalmanFilter.h"
#include <Adafruit_MPU6050.h>

// Global variables
static float in = 0;
static float sum = 0;
int size_queue = 25; // Adjustable queue size
bool firstIteration = true;
float heightOffset = NAN;
cppQueue q(sizeof(in), size_queue, FIFO); // instantiate queue

extern float estimated_pitch;
extern float estimated_yaw;

// constants
const float SEA_LEVEL_PRESSURE = 1023.6; // example for sea level pressure in Berlin
const float AMBIENT_TEMPERATURE = 5; // To be changed on the day
const unsigned long updateInterval = 100; // Sensor update interval (ms)
unsigned long lastUpdateTime = 0;


// Data Structure for a single datapoint
struct Data {

    long time; // time in ms

    struct Gyro {

        float x;
        float y;
        float z;

    } gyro; // Gyroscope data

    struct Acc {

        float x;
        float y;
        float z;

    } acc; // Accelerometer data

    float pressure;         // pressure in mbar
    float temperatureMS;    // Temperature in Celsius
    float height;           // Height in meters
    float estimated_altitude_average;   // Filtered height

    char const null_terminator = 0; // Null terminator to avoid overflow

} datapoint;


/* Initialize sensor objects here */
Adafruit_MPU6050 mpu6050;
Adafruit_BMP3XX bmp;
/* Initialize kalman filter */
KalmanFilter kalman_filter;


void setup_sensors() {

    Wire.begin(21, 22);
    /* Initialize I2C Commnication */
    if (!mpu6050.begin()) {
      Serial.println("Could not find a valid MPU6050 sensor, check wiring");
      while(1);
    }

    /* Initialize BMP390 */
    if (!bmp.begin_I2C()) {
        Serial.println("Could not find a valid BMP390 sensor, check wiring!");
        while(1);
    }

    // Recommended initialization by the manufacturer
    bmp.setTemperatureOversampling(BMP3_OVERSAMPLING_8X);
    bmp.setPressureOversampling(BMP3_OVERSAMPLING_4X);
    bmp.setIIRFilterCoeff(BMP3_IIR_FILTER_COEFF_3);
    bmp.setOutputDataRate(BMP3_ODR_100_HZ);

    /* Initialize MPU6050 */
    mpu6050.setAccelerometerRange(MPU6050_RANGE_8_G);
    mpu6050.setGyroRange(MPU6050_RANGE_500_DEG);
    mpu6050.setFilterBandwidth(MPU6050_BAND_44_HZ);

    float initial_height = bmp.readAltitude(SEA_LEVEL_PRESSURE); // adjust these based on the sea level
    float initial_velocity = 0.0;
    kalman_filter.init(initial_height, initial_velocity, 0.01); // dt = 0.01 (10ms)
    
}

/* Calculate the height based on temperature and pressure */
/* float calc_height(float temp, float pressure) {

    float calculatedHeight = ((pow((SEA_LEVEL_PRESSURE / pressure), (1 / 5.257)) - 1) * (AMBIENT_TEMPERATURE + 273.15)) / 0.0065;

    if (isnan(heightOffset)) {
        heightOffset = calculatedHeight;
    }

    return calculatedHeight - heightOffset;
} */

// calculate moving average for altitude
/* float height_average(float in) {

    float out;
    q.push(&in);
    if (q.getCount() < size_queue) {
        sum += in;
    }
    if (q.getCount() == size_queue) {
        sum += in;
        q.pop(&out);
        sum -= out;
        return sum / size_queue;
    } else {
        return 0;
    }
} */

/* Print data from the Data struct to the SD card */
void print_data() {

    /* code here */
}

float estimated_pitch = 0;
float estimated_yaw = 0;

/* Update sensor readings and log data */
void update_sensors() {

    sensors_event_t a, g, temp;
    mpu6050.getEvent(&a, &g, &temp);

    // BMP390 sensor reading
    if (!bmp.performReading()) {
        Serial.println("Failed to read from the BMP390 sensor!");
        return;
    }

    // Print raw accelerometer readings
    Serial.print("Raw acceleration X: "); Serial.println(a.acceleration.x);
    Serial.print("Raw acceleration Y: "); Serial.println(a.acceleration.y);
    Serial.print("Raw acceleration Z: "); Serial.println(a.acceleration.z);

    estimated_pitch = g.gyro.y * 180 / PI;
    estimated_yaw = g.gyro.z * 180 / PI;
    float accZ = (a.acceleration.z - 9.81);

    if (abs(accZ) < 0.5) {
      accZ = 0;
    }

    kalman_filter.predict(accZ);

    float measured_height = bmp.readAltitude(SEA_LEVEL_PRESSURE);
    kalman_filter.update(measured_height);

    float estimated_height = kalman_filter.get_estimated_height();
    float estimated_velocity = kalman_filter.get_estimated_velocity();

    Serial.print("Estimated height: ");
    Serial.print(estimated_height);
    Serial.print("Estimated velocity: ");
    Serial.println(estimated_velocity);
}