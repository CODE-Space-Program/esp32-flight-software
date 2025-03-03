#pragma once

/* include all the needed libraries */
#include <Adafruit_BMP3XX.h>
#include <Wire.h>
#include <cppQueue.h>
#include <SimpleKalmanFilter.h>
#include <Adafruit_MPU6050.h>
#include <WiFiClient.h>

// Server connection variables
const char* serverIP = "192.168.14.3"; // replace with server ip
uint16_t serverPort = 8080;

WiFiClient client;

// Global variables
static float in = 0;
static float sum = 0;
int size_queue = 25; // Adjustable queue size
bool firstIteration = true;
float heightOffset = NAN;
cppQueue q(sizeof(in), size_queue, FIFO); // instantiate queue

extern float estimated_pitch;
extern float estimated_yaw;

extern float estimated_height;
extern float estimated_velocity;

// constants
extern const float SEA_LEVEL_PRESSURE = 1000.17; // example for sea level pressure in Berlin
extern const float AMBIENT_TEMPERATURE = 15; // To be changed on the day
const float G = 9.81; // gravity constant
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
// SimpleKalmanFilter for height and velocity
SimpleKalmanFilter heightKalmanFilter(0.03, 1.0, 0.01); // tuning values: (e_mea, e_est, q)
SimpleKalmanFilter velocityKalmanFilter(0.03, 1.0, 0.01);


void setup_sensors() {

    Wire.begin(21, 22);
    /* Initialize I2C Commnication */
    if (!mpu6050.begin()) {
      Serial.println("Could not find a valid MPU6050 sensor, check wiring");
      while(1);
    }

    /* Initialize BMP390 */
    if (!bmp.begin_I2C(0x77)) {
        Serial.println("Could not find a valid BMP390 sensor, check wiring!");
        while(1);
    } else {
      Serial.println("BMP390 Initialized successfully!");
    }

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
    //Serial.print("Raw acceleration X: "); Serial.println(a.acceleration.x);
    //Serial.print("Raw acceleration Y: "); Serial.println(a.acceleration.y);
    //Serial.print("Raw acceleration Z: "); Serial.println(a.acceleration.z);

    estimated_pitch = g.gyro.x * 180 / PI;
    estimated_yaw = g.gyro.z * 180 / PI;

    float raw_height = bmp.readAltitude(SEA_LEVEL_PRESSURE);
    float raw_velocity = a.acceleration.z;

    float estimated_height = (heightKalmanFilter.updateEstimate(raw_height) - 33); // code 1st floor height for testing
    float estimated_velocity = velocityKalmanFilter.updateEstimate(raw_velocity);
    
    Serial.print("Estimated height: ");
    Serial.print(estimated_height);
    Serial.print("Estimated velocity: ");
    Serial.println(estimated_velocity);

    // Send data to server
    if (client.connect(serverIP, serverPort)) {
       String postData = "Height=" + String(estimated_height) + "&Velocity=" + String(estimated_velocity);
       client.println("POST /data HTTP/1.1");
       client.println("Host: " + String(serverIP));
       client.println("Content-Type: application/x-www-form-urlencoded");
       client.println("Content-Length: " + String(postData.length()));
       client.println();
       client.print(postData);
       client.stop();

        // Read response (optional)
        while (client.connected()) {
            String line = client.readStringUntil('\n');
            if (line == "\r") {
                break;
            }
        }
        client.stop();
    } else {
        Serial.println("Connection to server failed");
    }
}

