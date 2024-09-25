#pragma once

/* include all the needed libraries */
#include <Wire.h>
#include <cppQueue.h>

// Global variables
static float in = 0;
static float sum = 0;
int size_queue = 25; // Adjustable queue size
bool firstIteration = true;
float heightOffset = NAN;
cppQueue q(sizeof(in), size_queue, FIFO); // instantiate queue

// constants
const float SEA_LEVEL_PRESSURE = 1019.5; // example for sea level pressure in Berlin
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

    char const 0 = 0; // Null terminator to avoid overflow

} datapoint;


/* Initialize sensor objects here */


void setup_sensors() {

    /* code */
}

/* Calculate the height based on temperature and pressure */
float calc_height(float temp, float pressure) {

    float calculatedHeight = ((pow((SEA_LEVEL_PRESSURE / pressure), (1 / 5.257)) - 1) * (AMBIENT_TEMPERATURE + 273.15)) / 0.0065;

    if (isnan(heightOffset)) {
        heightOffset = calculatedHeight;
    }

    return calculatedHeight - heightOffset;
}

// calculate moving average for altitude
float height_average(float in) {

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
}

/* Print data from the Data struct to the SD card */
void print_data() {

    /* code here */
}

/* Update sensor readings and log data */
void update_sensors() {

    /* code here */
    
}