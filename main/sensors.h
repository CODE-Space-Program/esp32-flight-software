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