#pragma once 

#include <SPI.h>
#include <SD.h>

#define SD_CS_PORT PA4 // change to the chip select port from the esp32

/* MACROS */
#define PRINT_VALUE(value)                          \
    Serial.print(value); Serial.print(",");         \
    DATA_FILE.print(value); DATA_FILE.print(","); 

#define PRINTLN_VALUE(value)                        \
    Serial.println(value);                          \
    DATA_FILE.println(value);                       \
    DATA_FILE.flush();

// Global FILE-objects for SD access
File LOG_FILE;
File DATA_FILE;

