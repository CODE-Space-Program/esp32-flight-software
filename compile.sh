bin/arduino-cli core update-index
bin/arduino-cli core install esp32:esp32

bin/arduino-cli lib install "Adafruit BMP3XX Library"
bin/arduino-cli lib install "Queue"
bin/arduino-cli lib install "simpleKalmanFilter"
bin/arduino-cli lib install "Adafruit MPU6050"
bin/arduino-cli lib install "ESP32Servo"
bin/arduino-cli lib install "Adafruit PWM Servo Driver Library"
bin/arduino-cli lib install "ArduinoJson"

bin/arduino-cli compile --fqbn esp32:esp32:esp32 main