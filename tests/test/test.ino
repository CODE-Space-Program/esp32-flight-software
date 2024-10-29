#include <Wire.h>
#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BMP3XX.h>
#include <ESP32Servo.h>

// MPU6050 and BMP390 objects
Adafruit_MPU6050 mpu;
Adafruit_BMP3XX bmp;

// Servo objects
Servo servoPitch;
Servo servoYaw;

// Servo control pins
const int servoPitchPin = 16;
const int servoYawPin = 17;

// PID control parameters for TVC
float Kp = 1.0, Ki = 0.0, Kd = 0.0;
float prevErrorPitch = 0, prevErrorYaw = 0;
float integralPitch = 0, integralYaw = 0;

void setup() {
  Serial.begin(9600);
  Wire.begin();

  // Initialize the MPU6050
  if (!mpu.begin()) {
    Serial.println("MPU6050 not connected!");
    while (1);
  }

  // Initialize the BMP390
  if (!bmp.begin_I2C()) {
    Serial.println("BMP390 not connected!");
    while (1);
  }
  bmp.setTemperatureOversampling(BMP3_OVERSAMPLING_8X);
  bmp.setPressureOversampling(BMP3_OVERSAMPLING_4X);
  bmp.setIIRFilterCoeff(BMP3_IIR_FILTER_COEFF_3);
  bmp.setOutputDataRate(BMP3_ODR_50_HZ);

  // Initialize the servos
  servoPitch.attach(servoPitchPin);
  servoYaw.attach(servoYawPin);

  Serial.println("Setup complete");
}

void loop() {
  // Read sensor data
  sensors_event_t a, g, temp;
  mpu.getEvent(&a, &g, &temp);

  // Read pitch and yaw from the MPU6050
  float pitch = g.gyro.y * 180 / PI;
  float yaw = g.gyro.z * 180 / PI;

  // Read altitude from the BMP390
  if (!bmp.performReading()) {
    Serial.println("BMP390 reading failed!");
  } else {
    float altitude = bmp.readAltitude(1013.25);  // Adjust to your sea level pressure
    Serial.print("Altitude: ");
    Serial.println(altitude);
  }

  // Print pitch and yaw values for debugging
  Serial.print("Pitch: ");
  Serial.print(pitch);
  Serial.print(", Yaw: ");
  Serial.println(yaw);

  // Move the servos based on the pitch and yaw (simple proportional control)
  controlTVC(pitch, yaw);

  delay(100);  // Adjust delay for real-time performance
}

// TVC control function (Proportional Control)
void controlTVC(float pitch, float yaw) {
  // PID control for pitch
  float errorPitch = 0 - pitch;  // Target pitch is 0
  integralPitch += errorPitch;
  float derivativePitch = errorPitch - prevErrorPitch;
  float controlSignalPitch = Kp * errorPitch + Ki * integralPitch + Kd * derivativePitch;
  prevErrorPitch = errorPitch;

  // PID control for yaw
  float errorYaw = 0 - yaw;  // Target yaw is 0
  integralYaw += errorYaw;
  float derivativeYaw = errorYaw - prevErrorYaw;
  float controlSignalYaw = Kp * errorYaw + Ki * integralYaw + Kd * derivativeYaw;
  prevErrorYaw = errorYaw;

  // Map the control signal to servo positions
  int pitchServoPos = constrain(90 + controlSignalPitch, 0, 180);
  int yawServoPos = constrain(90 + controlSignalYaw, 0, 180);

  // Move servos
  servoPitch.write(pitchServoPos);
  servoYaw.write(yawServoPos);

  Serial.print("Pitch Servo: ");
  Serial.print(pitchServoPos);
  Serial.print(", Yaw Servo: ");
  Serial.println(yawServoPos);
}
