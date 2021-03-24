// includes

#include <Adafruit_Sensor_Calibration.h>
#include <Adafruit_AHRS.h>
#include <math.h>
Adafruit_Sensor *accelerometer, *gyroscope, *magnetometer;

#include "LSM9DS.h"           // LSM9DS1 or LSM9DS0
#include <Servo.h>

Servo sAileronLeft;
Servo sAileronRight;

// definitions
#define FILTER_UPDATE_RATE_HZ 100
#define PRINT_EVERY_N_UPDATES 10

#define rad2deg 180.0/3.141592653589793238463
#define mu 0.01

#define PGAIN 1.0

int pinControlAileronLeft = 2;
int pinControlAileronRight = 3;

float angleAileronLimits[2] = {0.0, 180.0};
int pwmRangeAileron[2] = {994, 1980};
float aileronAngleLimits[2] = {-45.0, 45.0};

float aileronZero = 90.0;
float aileronTarget[2] = {0.0, 0.0};

float getAileronAngle(int pwm) {
  return angleAileronLimits[0] + (angleAileronLimits[1] - angleAileronLimits[0]) * (pwm - pwmRangeAileron[0]) / (pwmRangeAileron[1] - pwmRangeAileron[0]);
}

float SQ(float x) {
  return x * x;
}

int8_t sgn(float val) {
 if (val < 0) return -1;
 if (val > 0) return 1;
 return 1;
}

float applyAileronLimits(float angle) {

  if (angle < aileronAngleLimits[0])
    return aileronZero + aileronAngleLimits[0];
  else if (angle > aileronAngleLimits[1])
    return aileronZero + aileronAngleLimits[1];
  else
    return aileronZero + angle;
}

float * getPitchRoll() {
  
  static float pitchRoll[2];
  
  // Read the motion sensors
  sensors_event_t accel, gyro, mag;
  accelerometer->getEvent(&accel);
  gyroscope->getEvent(&gyro);
  magnetometer->getEvent(&mag);

  float accel_norm = sqrt(SQ(accel.acceleration.x) + SQ(accel.acceleration.y) + SQ(accel.acceleration.z));
  float Gx, Gy, Gz;

  Gx = accel.acceleration.x / accel_norm;
  Gy = accel.acceleration.y / accel_norm;
  Gz = accel.acceleration.z / accel_norm;

  pitchRoll[0] = -asin(accel.acceleration.x / 9.81) * rad2deg;
  pitchRoll[1] = asin(accel.acceleration.y / (9.81 * cos(pitchRoll[0] / rad2deg))) * rad2deg;

  return pitchRoll;
  
}


uint32_t timestamp;

void setup() {
  Serial.begin(115200);
  while (!Serial) yield();

  sAileronLeft.attach(pinControlAileronLeft);
  sAileronRight.attach(pinControlAileronRight);

  if (!init_sensors()) {
    Serial.println("Failed to find sensors");
    while (1) delay(10);
  }
  
  setup_sensors();
  
  Wire.setClock(400000); // 400KHz
  
}


void loop() {

  float *pitchRoll;

  // IMU sample rate
  if ((millis() - timestamp) < (1000 / FILTER_UPDATE_RATE_HZ)) {
    return;
  }
  
  timestamp = millis();
  
  pitchRoll = getPitchRoll();

  // pcontrol pitch
  float roll_offset = pitchRoll[1] - aileronTarget[1];

  float vAileronLeft = applyAileronLimits(roll_offset * PGAIN);
  float vAileronRight = applyAileronLimits(roll_offset * PGAIN);

  sAileronLeft.write(vAileronLeft);
  sAileronRight.write(vAileronRight);
 
  Serial.print("PITCH = "); Serial.print(pitchRoll[0], 4); Serial.println("");
  Serial.print("ROLL = "); Serial.print(pitchRoll[1], 4); Serial.println("");
  Serial.println(vAileronLeft);
  Serial.println(""); Serial.println(""); Serial.println("");
  


}
