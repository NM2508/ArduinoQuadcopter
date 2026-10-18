#include <Wire.h>
#include <SPI.h>
#include <nRF24L01.h>
#include <RF24.h>
#include <Servo.h>
#include <Adafruit_MPU6050.h>
#include <Adafruit_BMP280.h>
#include "Arduino_LED_Matrix.h"

//Basic sensor setup
const int MPU_addr = 0x68; 
RF24 radio(9, 10);
const byte address[6] = "00001";
int16_t dataArray[12]; // radio input array
Adafruit_MPU6050 mpu;
Adafruit_BMP280 bmp;

//Altitude calculation variables
const float SEA_LEVEL_PRESSURE = 1013.25; 
const int N = 1; //Increase for more accuracy but more lag, decrease for less lag but less accuracy 
float altBuffer[N];
int idx = 0;
bool filled = false;

//Motor configuration
Servo esc3; // Front‑Left  → pin3
Servo esc4; // Front‑Right → pin4
Servo esc5; // Rear‑Right  → pin5
Servo esc6; // Rear‑Left   → pin6

//Motor throttle presets
int Tpercent3 = 0, Tpercent4 = 0, Tpercent5 = 0, Tpercent6 = 0;

//PID controller variables
float Kp = 7.1;   // proportional gain
float Ki = 0.3;   // integral gain
float Kd = 0.3;   // derivative gain

float prevRollError = 0, prevPitchError = 0;
float integralRollError = 0, integralPitchError = 0;
float prevYawError, integralYawError = 0;

//LED Matrix setup for calibration status updates
ArduinoLEDMatrix matrix;

const uint32_t ALL_ON[] = {
  0xFFFFFFFF,
  0xFFFFFFFF,
  0xFFFFFFFF
};

// 1D array representing all 96 LEDs OFF
const uint32_t ALL_OFF[] = {
  0x00000000,
  0x00000000,
  0x00000000
};

//Failsafe variables
unsigned long lastSignalTime = 0;
const unsigned long FAILSAFE_TIMEOUT   = 1000; 
const int FAILSAFE_STEP = 15; 

//Timing variables
unsigned long lastSensorTime = 0;
unsigned long lastPIDTime = 0;

//Gyro and accelerometer variables
float roll = 0, pitch = 0, yaw = 0; 
float gyroXOffset = 0, gyroYOffset = 0, gyroZOffset = 0;
float accelXOffset = 0, accelYOffset = 0, accelZOffset = 0;

//Struct for sensor outputs
struct SensorData {
  float pitch, roll, yaw, altitude, temperature;
} sensorData;

void readSensors(){ 
    unsigned long currentTime = millis();
    float dt = (currentTime - lastSensorTime) / 1000.0; 
    lastSensorTime = currentTime;

    //MPU6050 sensor configuration
    Wire.beginTransmission(MPU_addr);
    Wire.write(0x3B);
    Wire.endTransmission(false);
    Wire.requestFrom(MPU_addr, 14, true);

    int16_t ax = (Wire.read() << 8 | Wire.read()) - accelXOffset;
    int16_t ay = (Wire.read() << 8 | Wire.read()) - accelYOffset;
    int16_t az = (Wire.read() << 8 | Wire.read()) - accelZOffset;
    Wire.read(); Wire.read(); 
    int16_t gx = (Wire.read() << 8 | Wire.read()) - gyroXOffset;
    int16_t gy = (Wire.read() << 8 | Wire.read()) - gyroYOffset;
    int16_t gz = (Wire.read() << 8 | Wire.read()) - gyroZOffset;

    float accelX = ax / 16384.0;
    float accelY = ay / 16384.0;
    float accelZ = az / 16384.0;

    float accelPitch = atan2(-accelX, sqrt(accelY * accelY + accelZ * accelZ)) * 180.0 / PI;
    float accelRoll  = atan2(accelY, accelZ) * 180.0 / PI;
    float accelYaw = atan2(accelY, accelX) * 180.0 / PI;

    float gyroX = gx / 131.0;
    float gyroY = gy / 131.0;
    float gyroZ = gz / 131.0;

    //Complementary filter
    pitch = 0.98 * (pitch + gyroX * dt) + 0.02 * accelPitch;
    roll  = 0.98 * (roll + gyroY * dt) + 0.02 * accelRoll;

    if (abs(pitch) > 5 || abs(roll) > 5) {
      yaw = 0.99 * (yaw + gyroZ * dt) + 0.01 * accelYaw;
    }     
    else {
      yaw += gyroZ * dt; // pure gyro when level
    }

    if (pitch > 180) pitch -= 360;
    else if (pitch < -180) pitch += 360;

    if (roll > 180) roll -= 360;
    else if (roll < -180) roll += 360;

    if (yaw > 180) yaw -= 360;
    else if (yaw < -180) yaw += 360;

    //Altitude calculation
  
    float rawAlt = bmp.readAltitude(SEA_LEVEL_PRESSURE);
    altBuffer[idx] = rawAlt;
    idx = (idx + 1) % N;

    float sum = 0;
    int count = filled ? N : idx;
    for (int i = 0; i < count; i++) sum += altBuffer[i];

if (idx == 0) filled = true;

    //Store results in struct
    sensorData.pitch = pitch;
    sensorData.roll = roll;
    sensorData.yaw = yaw;
    sensorData.altitude = sum / count;
    sensorData.temperature = bmp.readTemperature();

    Serial.print("P: ");   Serial.print(sensorData.pitch, 1);
    Serial.print(" | R: "); Serial.print(sensorData.roll, 1);
    Serial.print(" | Y: "); Serial.print(sensorData.yaw, 1);
    Serial.print(" | Alt: "); Serial.print(sensorData.altitude, 2); Serial.print("m");
    Serial.print(" | Temp: "); Serial.print(sensorData.temperature, 1); Serial.println("C");
}

void setup() {
  Serial.begin(115200);
  matrix.begin(); 
  Wire.begin();

  //Turning matrix lights on to tell that the void setup has started
  matrix.loadFrame(ALL_ON);
  
  esc3.attach(3, 1000, 2000);
  esc4.attach(4, 1000, 2000);
  esc5.attach(5, 1000, 2000);
  esc6.attach(6, 1000, 2000);

  esc3.writeMicroseconds(1000); 
  esc4.writeMicroseconds(1000); 
  esc5.writeMicroseconds(1000); 
  esc6.writeMicroseconds(1000); 

  delay(5000); 

  Wire.beginTransmission(MPU_addr);
  Wire.write(0x6B);
  Wire.write(0);
  Wire.endTransmission(true);

  mpu.begin();
  bmp.begin(0x76);

  radio.begin();
  radio.openReadingPipe(0, address);
  radio.setPALevel(RF24_PA_LOW);
  radio.startListening();

  bmp.setSampling(Adafruit_BMP280::MODE_NORMAL,     
                  Adafruit_BMP280::SAMPLING_X2,     
                  Adafruit_BMP280::SAMPLING_X16,    
                  Adafruit_BMP280::FILTER_X16,      
                  Adafruit_BMP280::STANDBY_MS_1);   

  Serial.println("Calibrating MPU6050... Keep drone perfectly flat and still.");
  delay(1000);
  
  int numReads = 500;
  long agxSum = 0, agySum = 0, agzSum = 0;
  long gcxSum = 0, gcySum = 0, gczSum = 0;

  for (int i = 0; i < numReads; i++) {
    Wire.beginTransmission(MPU_addr);
    Wire.write(0x3B);
    Wire.endTransmission(false);
    Wire.requestFrom(MPU_addr, 14, true);
    
    int16_t rax = Wire.read() << 8 | Wire.read();
    int16_t ray = Wire.read() << 8 | Wire.read();
    int16_t raz = Wire.read() << 8 | Wire.read();
    Wire.read(); Wire.read(); 
    int16_t rgx = Wire.read() << 8 | Wire.read();
    int16_t rgy = Wire.read() << 8 | Wire.read();
    int16_t rgz = Wire.read() << 8 | Wire.read();

    agxSum += rax; agySum += ray; agzSum += raz;
    gcxSum += rgx; gcySum += rgy; gczSum += rgz;
    delay(3);
  }
  
  accelXOffset = agxSum / numReads;
  accelYOffset = agySum / numReads;
  accelZOffset = (agzSum / numReads) - 16384; 

  gyroXOffset = gcxSum / (float)numReads;
  gyroYOffset = gcySum / (float)numReads;
  gyroZOffset = gczSum / (float)numReads;

  Serial.println("Calibration finished!");
  lastSensorTime = millis();
  lastPIDTime = millis();

  //Turning of the Matrix to tell that the setup has ended
  matrix.loadFrame(ALL_OFF);
}

void loop() {
  if (radio.available()) {
    radio.read(&dataArray, sizeof(dataArray));
    lastSignalTime = millis(); 

    for (int i = 0; i < 12; i++) {
      Serial.print(dataArray[i]);
      if (i < 11) Serial.print(", ");
    }
    
    // Update sensors
    readSensors(); 

    int cleanThrottle = map(dataArray[11], 1023, 0, 2000, 1000);
    cleanThrottle = constrain(cleanThrottle, 1000, 2000);

    float targetPitch = map(dataArray[1], 0, 1023, 30, -30);
    float targetRoll  = map(dataArray[0], 0, 1023, 30, -30);
    float targetYaw   = map(dataArray[4], 0, 1023, 90, -90);

    targetPitch = constrain(targetPitch, -30, 30);
    targetRoll  = constrain(targetRoll,  -30, 30);
    targetYaw   = constrain(targetYaw,   -90, 90);

    //Swaping roll and pitch because there was a bug in the code that switched the two during PID control
    float rollError  = targetRoll + sensorData.pitch;
    float pitchError = targetPitch + sensorData.roll;
    float yawError   = targetYaw - sensorData.yaw;

    // PID timing
    unsigned long currentPIDTime = millis();
    float dt = (currentPIDTime - lastPIDTime) / 1000.0;
    lastPIDTime = currentPIDTime;

    // Integrals
    integralRollError  += rollError * dt;
    integralPitchError += pitchError * dt;
    integralYawError   += yawError * dt;

    integralRollError  = constrain(integralRollError, -100, 100);
    integralPitchError = constrain(integralPitchError, -100, 100);
    integralYawError   = constrain(integralYawError, -100, 100);

    // Derivatives
    float dRollError  = (rollError - prevRollError) / dt;
    float dPitchError = (pitchError - prevPitchError) / dt;
    float dYawError   = (yawError - prevYawError) / dt;

    // PID outputs
    float rollCorrection  = Kp * rollError + Ki * integralRollError + Kd * dRollError;
    float pitchCorrection = Kp * pitchError + Ki * integralPitchError + Kd * dPitchError;
    float yawCorrection   = Kp * yawError + Ki * integralYawError + Kd * dYawError;

    prevRollError  = rollError;
    prevPitchError = pitchError;
    prevYawError   = yawError;

    // Motor mixing with yaw
    Tpercent3 = cleanThrottle - rollCorrection + pitchCorrection + yawCorrection; // Front-Left
    Tpercent4 = cleanThrottle - rollCorrection - pitchCorrection - yawCorrection; // Front-Right
    Tpercent5 = cleanThrottle + rollCorrection - pitchCorrection + yawCorrection + 40; // Rear-Right
    Tpercent6 = cleanThrottle + rollCorrection + pitchCorrection - yawCorrection; // Rear-Left

    Tpercent3 = constrain(Tpercent3, 1000, 2000);
    Tpercent4 = constrain(Tpercent4, 1000, 2000);
    Tpercent5 = constrain(Tpercent5, 1000, 2000);
    Tpercent6 = constrain(Tpercent6, 1000, 2000);

    esc3.writeMicroseconds(Tpercent3);
    esc4.writeMicroseconds(Tpercent4);
    esc5.writeMicroseconds(Tpercent5);
    esc6.writeMicroseconds(Tpercent6);

    if (cleanThrottle < 1030) {
       Tpercent3 = 1000; Tpercent4 = 1000; Tpercent5 = 1000; Tpercent6 = 1000;
       integralRollError = 0; integralPitchError = 0; integralYawError = 0;
    }

    Serial.print("Target Pitch/Roll/Yaw: "); 
    Serial.print(targetPitch, 0); 
    Serial.print("/"); 
    Serial.print(targetRoll, 0);
    Serial.print("/"); 
    Serial.print(targetYaw, 0);
    Serial.print(" | Outputs (3-6): "); 
    Serial.print(Tpercent3); Serial.print(", "); 
    Serial.print(Tpercent4); Serial.print(", "); 
    Serial.print(Tpercent5); Serial.print(", "); 
    Serial.print(Tpercent6); Serial.print("  ");
  }

  unsigned long elapsed = millis() - lastSignalTime;

  if (elapsed > FAILSAFE_TIMEOUT) {
    if (Tpercent3 > 1000 or Tpercent4 > 1000 or Tpercent5 > 1000 or Tpercent6 > 1000){
      Tpercent3 = max(Tpercent3 - 1, 1000);
      Tpercent4 = max(Tpercent4 - 1, 1000);
      Tpercent5 = max(Tpercent5 - 1, 1000);
      Tpercent6 = max(Tpercent6 - 1, 1000);

      Tpercent3 = constrain(Tpercent3, 1000, 2000);
      Tpercent4 = constrain(Tpercent4, 1000, 2000);
      Tpercent5 = constrain(Tpercent5, 1000, 2000);
      Tpercent6 = constrain(Tpercent6, 1000, 2000);

      delay(FAILSAFE_STEP);
      Serial.println("FAILSAFE: Throttle reducing...");
    }
  } 

  esc3.writeMicroseconds(Tpercent3);
  esc4.writeMicroseconds(Tpercent4);
  esc5.writeMicroseconds(Tpercent5);
  esc6.writeMicroseconds(Tpercent6);
}
